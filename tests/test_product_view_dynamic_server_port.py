"""Exercise the production Qt process owner and the real Python HTTP child."""
import json
from pathlib import Path
import select
import shlex
import socket
import subprocess
import time
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def owner_binary(tmp_path_factory):
    folder = tmp_path_factory.mktemp("product-view-owner")
    source = folder / "owner.cpp"
    source.write_text(r'''
#include "owned_product_view_server.hpp"
#include <QCoreApplication>
#include <QSocketNotifier>
#include <iostream>
int main(int argc, char ** argv) {
  QCoreApplication app(argc, argv);
  OwnedProductViewServer owner;
  auto output = [](const QJsonObject & object) {
    std::cout << QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString() << std::endl;
  };
  owner.ready = [&](int port) { output({{"event", "ready"}, {"port", port}, {"pid", owner.pid()}}); };
  owner.failed = [&](const QString & detail) { output({{"event", "failed"}, {"detail", detail}}); };
  const QString root = QString::fromLocal8Bit(argv[1]);
  QSocketNotifier input(0, QSocketNotifier::Read);
  QObject::connect(&input, &QSocketNotifier::activated, &app, [&]() {
    std::string command;
    std::getline(std::cin, command);
    if (command == "start") owner.start(root);
    if (command == "replace") {
      owner.start(root + "/missing-old-request");
      owner.start(root);
    }
    if (command == "stop") { owner.stop(); output({{"event", "stopped"}}); }
    if (command == "quit" || std::cin.eof()) app.quit();
  });
  return app.exec(); // stack destruction must stop and reap the owned child
}
''')
    binary = folder / "owner"
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "Qt5Core"], text=True))
    subprocess.run(["g++", "-std=c++17", "-fPIC", str(source),
                    "-I" + str(ROOT / "workcell_builder/workcell_builder/gui"),
                    *flags, "-o", str(binary)], check=True)
    return binary


def receive(process):
    assert select.select([process.stdout], [], [], 12)[0], "Qt owner did not report an event"
    line = process.stdout.readline()
    assert line, f"Qt owner exited: {process.poll()}"
    return json.loads(line)


def command(process, text):
    process.stdin.write(text + "\n")
    process.stdin.flush()


def gone(pid):
    for _ in range(100):
        path = Path(f"/proc/{pid}/stat")
        if not path.exists() or path.read_text().split()[2] == "Z":
            return
        time.sleep(.02)
    pytest.fail(f"owned child {pid} survived shutdown")


def test_owned_port_restart_stale_callbacks_and_destructor(owner_binary):
    occupant = socket.socket()
    try:
        occupant.bind(("127.0.0.1", 8765))
        occupant.listen()
    except OSError:
        # An existing workstation listener is deliberately left untouched.
        occupant.close()
    process = subprocess.Popen([str(owner_binary), str(ROOT)], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True)
    try:
        command(process, "start")
        first = receive(process)
        assert first["event"] == "ready" and first["port"] != 8765
        with urllib.request.urlopen(f'http://127.0.0.1:{first["port"]}/workcell_studio_web/viewer/workcell_runtime_marker.json') as response:
            assert response.status == 200
        command(process, "stop")
        assert receive(process)["event"] == "stopped"
        gone(first["pid"])
        # A cancelled startup's queued error/started signals must not escape
        # into the replacement request or prevent its handshake.
        command(process, "replace")
        second = receive(process)
        assert second["event"] == "ready" and second["pid"] != first["pid"]
        with urllib.request.urlopen(f'http://127.0.0.1:{second["port"]}/workcell_studio_web/viewer/index.html') as response:
            assert response.status == 200
        command(process, "quit")
        assert process.wait(timeout=5) == 0
        gone(second["pid"])
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        occupant.close()


def test_parent_crash_cannot_orphan_http_child(owner_binary):
    process = subprocess.Popen([str(owner_binary), str(ROOT)], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True)
    command(process, "start")
    child = receive(process)
    assert child["event"] == "ready"
    process.kill()
    process.wait(timeout=5)
    gone(child["pid"])


def test_full_startup_traceback_is_retained(owner_binary, tmp_path):
    script = tmp_path / "scripts/workcell_product_view_server.py"
    script.parent.mkdir()
    script.write_text("raise RuntimeError('" + "diagnostic context " * 40 + "END_OF_TRACEBACK')\n")
    process = subprocess.Popen([str(owner_binary), str(tmp_path)], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True)
    command(process, "start")
    failure = receive(process)
    assert failure["event"] == "failed"
    assert "Traceback" in failure["detail"] and "END_OF_TRACEBACK" in failure["detail"]
    assert len(failure["detail"]) > 240
    command(process, "quit")
    assert process.wait(timeout=5) == 0
