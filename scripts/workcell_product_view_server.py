#!/usr/bin/env python3
"""Studio-owned loopback server. Stdout is a single JSON readiness handshake."""

import argparse
import functools
import http.server
import json
import os
from pathlib import Path
import sys
import threading


def main():
    # Qt 5 may pass unrelated non-CLOEXEC descriptors (including WebEngine's
    # debug listener). This child owns only its stdio and its own HTTP socket.
    os.closerange(3, os.sysconf("SC_OPEN_MAX"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--session", required=True)
    args = parser.parse_args()
    root = Path(args.directory).resolve(strict=True)
    if not root.is_dir():
        raise NotADirectoryError(str(root))

    class Handler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *values):
            # Routine requests must not fill the parent's process pipes.
            pass

        def log_error(self, format, *values):
            print(format % values, file=sys.stderr, flush=True)

    # The parent keeps the QProcess stdin pipe open. EOF also shuts the server
    # down if Studio crashes, without depending on a destructor or PID polling.
    def watch_parent():
        while sys.stdin.buffer.read(1):
            pass
        os._exit(0)

    threading.Thread(target=watch_parent, daemon=True).start()
    with http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), functools.partial(Handler, directory=str(root))
    ) as server:
        print(json.dumps({
            "schema": "workcell_product_view_server/v1",
            "host": server.server_address[0],
            "port": server.server_address[1],
            "repo_root": str(root),
            "session": args.session,
            "pid": os.getpid(),
        }), flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
