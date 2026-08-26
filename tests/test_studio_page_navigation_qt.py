import os
from pathlib import Path
import shlex
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_qt_navigation_round_trip_reopens_same_and_alternating_scenes(tmp_path):
    source = tmp_path / "navigation_round_trip.cpp"
    source.write_text(r'''
#include <QApplication>
#include <QLabel>
#include <QListWidget>
#include <QStackedWidget>
#include "studio_page_navigation.hpp"

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QListWidget nav;
  QStackedWidget pages;
  for (int i = 0; i < 2; ++i) {
    nav.addItem(i == 0 ? "Home" : "Scene Builder");
    pages.addWidget(new QLabel(i == 0 ? "Home" : "Scene Builder"));
  }
  int open_count = 0;
  QString active_scene;
  auto open = [&](const QString & scene) {
    active_scene = scene;
    ++open_count;
    return workcell_builder::synchronize_studio_page(&nav, &pages, 1);
  };
  auto back = [&]() { return workcell_builder::synchronize_studio_page(&nav, &pages, 0); };

  if (!back() || nav.currentRow() != 0 || pages.currentIndex() != 0) return 1;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const QString expected = cycle % 2 ? "ur5_2f_test" : "suction_test";
    if (!open(expected) || nav.currentRow() != 1 || pages.currentIndex() != 1) return 2;
    if (active_scene != expected || open_count != cycle + 1) return 3;
    if (!back() || nav.currentRow() != 0 || pages.currentIndex() != 0) return 4;
  }
  // Reproduce the escaped state directly: stack at Home, nav still at Scene Builder.
  nav.setCurrentRow(1);
  pages.setCurrentIndex(0);
  if (!open("ur5_2f_test") || nav.currentRow() != 1 || pages.currentIndex() != 1) return 5;
  return active_scene == "ur5_2f_test" && open_count == 11 ? 0 : 6;
}
''', encoding="utf-8")
    flags = subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "Qt5Widgets"], text=True
    ).strip()
    executable = tmp_path / "navigation_round_trip"
    subprocess.run([
        "g++", "-std=c++17", "-fPIC", str(source),
        "-I", str(ROOT / "workcell_builder/workcell_builder/include"),
        *shlex.split(flags), "-o", str(executable),
    ], check=True)
    environment = os.environ.copy()
    environment["QT_QPA_PLATFORM"] = "offscreen"
    subprocess.run([str(executable)], check=True, env=environment)
