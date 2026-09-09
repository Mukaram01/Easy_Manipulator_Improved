"""Save must retain off-grid inspector coordinates while drag snapping remains enabled."""
from pathlib import Path
import os
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def test_authored_position_bypasses_drag_filter_and_restores_it(tmp_path):
    source = tmp_path / 'position.cpp'
    source.write_text(r'''
#include "authored_canvas_position.hpp"
#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <cassert>
#include <cmath>
class SnappedItem : public QGraphicsRectItem {
  QVariant itemChange(GraphicsItemChange change, const QVariant & value) override {
    if (change == ItemPositionChange) {
      auto p = value.toPointF();
      return QPointF(std::round(p.x()/5)*5, std::round(p.y()/5)*5);
    }
    return QGraphicsRectItem::itemChange(change, value);
  }
};
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QGraphicsScene scene;
  auto * item = new SnappedItem;
  scene.addItem(item);
  item->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  for (const auto & pose : {QPointF(46,22), QPointF(47,23)}) {
    workcell_builder::set_authored_canvas_position(item, pose);
    assert(item->pos() == pose);
    assert(item->flags().testFlag(QGraphicsItem::ItemSendsGeometryChanges));
  }
  item->setPos(48,24);
  assert(item->pos() == QPointF(50,25));
}
''')
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'Qt5Widgets'], text=True))
    exe = tmp_path / 'position'
    subprocess.run(['g++', '-std=c++17', '-fPIC', str(source), '-I'+str(ROOT/'workcell_builder/workcell_builder/gui'), *flags, '-o', str(exe)], check=True)
    subprocess.run([str(exe)], env={**os.environ, 'QT_QPA_PLATFORM':'offscreen'}, check=True)
