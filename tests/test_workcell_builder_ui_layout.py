from pathlib import Path
import xml.etree.ElementTree as ET

UI_DIR = Path('workcell_builder/workcell_builder/gui')

def _dialog_size(ui_file: Path):
    root = ET.parse(ui_file).getroot()
    widget = root.find('widget')
    if widget is None or widget.attrib.get('class') not in {'QDialog', 'QMainWindow'}:
        return None
    geom = widget.find("property[@name='geometry']/rect")
    if geom is None:
        return None
    width = int(geom.findtext('width', default='0'))
    height = int(geom.findtext('height', default='0'))
    return width, height

def test_dialog_geometry_caps():
    oversized = []
    for ui in UI_DIR.glob('*.ui'):
        size = _dialog_size(ui)
        if not size:
            continue
        w, h = size
        if w > 1200 or h > 900:
            oversized.append((ui.name, w, h))
    assert not oversized, f"Oversized dialogs found: {oversized}"

def test_addscene_height_compact():
    addscene = UI_DIR / 'addscene.ui'
    w, h = _dialog_size(addscene)
    assert w <= 1100
    assert h <= 900
