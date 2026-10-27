from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
