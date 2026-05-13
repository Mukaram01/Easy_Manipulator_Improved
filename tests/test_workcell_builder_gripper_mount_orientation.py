from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GEN = ROOT / "workcell_builder" / "workcell_builder" / "include" / "yaml_parser" / "generate_yaml.h"
EE_GUI = ROOT / "workcell_builder" / "workcell_builder" / "gui" / "addendeffector.cpp"


def test_default_mount_rpy_is_correct_in_generator():
    text = GEN.read_text(encoding="utf-8")
    assert "origin.roll = -1.5708F;" in text
    assert "origin.pitch = -1.5708F;" in text
    assert "origin.yaw = 0.0F;" in text


def test_negative_rpy_not_filtered_when_loading_existing_ee():
    text = EE_GUI.read_text(encoding="utf-8")
    assert "ui->roll->setText(QString::number(ee.origin.roll));" in text
    assert "ui->pitch->setText(QString::number(ee.origin.pitch));" in text
    assert "ui->yaw->setText(QString::number(ee.origin.yaw));" in text
    assert "ee.origin.roll >= 0" not in text
