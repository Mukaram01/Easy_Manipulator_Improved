"""Read-only camera preview conversion and source-of-truth regressions."""
from __future__ import annotations

import importlib.util
import struct
import zlib
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("workcell_camera_probe", ROOT / "scripts/workcell_camera_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def message(encoding, width, height, step, data, frame="camera_color_optical_frame", big_endian=0):
    return SimpleNamespace(
        encoding=encoding, width=width, height=height, step=step, data=data,
        is_bigendian=big_endian,
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=123, nanosec=456)),
    )


def unpack_png(png):
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    width, height, depth, color, *_ = struct.unpack(">IIBBBBB", png[16:29])
    assert depth == 8
    off = 8
    raw = b""
    while off < len(png):
        length = struct.unpack(">I", png[off:off+4])[0]
        chunk_type = png[off+4:off+8]
        payload = png[off+8:off+8+length]
        if chunk_type == b"IDAT":
            raw += zlib.decompress(payload)
        off += 12 + length
    return width, height, color, raw


def test_bgr_preview_honours_stride_and_preserves_ros_header():
    # RGB PNG should contain R,G,B, despite bgr8 transport and row padding.
    png, meta = probe.preview_image(
        message("bgr8", 2, 1, 8, bytes([10, 20, 30, 40, 50, 60, 200, 200])), "rgb")
    assert unpack_png(png) == (2, 1, 2, bytes([0, 30, 20, 10, 60, 50, 40]))
    assert meta["frame_id"] == "camera_color_optical_frame"
    assert meta["timestamp_ns"] == 123000000456
    assert meta["preview_only"] is True


def test_depth_preview_remains_display_only_and_big_endian_safe():
    little = message("16UC1", 2, 1, 4, struct.pack("<HH", 1000, 3000))
    png, meta = probe.preview_image(little, "depth")
    assert unpack_png(png) == (2, 1, 0, bytes([0, 85, 255]))
    assert meta["encoding"] == "16uc1"
    big = message("16UC1", 2, 1, 4, struct.pack(">HH", 1000, 3000), big_endian=1)
    assert unpack_png(probe.preview_image(big, "depth")[0]) == unpack_png(png)


@pytest.mark.parametrize("encoding,stream,data,step", [
    ("16UC1", "rgb", b"\x00\x00", 2),
    ("mono8", "depth", b"\x00", 1),
    ("unknown", "rgb", b"\x00", 1),
    ("rgb8", "rgb", b"\x00", 3),
])
def test_invalid_encoding_or_truncated_frame_blocks(encoding, stream, data, step):
    with pytest.raises(ValueError, match="IMAGE_"):
        probe.preview_image(message(encoding, 2, 1, step, data), stream)


def test_camera_probe_reads_saved_task_and_authored_topics_without_default_motion(tmp_path):
    (tmp_path / "config").mkdir()
    task = {"perception": {"mode": "live_epd", "camera_source": "simulated_rgbd"}}
    (tmp_path / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(task))
    (tmp_path / "environment.yaml").write_text(yaml.safe_dump({
        "camera": {"enabled": True, "camera_id": "overhead",
                   "frame_id": "camera_color_optical_frame",
                   "rgb_topic": "/my/rgb", "depth_topic": "/my/aligned_depth",
                   "calibration_status": "authored_extrinsics_require_physical_verification"}
    }))
    cfg = probe.camera_config(tmp_path, "simulated_rgbd")
    assert cfg["rgb_topic"] == "/my/rgb"
    assert cfg["depth_topic"] == "/my/aligned_depth"
    assert cfg["camera_info_inferred"]
    assert cfg["camera_info_topic"] == "/my/camera_info"
    with pytest.raises(ValueError, match="CAMERA_SOURCE_MISMATCH"):
        probe.camera_config(tmp_path, "realsense_d435i")
    task["perception"]["mode"] = "replayed_snapshot"
    (tmp_path / "config/workcell_builder_task_intent.yaml").write_text(yaml.safe_dump(task))
    with pytest.raises(ValueError, match="CAMERA_SOURCE_MISMATCH"):
        probe.camera_config(tmp_path, "simulated_rgbd")


def test_camera_probe_missing_topics_fails_without_guessing(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/workcell_builder_task_intent.yaml").write_text(
        yaml.safe_dump({"perception": {"mode": "live_epd", "camera_source": "realsense_d435i"}}))
    (tmp_path / "environment.yaml").write_text(
        yaml.safe_dump({"camera": {"enabled": True, "frame_id": "optical"}}))
    with pytest.raises(ValueError, match="CAMERA_TOPICS_MISSING"):
        probe.camera_config(tmp_path, "realsense_d435i")


def test_malformed_environment_blocks_with_actionable_reason(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/workcell_builder_task_intent.yaml").write_text(
        yaml.safe_dump({"perception": {"mode": "live_epd", "camera_source": "simulated_rgbd"}}))
    (tmp_path / "environment.yaml").write_text("null\n")
    with pytest.raises(ValueError, match="CAMERA_SCENE_INVALID"):
        probe.camera_config(tmp_path, "simulated_rgbd")
