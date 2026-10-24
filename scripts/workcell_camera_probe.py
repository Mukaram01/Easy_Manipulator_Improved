#!/usr/bin/env python3
"""Read-only, bounded ROS RGB-D preview for Workcell Studio Task settings.

Preview pixels are NOT calibrated collision geometry or a motion-readiness proof.
This script neither launches camera/EPD processes nor publishes ROS commands.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import time
import zlib
from pathlib import Path
from typing import Any

import yaml


CAMERA_SOURCES = {"simulated_rgbd", "realsense_d435i"}
MAX_PIXELS = 4096 * 4096
MAX_BYTES = 64 * 1024 * 1024


def intrinsics_present(k) -> bool:
    """Plain bool: rclpy delivers CameraInfo.k as a numpy array, whose comparisons are not JSON-serializable."""
    values = [float(v) for v in k]
    return bool(len(values) == 9 and all(math.isfinite(v) for v in values) and values[0] > 0 and values[4] > 0)


def camera_config(scene: Path, selected: str) -> dict[str, str]:
    if selected not in CAMERA_SOURCES:
        raise ValueError("CAMERA_SOURCE_UNSUPPORTED: select simulated RGB-D or RealSense D435i")
    scene = Path(scene)
    intent = yaml.safe_load((scene / "config/workcell_builder_task_intent.yaml").read_text(encoding="utf-8"))
    perception = intent.get("perception", {}) if isinstance(intent, dict) else {}
    if not isinstance(perception, dict) or perception.get("mode") != "live_epd" or perception.get("camera_source") != selected:
        raise ValueError("CAMERA_SOURCE_MISMATCH: save the selected live camera in Task settings before preview")
    document = yaml.safe_load((scene / "environment.yaml").read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("CAMERA_SCENE_INVALID: environment.yaml must be a mapping")
    environment = document.get("environment", document)
    if not isinstance(environment, dict):
        raise ValueError("CAMERA_SCENE_INVALID: environment must be a mapping")
    camera = environment.get("camera") or document.get("camera", {})
    if not isinstance(camera, dict) or camera.get("enabled") is not True:
        raise ValueError("CAMERA_NOT_CONFIGURED: enable a physical camera in the existing scene environment")
    rgb = camera.get("rgb_topic")
    depth = camera.get("depth_topic")
    frame = camera.get("frame_id")
    if any(not isinstance(v, str) or not v.startswith("/") for v in (rgb, depth)):
        raise ValueError("CAMERA_TOPICS_MISSING: configure authored RGB and aligned depth ROS topics")
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError("CAMERA_FRAME_MISSING: set an authored optical frame")
    info = camera.get("camera_info_topic")
    info_inferred = False
    if not info:
        # Discovery hint for preview only, not an authored/calibrated camera binding.
        info = rgb.rsplit("/", 1)[0] + "/camera_info"
        info_inferred = True
    if not isinstance(info, str) or not info.startswith("/"):
        raise ValueError("CAMERA_INFO_TOPIC_INVALID: check authored camera_info_topic")
    return {
        "camera_source": selected,
        "camera_id": str(camera.get("camera_id") or ""),
        "rgb_topic": rgb,
        "depth_topic": depth,
        "camera_info_topic": info,
        "camera_info_inferred": info_inferred,
        "frame_id": frame,
        "calibration_status": str(camera.get("calibration_status") or "not_verified"),
    }


def _chunk(kind: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def _png(width: int, height: int, channels: int, pixels: bytes) -> bytes:
    if width < 1 or height < 1 or width * height > MAX_PIXELS:
        raise ValueError("IMAGE_DIMENSIONS_INVALID")
    if len(pixels) != width * height * channels:
        raise ValueError("IMAGE_PAYLOAD_INVALID")
    # Filter 0 per scanline; no PIL, OpenCV, GPU or Qt dependency in the ROS probe.
    raw = b"".join(b"\x00" + pixels[y * width * channels:(y + 1) * width * channels]
                   for y in range(height))
    color = 0 if channels == 1 else 2
    return (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0))
            + _chunk(b"IDAT", zlib.compress(raw, level=3)) + _chunk(b"IEND", b""))


def preview_image(msg: Any, stream: str) -> tuple[bytes, dict[str, Any]]:
    """Convert a received sensor_msgs/Image into diagnostic PNG; never geometry."""
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width < 1 or height < 1 or width * height > MAX_PIXELS:
        raise ValueError("IMAGE_DIMENSIONS_INVALID")
    encoding = str(msg.encoding).lower()
    layouts = {"rgb8": (3, "rgb"), "bgr8": (3, "bgr"),
               "rgba8": (4, "rgba"), "bgra8": (4, "bgra"),
               "mono8": (1, "mono"), "16uc1": (2, "depth16"), "32fc1": (4, "depth32")}
    if encoding not in layouts or (stream == "rgb" and encoding in ("16uc1", "32fc1")) or (
            stream == "depth" and encoding not in ("16uc1", "32fc1")):
        raise ValueError(f"IMAGE_ENCODING_UNSUPPORTED: {stream} {encoding}")
    bytes_per_pixel, kind = layouts[encoding]
    if step < width * bytes_per_pixel or step * height > MAX_BYTES:
        raise ValueError("IMAGE_STRIDE_INVALID")
    data = bytes(msg.data)
    if len(data) < step * height:
        raise ValueError("IMAGE_TRUNCATED")
    rows = [data[y * step:y * step + width * bytes_per_pixel] for y in range(height)]
    pixels = b"".join(rows)
    if kind in ("bgr", "bgra", "rgba"):
        out = bytearray(width * height * 3)
        for i in range(width * height):
            base = i * bytes_per_pixel
            rgb = pixels[base:base+3]
            out[i*3:i*3+3] = rgb[::-1] if kind == "bgra" or kind == "bgr" else rgb
        pixels, channels = bytes(out), 3
    elif kind in ("rgb", "mono"):
        channels = bytes_per_pixel
    else:
        # Static grey-range for VISUAL preview ONLY, not physical depth scaling.
        endian = ">" if bool(getattr(msg, "is_bigendian", 0)) else "<"
        fmt = endian + ("H" if kind == "depth16" else "f")
        unpacked = struct.iter_unpack(fmt, pixels)
        scale = 1000.0 if kind == "depth32" else 1.0
        pixels = bytes(int(max(0, min(255, round(float(v) * scale * 255.0 / 3000.0))))
                       if math.isfinite(float(v)) and float(v) > 0 else 0 for (v,) in unpacked)
        channels = 1
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    timestamp_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)) if stamp is not None else None
    return _png(width, height, channels, pixels), {
        "width": width, "height": height, "encoding": encoding,
        "frame_id": str(getattr(header, "frame_id", "") or ""),
        "timestamp_ns": timestamp_ns,
        "preview_only": True,
    }


def capture(config: dict[str, Any], output_dir: Path, timeout: float) -> dict[str, Any]:
    """Subscribe to *existing* ROS topics. Does not launch or drive hardware."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String

    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("workcell_camera_diagnostic_capture")
    images: dict[str, Any] = {}
    info: dict[str, Any] = {}
    epd_status: dict[str, Any] = {}
    errors: list[str] = []
    started = time.monotonic()

    def image_callback(stream: str):
        def receive(msg):
            if stream in images:
                return
            try:
                png, details = preview_image(msg, stream)
                name = stream + ".png"
                (output_dir / name).write_bytes(png)
                images[stream] = {"file": name, **details}
            except (ValueError, OSError, OverflowError) as exc:
                errors.append(f"{stream}: {exc}")
                images[stream] = {"error": str(exc)}
        return receive

    def on_info(msg):
        if info:
            return
        info.update(frame_id=msg.header.frame_id, width=msg.width, height=msg.height,
                    distortion_model=msg.distortion_model,
                    intrinsics_present=intrinsics_present(msg.k))

    def on_status(msg):
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, dict):
                epd_status.update(parsed)
        except (ValueError, TypeError):
            pass

    subscriptions = [
        node.create_subscription(Image, config["rgb_topic"], image_callback("rgb"), qos_profile_sensor_data),
        node.create_subscription(Image, config["depth_topic"], image_callback("depth"), qos_profile_sensor_data),
        node.create_subscription(CameraInfo, config["camera_info_topic"], on_info, qos_profile_sensor_data),
        node.create_subscription(String, "/workcell_studio/epd_connector_status", on_status, 10),
    ]
    try:
        while time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(k in images for k in ("rgb", "depth")) and info:
                break
    finally:
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()
    warnings: list[str] = []
    if config["camera_info_inferred"]:
        warnings.append("CameraInfo topic inferred for diagnosis only, not authoritative calibration")
    if not info or not info.get("intrinsics_present"):
        warnings.append("CALIBRATION_UNVERIFIED: no valid CameraInfo intrinsics were received")
    if config["calibration_status"] != "verified":
        warnings.append("CALIBRATION_UNVERIFIED: authored camera extrinsics require physical/TF verification")
    if images.get("rgb", {}).get("frame_id") != config["frame_id"]:
        warnings.append("RGB_FRAME_MISMATCH: received frame differs from authored optical frame")
    if images.get("rgb", {}).get("frame_id") != images.get("depth", {}).get("frame_id"):
        warnings.append("DEPTH_ALIGNMENT_UNVERIFIED: RGB/depth frames differ")
    if not epd_status or epd_status.get("state") != "READY" or epd_status.get("camera_id") != config["camera_id"]:
        warnings.append("EPD_NOT_QUALIFIED: matching READY connector status not observed")
    missing = [name for name in ("rgb", "depth") if name not in images or "error" in images[name]]
    if missing:
        errors.append("MISSING_RGBD_STREAM: " + ", ".join(missing) + " image not received from configured ROS topic")
    return {
        "status": "BLOCKED" if errors else "PREVIEW_CAPTURED",
        "camera_source": config["camera_source"],
        "camera_id": config["camera_id"],
        "topics": {k: config[k] for k in ("rgb_topic", "depth_topic", "camera_info_topic")},
        "images": images, "camera_info": info, "epd_connector_status": epd_status,
        "errors": errors, "warnings": warnings,
        "robot_motion_commanded": False, "planning_ready": False,
        "note": "Transport/visual diagnostic only; ROS topic presence cannot authenticate camera hardware or prove calibrated geometry.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--camera-source", required=True, choices=sorted(CAMERA_SOURCES))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    result: dict[str, Any]
    try:
        if not 0.5 <= args.timeout <= 15:
            raise ValueError("TIMEOUT_INVALID: expected 0.5 to 15 seconds")
        config = camera_config(args.scene, args.camera_source)
        result = capture(config, args.output_dir, args.timeout)
    except (ValueError, OSError, RuntimeError, ImportError, yaml.YAMLError) as exc:
        result = {"status": "BLOCKED", "errors": [str(exc)], "warnings": [],
                  "planning_ready": False, "robot_motion_commanded": False}
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PREVIEW_CAPTURED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
