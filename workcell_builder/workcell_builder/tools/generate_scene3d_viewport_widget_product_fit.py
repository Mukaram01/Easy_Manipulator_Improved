#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one {label} block, found {count}")
    return text.replace(old, new, 1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()

    source = args.input.read_text(encoding="utf-8")

    source = replace_once(
        source,
        "void Scene3DViewportWidget::set_isometric_view()\n{\n"
        "  yaw_ = -0.78539816339;\n"
        "  pitch_ = 0.61547970867;\n"
        "  orbit_offset_ = QVector3D(0.0f, 0.0f, 0.0f);\n"
        "  distance_ = 6.0;\n"
        "  update();\n"
        "}\n",
        "void Scene3DViewportWidget::set_isometric_view()\n{\n"
        "  yaw_ = -0.78539816339;\n"
        "  pitch_ = -0.61547970867;\n"
        "  orbit_offset_ = QVector3D(0.0f, 0.0f, 0.0f);\n"
        "  distance_ = 6.0;\n"
        "  update();\n"
        "}\n",
        "isometric pitch",
    )

    source = replace_once(
        source,
        "  yaw_ = -0.86;\n"
        "  pitch_ = 0.60;\n"
        "  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.06, product_radius * 0.035)));\n",
        "  yaw_ = -0.86;\n"
        "  pitch_ = -0.58;\n"
        "  orbit_offset_.setY(orbit_offset_.y() + static_cast<float>(qMax(0.02, product_radius * 0.015)));\n",
        "product-view pitch",
    )

    source = replace_once(
        source,
        "  const float near_plane = qMax(0.01f, qMin(0.2f, radius * 0.05f));\n"
        "  const float far_plane = qMax(clamped_distance + (radius * 8.0f), radius * 20.0f);\n",
        "  const float near_plane = qMax(0.005f, qMin(0.05f, radius * 0.01f));\n"
        "  const float far_plane = qMax(clamped_distance + (radius * 12.0f), radius * 30.0f);\n",
        "camera clipping planes",
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(source, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
