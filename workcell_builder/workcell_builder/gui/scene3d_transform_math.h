#pragma once

// Incremental extraction target for Scene3D transform and bounds math helpers.
// Behavior remains owned by scene3d_viewport_widget.cpp until the next small move.

#include <QMatrix4x4>
#include <QtMath>

namespace workcell_builder
{

// The authored mesh calibration of a layout item, as a URDF <visual><origin>:
// vertex_item = origin_offset + R(mesh_rpy) * vertex_mesh (ROS fixed-axis RPY).
// The Web3D Product View, the MoveIt collision manifest and the alignment bounds
// all use this order; Scene3D must draw the same geometry.
inline QMatrix4x4 urdf_visual_origin_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
  QMatrix4x4 matrix;
  matrix.translate(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  matrix.rotate(static_cast<float>(qRadiansToDegrees(yaw)), 0.0f, 0.0f, 1.0f);
  matrix.rotate(static_cast<float>(qRadiansToDegrees(pitch)), 0.0f, 1.0f, 0.0f);
  matrix.rotate(static_cast<float>(qRadiansToDegrees(roll)), 1.0f, 0.0f, 0.0f);
  return matrix;
}

}  // namespace workcell_builder
