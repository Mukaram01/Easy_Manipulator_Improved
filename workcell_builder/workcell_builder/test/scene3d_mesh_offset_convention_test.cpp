// Scene3D draws an offset, rotated mesh where Web3D, the collision manifest
// (generate_moveit_collision_manifest.py) and the alignment bounds put it:
// vertex_item = origin_offset + R(mesh_rpy) * vertex_mesh.

#include <gtest/gtest.h>

#include <QVector3D>

#include <cmath>

#include "gui/scene3d_transform_math.h"

TEST(Scene3DMeshOffsetConvention, OffsetIsInTheItemFrameNotTheTurnedMeshFrame)
{
  // The calibrated ur5_2f_test sorting bin.
  const double roll = M_PI / 2, pitch = 0., yaw = M_PI / 2;
  const QVector3D offset(-0.173899436593f, 0.f, -0.1f);
  const QVector3D vertex(0.1f, 0.2f, 0.3f);
  const QMatrix4x4 matrix = workcell_builder::urdf_visual_origin_matrix(offset.x(), offset.y(), offset.z(), roll, pitch, yaw);
  // ROS fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch), cy = std::cos(yaw), sy = std::sin(yaw);
  const double R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                          {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                          {-sp, cp * sr, cp * cr}};
  const double v[3] = {vertex.x(), vertex.y(), vertex.z()};
  const double o[3] = {offset.x(), offset.y(), offset.z()};
  const QVector3D mapped = matrix.map(vertex);
  for (int k = 0; k < 3; ++k) {
    const double expected = o[k] + R[k][0] * v[0] + R[k][1] * v[1] + R[k][2] * v[2];
    EXPECT_NEAR(mapped[k], expected, 1e-6) << "axis " << k;
  }
  // The previous Scene3D order (offset applied after the RPY) put this vertex 0.17 m away.
  QMatrix4x4 old_order;
  old_order.rotate(90.f, 0.f, 0.f, 1.f);
  old_order.rotate(90.f, 1.f, 0.f, 0.f);
  old_order.translate(offset);
  EXPECT_GT((old_order.map(vertex) - mapped).length(), 0.1f);
}
