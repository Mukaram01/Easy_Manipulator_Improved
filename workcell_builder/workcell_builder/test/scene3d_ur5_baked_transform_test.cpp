#include <gtest/gtest.h>

#include <QMatrix4x4>
#include <QString>
#include <QVector>
#include <cmath>

#include "../gui/scene3d_viewport_widget.h"

namespace {

constexpr double kHalfPi = 1.57079632679489661923;

ScenePreviewWidget::PreviewItem make_required_ur5_link(
  const QString & link_name,
  const QString & mesh_uri,
  double mesh_r,
  double mesh_p,
  double mesh_y)
{
  ScenePreviewWidget::PreviewItem item;
  item.id = QStringLiteral("ur5_%1_visual").arg(link_name);
  item.display_name = link_name;
  item.source_layer = QStringLiteral("locked_generated_urdf_visual");
  item.active_visual_source = QStringLiteral("mesh_preview");
  item.locked = true;
  item.editable = false;
  item.lock_reason = QStringLiteral("Generated URDF visual");
  item.mesh_available = true;
  item.has_mesh_metadata = true;
  item.visual_index_link_name = link_name;
  item.visual_index_mesh_uri = mesh_uri;
  item.package_uri = mesh_uri;
  item.mesh_path = mesh_uri;
  item.has_baked_world_visual_transform = true;
  item.baked_world_visual_transform_source = QStringLiteral("scene_visual_mesh_index");
  item.has_baked_world_visual_matrix = true;
  item.baked_world_visual_matrix.setToIdentity();
  item.baked_world_visual_matrix.translate(0.42f, -0.13f, 0.77f);
  item.baked_world_visual_matrix.rotate(17.0f, 0.0f, 0.0f, 1.0f);
  item.mesh_r = mesh_r;
  item.mesh_p = mesh_p;
  item.mesh_y = mesh_y;
  item.has_origin_offset = true;
  item.origin_offset_x = 0.011;
  item.origin_offset_y = -0.022;
  item.origin_offset_z = 0.033;
  item.mesh_scale_x = 1.0;
  item.mesh_scale_y = 1.0;
  item.mesh_scale_z = 1.0;
  return item;
}

QMatrix4x4 ros_to_viewport_basis_matrix_for_expected()
{
  QMatrix4x4 basis;
  basis.setToIdentity();
  basis(1, 1) = 0.0f;
  basis(1, 2) = 1.0f;
  basis(2, 1) = -1.0f;
  basis(2, 2) = 0.0f;
  return basis;
}

void expect_matrix_near(const QMatrix4x4 & actual, const QMatrix4x4 & expected)
{
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), 1e-5)
        << "matrix mismatch at (" << row << ", " << col << ")";
    }
  }
}

}  // namespace

TEST(Scene3DUr5BakedTransform, MatrixBakedRequiredUr5LinksIncludeAssetLocalCorrection)
{
  const QVector<ScenePreviewWidget::PreviewItem> items = {
    make_required_ur5_link(
      QStringLiteral("upper_arm_link"),
      QStringLiteral("package://ur_description/meshes/ur5/visual/upperarm.dae"),
      kHalfPi, 0.0, 0.0),
    make_required_ur5_link(
      QStringLiteral("forearm_link"),
      QStringLiteral("package://ur_description/meshes/ur5/visual/forearm.dae"),
      0.0, kHalfPi, 0.0),
    make_required_ur5_link(
      QStringLiteral("wrist_1_link"),
      QStringLiteral("package://ur_description/meshes/ur5/visual/wrist1.dae"),
      0.0, 0.0, kHalfPi),
  };

  for (const ScenePreviewWidget::PreviewItem & item : items) {
    const QMatrix4x4 correction =
      Scene3DViewportWidget::baked_mesh_asset_local_correction_matrix_for_test(item);
    ASSERT_FALSE(correction.isIdentity()) << item.visual_index_link_name.toStdString();

    QMatrix4x4 expected = ros_to_viewport_basis_matrix_for_expected() * item.baked_world_visual_matrix;
    expected *= correction;
    expected.scale(static_cast<float>(item.mesh_scale_x),
                   static_cast<float>(item.mesh_scale_y),
                   static_cast<float>(item.mesh_scale_z));

    const QMatrix4x4 actual = Scene3DViewportWidget::final_mesh_transform_matrix_for_test(item);
    expect_matrix_near(actual, expected);

    const QMatrix4x4 without_asset_local_correction =
      ros_to_viewport_basis_matrix_for_expected() * item.baked_world_visual_matrix;
    bool observed_correction = false;
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (std::abs(actual(row, col) - without_asset_local_correction(row, col)) > 1e-5) {
          observed_correction = true;
        }
      }
    }
    EXPECT_TRUE(observed_correction) << item.visual_index_link_name.toStdString();
  }
}
