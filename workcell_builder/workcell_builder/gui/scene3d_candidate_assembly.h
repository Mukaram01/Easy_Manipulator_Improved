#pragma once

#include "scene_preview_widget.h"

#include <QSet>
#include <QVector>

namespace workcell_builder {

struct Scene3DLayerVisibilityDefaults
{
  bool editable_layout{ true };
  bool mesh_preview{ true };
  bool primitive_fallback{ true };
  bool locked_generated_urdf_visual{ false };
};

bool include_preview_item_for_scene3d(
  const ScenePreviewWidget::PreviewItem & item,
  const QSet<QString> & enabled_layers);

Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility(
  const QVector<ScenePreviewWidget::PreviewItem> & all_items);

}  // namespace workcell_builder
