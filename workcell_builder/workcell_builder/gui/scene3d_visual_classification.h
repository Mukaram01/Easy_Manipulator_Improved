#pragma once

#include "scene_preview_widget.h"

#include <QString>
#include <QStringList>

namespace workcell_builder::scene3d_visual_classification {

QString normalized_token(QString value);
QString normalized_layer_token(const QString & value);
QStringList canonical_helper_overlay_tokens();
bool identity_contains_helper_overlay_token(const ScenePreviewWidget::PreviewItem & item);
bool is_generated_urdf_visual_identity(const ScenePreviewWidget::PreviewItem & item);
bool is_helper_overlay_identity(const ScenePreviewWidget::PreviewItem & item);

}  // namespace workcell_builder::scene3d_visual_classification
