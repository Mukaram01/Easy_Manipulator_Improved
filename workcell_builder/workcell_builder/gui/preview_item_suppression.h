#pragma once

#include "scene_preview_widget.h"

#include <QMap>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

namespace workcell_builder
{

struct PreviewSuppressionResult
{
  QVector<ScenePreviewWidget::PreviewItem> items;
  QMap<QString, int> suppression_reason_counts;
  QStringList suppression_diagnostics;
  int suppressed_preview_placeholder_count{0};
  int preserved_editable_source_count{0};
  int authoritative_visual_equivalence_key_count{0};
};

bool is_authoritative_generated_urdf_mesh_preview_item(const ScenePreviewWidget::PreviewItem & item);
bool is_lower_fidelity_generated_placeholder(const ScenePreviewWidget::PreviewItem & item);
QSet<QString> generated_urdf_preview_equivalence_keys_for_item(const ScenePreviewWidget::PreviewItem & item);
QString generated_urdf_preview_suppression_reason_for_equivalence_key(const QString & key);
PreviewSuppressionResult suppress_lower_fidelity_preview_items(
  const QVector<ScenePreviewWidget::PreviewItem> & preview_items,
  bool authoritative_mesh_index_healthy);

}  // namespace workcell_builder
