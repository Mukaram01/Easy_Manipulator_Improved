// Copyright 2026 Mukaram01
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WORKCELL_BUILDER_VISUAL_MESH_SOURCE_RESOLVER_HPP_
#define WORKCELL_BUILDER_VISUAL_MESH_SOURCE_RESOLVER_HPP_

#include <boost/filesystem.hpp>
#include <QString>
#include <QStringList>
#include <QMap>

namespace workcell_builder
{

boost::filesystem::path workcell_builder_repo_root_from_source();

QStringList product_asset_roots(const QString & workspace_root);

QString mesh_asset_category_for_path(const QString & mesh_path);

QMap<QString, int> discover_visual_mesh_asset_category_counts(
  const QStringList & asset_roots, int * total_mesh_count = nullptr, QStringList * mesh_paths = nullptr);

QMap<QString, QString> discover_visual_mesh_package_map(
  const boost::filesystem::path & scene_dir, const QString & workspace_root,
  QStringList * detected_asset_roots = nullptr);

QString resolve_visual_mesh_source_path(
  const QString & raw_path, const QString & package_uri,
  const boost::filesystem::path & scene_dir, const QString & workspace_root,
  QStringList * tried_candidates = nullptr);

}  // namespace workcell_builder

#endif  // WORKCELL_BUILDER_VISUAL_MESH_SOURCE_RESOLVER_HPP_
