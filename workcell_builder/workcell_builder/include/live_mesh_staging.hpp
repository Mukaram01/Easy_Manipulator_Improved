#pragma once

// Mesh URIs for items added to the live Web3D authoring session.
//
// The embedded Product View loads meshes only from browser-served roots
// (assets/, scenes/, workcell_studio_web/ and the exporter's staged cache
// build/workcell_studio_web_scene/assets/). Asset Library parts live elsewhere
// in the repository (workcell_builder/workcell_builder/assets/...), so a newly
// placed part used to be drawn as a placeholder until the Product View was
// regenerated. This stages such a mesh into the same cache path the exporter
// uses, build/workcell_studio_web_scene/assets/<scene_id>/<repo path>, so the
// live item draws its real mesh at once and a later export overwrites the same
// file. The staged copy is a derived cache: authored state keeps the original
// mesh path, and nothing here marks the layout dirty.

#include <QString>

namespace workcell_builder
{

struct LiveMeshStaging
{
  QString uri;     // repo-relative URI the viewer may load; empty when the mesh cannot be drawn
  QString status;  // "available" (already served), "staged" (copied into the cache), or why not:
                   // no_mesh_uri, unsupported_format, missing_file, unsafe_path, unsafe_destination, stage_failed
  QString reason;  // actionable reason when uri is empty
  bool ok() const { return !uri.isEmpty(); }
};

// mesh_source: absolute, repository-relative or scene-relative mesh path.
// Relative paths resolve against the repository first, then the scene
// directory, as the exporter does. Meshes outside the served repository are
// refused rather than copied.
LiveMeshStaging stage_live_authoring_mesh(
  const QString & repo_root, const QString & scene_dir, const QString & scene_id,
  const QString & mesh_source);

}  // namespace workcell_builder
