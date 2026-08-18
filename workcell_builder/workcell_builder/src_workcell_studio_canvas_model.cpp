#include "workcell_studio_canvas_model.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <iostream>
#include <initializer_list>
#include <map>
#include <ctime>
#include <set>
#include <sstream>
#include <mutex>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <boost/system/error_code.hpp>

namespace fs = boost::filesystem;
namespace workcell_builder {

void merge_dirty_editable_layout_session(
  WorkcellStudioCanvasModel & model,
  const std::vector<WorkcellStudioCanvasItem> & session_items,
  const std::vector<std::string> & deleted_item_ids)
{
  const std::set<std::string> deleted(deleted_item_ids.begin(), deleted_item_ids.end());
  model.items.erase(
    std::remove_if(model.items.begin(), model.items.end(), [&deleted](const auto & item) {
      return item.provenance == WorkcellStudioItemProvenance::EditableLayout ||
             deleted.count(item.id) != 0U;
    }), model.items.end());
  for (const auto & item : session_items) {
    if (deleted.count(item.id) == 0U) model.items.push_back(item);
  }
  model.provenance_status.editable_layout_count = 0;
  model.provenance_status.generated_or_legacy_preview_count = 0;
  model.provenance_status.static_fallback_preview_count = 0;
  for (const auto & item : model.items) {
    switch (item.provenance) {
      case WorkcellStudioItemProvenance::EditableLayout:
        ++model.provenance_status.editable_layout_count;
        break;
      case WorkcellStudioItemProvenance::StaticFallbackPreview:
        ++model.provenance_status.static_fallback_preview_count;
        break;
      case WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
      default:
        ++model.provenance_status.generated_or_legacy_preview_count;
        break;
    }
  }
}


static bool log_task_metadata_loader_warning_once(const fs::path & p, const std::string & reason)
{
  return workcell_builder::log_warning_once_per_context_path_reason(
    "task_metadata_summary_loader", p, reason);
}

struct YamlLoadStatus
{
  bool exists{false};
  bool loaded{false};
  bool parse_warning{false};
  std::string reason;
};

static YamlLoadStatus read_yaml(const fs::path & p, YAML::Node * out)
{
  YamlLoadStatus status;
  status.exists = fs::exists(p);
  if (!status.exists) return status;
  try {
    *out = YAML::LoadFile(p.string());
    status.loaded = true;
    return status;
  } catch (const YAML::Exception & e) {
    status.parse_warning = true;
    status.reason = e.what();
  } catch (const std::exception & e) {
    status.parse_warning = true;
    status.reason = e.what();
  } catch (...) {
    status.parse_warning = true;
    status.reason = "unknown exception";
  }
  log_task_metadata_loader_warning_once(p, "scene YAML parse warning: " + status.reason);
  return status;
}


struct SceneMetadataFileRevision
{
  bool exists{false};
  uintmax_t size{0};
  std::time_t mtime{0};
  std::size_t content_hash{0};
};

struct SceneMetadataSnapshot
{
  fs::path scene_dir;
  std::string scene_id;
  std::string revision;
  std::string invalidation_reason{"initial"};
  std::size_t files_parsed{0};
  std::size_t cache_hits{0};
  std::map<std::string, SceneMetadataFileRevision> file_revisions;
  std::map<std::string, YamlLoadStatus> statuses;
  std::map<std::string, YAML::Node> documents;
};

static const std::array<const char *, 7> kSceneMetadataFiles = {{
  "environment.yaml",
  "cell_definition.yaml",
  "scene_manifest.yaml",
  "environment_layout.yaml",
  "layout/workcell_studio_layout.yaml",
  "config/workcell_builder_task_intent.yaml",
  "config/task_recipe.yaml"
}};

static SceneMetadataFileRevision scene_metadata_file_revision(const fs::path & path)
{
  SceneMetadataFileRevision revision;
  boost::system::error_code ec;
  revision.exists = fs::exists(path, ec);
  if (!revision.exists || ec) return revision;
  revision.size = fs::file_size(path, ec);
  if (ec) revision.size = 0;
  revision.mtime = fs::last_write_time(path, ec);
  if (ec) revision.mtime = 0;
  std::ifstream in(path.string(), std::ios::binary);
  if (in) {
    std::ostringstream buffer;
    buffer << in.rdbuf();
    revision.content_hash = std::hash<std::string>{}(buffer.str());
  }
  return revision;
}

static fs::path canonical_scene_cache_key(const fs::path & scene_dir)
{
  boost::system::error_code ec;
  const fs::path canonical = fs::weakly_canonical(scene_dir, ec);
  return ec ? fs::absolute(scene_dir).lexically_normal() : canonical;
}

static std::string scene_metadata_revision_token(const std::map<std::string, SceneMetadataFileRevision> & revisions)
{
  std::ostringstream out;
  for (const auto & entry : revisions) {
    out << entry.first << ':' << entry.second.exists << ':' << entry.second.size << ':'
        << entry.second.mtime << ':' << entry.second.content_hash << ';';
  }
  return std::to_string(std::hash<std::string>{}(out.str()));
}


struct SceneMetadataSnapshotCacheState
{
  std::mutex mutex;
  SceneMetadataSnapshot cached;
  fs::path invalidated_scene_dir;
  std::string invalidation_reason;
};

static SceneMetadataSnapshotCacheState & scene_metadata_snapshot_cache_state()
{
  static SceneMetadataSnapshotCacheState state;
  return state;
}

void invalidate_workcell_studio_scene_metadata_snapshot(const fs::path & scene_dir, const std::string & reason)
{
  auto & state = scene_metadata_snapshot_cache_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  const fs::path key = canonical_scene_cache_key(scene_dir);
  if (!state.cached.scene_dir.empty() && state.cached.scene_dir == key) state.cached = SceneMetadataSnapshot{};
  state.invalidated_scene_dir = key;
  state.invalidation_reason = reason.empty() ? "explicit_refresh" : reason;
}

static SceneMetadataSnapshot load_scene_metadata_snapshot(const fs::path & scene_dir, const std::string & scene_id)
{
  auto & state = scene_metadata_snapshot_cache_state();
  const fs::path key = canonical_scene_cache_key(scene_dir);
  std::map<std::string, SceneMetadataFileRevision> revisions;
  for (const char * rel : kSceneMetadataFiles) revisions[rel] = scene_metadata_file_revision(key / rel);
  const std::string revision = scene_metadata_revision_token(revisions);
  std::lock_guard<std::mutex> lock(state.mutex);
  const bool explicitly_invalidated = !state.invalidated_scene_dir.empty() && state.invalidated_scene_dir == key;
  SceneMetadataSnapshot & cached = state.cached;
  if (!explicitly_invalidated && !cached.scene_dir.empty() && cached.scene_dir == key && cached.revision == revision) {
    ++cached.cache_hits;
    return cached;
  }

  const std::string reload_reason = explicitly_invalidated ?
    (state.invalidation_reason.empty() ? "explicit_refresh" : state.invalidation_reason) :
    (cached.scene_dir.empty() ? "initial" : (cached.scene_dir == key ? "file_revision_change" : "scene_switch"));

  SceneMetadataSnapshot snapshot;
  snapshot.scene_dir = key;
  snapshot.scene_id = scene_id;
  snapshot.revision = revision;
  snapshot.invalidation_reason = reload_reason;
  snapshot.file_revisions = revisions;
  for (const char * rel : kSceneMetadataFiles) {
    YAML::Node doc;
    YamlLoadStatus status = read_yaml(key / rel, &doc);
    snapshot.statuses[rel] = status;
    if (status.loaded) {
      snapshot.documents[rel] = doc;
      ++snapshot.files_parsed;
    }
  }
  cached = snapshot;
  if (explicitly_invalidated) {
    state.invalidated_scene_dir.clear();
    state.invalidation_reason.clear();
  }
  std::cerr << "Workcell Studio scene metadata snapshot: scene_id=" << scene_id
            << " revision=" << snapshot.revision << " files_parsed=" << snapshot.files_parsed
            << " cache_hits=0 invalidation_reason=" << snapshot.invalidation_reason << std::endl;
  return cached;
}

static YamlLoadStatus snapshot_yaml(const SceneMetadataSnapshot & snapshot, const char * relative_path, YAML::Node * out)
{
  const auto status_it = snapshot.statuses.find(relative_path);
  if (status_it == snapshot.statuses.end()) return {};
  const auto doc_it = snapshot.documents.find(relative_path);
  if (doc_it != snapshot.documents.end() && out != nullptr) *out = doc_it->second;
  return status_it->second;
}

static void add_mesh_candidate(const YAML::Node & node, std::vector<std::string> * out)
{
  try {
    if (!node.IsDefined() || !node.IsScalar()) return;
    const std::string text = node.as<std::string>("");
    if (text.empty()) return;
    if (text.rfind("package://", 0) == 0 || text.find(".stl") != std::string::npos || text.find("meshes/") != std::string::npos) out->push_back(text);
  } catch (...) {
    return;
  }
}

static bool mesh_node_disabled(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return false;
  std::string value;
  if (!yaml_read_string(node, &value)) return false;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value == "none" || value == "disabled" || value == "false";
}

static std::string provenance_summary_text(const WorkcellStudioProvenanceStatus & status)
{
  return "Editable layout: " + std::to_string(status.editable_layout_count) + " items. Preview fallback: " +
    std::to_string(status.generated_or_legacy_preview_count + status.static_fallback_preview_count) +
    " items loaded from scene metadata.";
}

static bool yaml_node_is_map(const YAML::Node & node)
{
  try { return node.IsDefined() && node.IsMap(); } catch (...) { return false; }
}

static bool yaml_node_is_sequence(const YAML::Node & node)
{
  try { return node.IsDefined() && node.IsSequence(); } catch (...) { return false; }
}

static bool yaml_node_is_defined(const YAML::Node & node)
{
  try { return node.IsDefined(); } catch (...) { return false; }
}

static std::string normalized_canvas_match_token(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  for (char & c : value) {
    if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  }
  value.erase(std::unique(value.begin(), value.end(), [](char a, char b){ return a == '_' && b == '_'; }), value.end());
  while (!value.empty() && value.front() == '_') value.erase(value.begin());
  while (!value.empty() && value.back() == '_') value.pop_back();
  return value;
}

static bool safe_visual_mesh_index_has_matching_visual_items(const fs::path & scene_dir)
{
  const fs::path index_path = scene_dir / "generated" / "scene_visual_mesh_index.json";
  if (!fs::exists(index_path)) return false;
  try {
    const YAML::Node index = YAML::LoadFile(index_path.string());
    if (!yaml_map_key(index, "safe_for_preview").as<bool>(false)) return false;
    const std::string scene_name = yaml_map_value_or_empty(index, "scene_name");
    if (!scene_name.empty() && scene_name != scene_dir.filename().string()) return false;
    const YAML::Node visual_items = yaml_map_key(index, "visual_items");
    if (!visual_items || !visual_items.IsSequence() || visual_items.size() == 0) return false;
    for (const auto & visual : visual_items) {
      if (!visual || !visual.IsMap()) continue;
      const bool render_expected = yaml_map_key(visual, "render_expected").as<bool>(false);
      const bool resolved = yaml_map_key(visual, "resolved").as<bool>(false);
      const std::string geometry_type = normalized_canvas_match_token(yaml_map_value_or_empty(visual, "geometry_type"));
      const std::string id = normalized_canvas_match_token(yaml_map_value_or_empty(visual, "id"));
      const std::string link = normalized_canvas_match_token(yaml_map_value_or_empty(visual, "link"));
      const std::string source = normalized_canvas_match_token(yaml_map_value_or_empty(visual, "source_path"));
      if ((render_expected || resolved) &&
          (!geometry_type.empty() || !id.empty() || !link.empty() || !source.empty())) {
        return true;
      }
    }
  } catch (...) {
    return false;
  }
  return false;
}

static std::vector<std::string> gather_mesh_candidates(const YAML::Node & env, const YAML::Node & manifest, const YAML::Node & layout_items, const std::string & item_id)
{
  std::vector<std::string> out;
  const auto scan_fields = [&out](const YAML::Node & n) {
    const YAML::Node mesh = yaml_map_key(n, "mesh");
    if (yaml_node_is_map(mesh)) add_mesh_candidate(optional_scalar(mesh, "path"), &out);
    else add_mesh_candidate(mesh, &out);
    add_mesh_candidate(optional_scalar(n, "mesh_path"), &out);
    add_mesh_candidate(yaml_map_key(n, "visual_mesh"), &out);
    add_mesh_candidate(yaml_map_key(n, "collision_mesh"), &out);
    const YAML::Node visual = yaml_map_key(n, "visual");
    const YAML::Node v = yaml_node_is_map(visual) ? yaml_map_key(visual, "geometry") : YAML::Node();
    add_mesh_candidate(yaml_map_key(v, "filepath"), &out);
    add_mesh_candidate(yaml_map_key(v, "mesh"), &out);
  };
  scan_fields(env);
  scan_fields(manifest);
  if (yaml_node_is_sequence(layout_items)) {
    for (const auto & node : layout_items) {
      if (!yaml_node_is_map(node)) continue;
      if (yaml_map_value_or_empty(node, "id") != item_id) continue;
      scan_fields(node);
      YAML::Node metadata = yaml_map_key(node, "metadata");
      scan_fields(metadata);
    }
  }
  return out;
}

static fs::path resolve_mesh_candidate(const std::string & c, const fs::path & scene_dir)
{
  fs::path path(c);
  if (c.rfind("package://", 0) == 0) {
    const std::string rest = c.substr(std::string("package://").size());
    path = scene_dir / "assets" / rest;
  } else if (!path.is_absolute()) {
    path = scene_dir / path;
  }
  return path;
}


static bool is_safe_scene_relative_path(const fs::path & candidate)
{
  if (candidate.empty() || candidate.is_absolute()) return false;
  for (const auto & part : candidate) {
    if (part.string() == "..") return false;
  }
  return true;
}

static double yaml_double_or_default(const YAML::Node & node, double fallback)
{
  double out = fallback;
  return yaml_read_double(node, &out) ? out : fallback;
}

[[maybe_unused]] static fs::path manifest_declared_canvas_layout_path(const fs::path & scene_dir, const YAML::Node & manifest)
{
  try {
    const YAML::Node files = yaml_map_key(manifest, "files");
    std::string declared;
    if (files && files.IsMap()) {
      declared = yaml_map_value_or_empty(files, "layout");
      if (declared.empty()) declared = yaml_map_value_or_empty(files, "editable_layout");
      if (declared.empty()) declared = yaml_map_value_or_empty(files, "workcell_studio_layout");
    }
    if (declared.empty()) declared = yaml_map_value_or_empty(manifest, "layout");
    const fs::path relative(declared);
    if (!is_safe_scene_relative_path(relative)) return {};
    return scene_dir / relative;
  } catch (const YAML::Exception &) {
    return {};
  } catch (const std::exception &) {
    return {};
  }
}

static std::string canvas_item_id_from_node(const YAML::Node & node, const std::string & fallback_prefix, std::size_t index)
{
  std::string id = yaml_map_value_or_empty(node, "id");
  if (id.empty()) id = yaml_map_value_or_empty(node, "name");
  if (id.empty()) id = yaml_map_value_or_empty(node, "asset_id");
  if (id.empty()) id = yaml_map_value_or_empty(node, "object_id");
  if (id.empty()) id = yaml_map_value_or_empty(node, "target_id");
  if (id.empty()) id = fallback_prefix + "_" + std::to_string(index + 1);
  return id;
}

static void copy_canvas_pose_from_environment_node(const YAML::Node & source, YAML::Node * target)
{
  YAML::Node pose = yaml_map_key(source, "pose");
  YAML::Node xyz;
  YAML::Node rpy;
  if (yaml_node_is_map(pose)) {
    xyz = yaml_map_key(pose, "xyz");
    rpy = yaml_map_key(pose, "rpy");
  } else if (yaml_node_is_sequence(pose)) {
    xyz = pose;
  }
  if (!yaml_node_is_defined(xyz)) {
    const YAML::Node position = yaml_map_key(source, "position");
    if (yaml_node_is_sequence(position)) xyz = position;
    else if (yaml_node_is_map(position)) {
      YAML::Node synthesized(YAML::NodeType::Sequence);
      synthesized.push_back(yaml_double_or_default(yaml_map_key(position, "x"), 0.0));
      synthesized.push_back(yaml_double_or_default(yaml_map_key(position, "y"), 0.0));
      synthesized.push_back(yaml_double_or_default(yaml_map_key(position, "z"), 0.0));
      xyz = synthesized;
    }
  }
  if (!yaml_node_is_defined(rpy)) {
    const YAML::Node orientation = yaml_map_key(source, "rpy");
    if (yaml_node_is_sequence(orientation)) rpy = orientation;
  }

  YAML::Node out_pose(YAML::NodeType::Map);
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 0), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 1), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 2), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 0), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 1), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 2), 0.0));
  (*target)["pose"] = out_pose;
}

static void copy_canvas_size_from_environment_node(const YAML::Node & source, YAML::Node * target)
{
  YAML::Node size = yaml_map_key(source, "size");
  if (!yaml_node_is_defined(size)) size = yaml_map_key(source, "dimensions");
  if (!yaml_node_is_defined(size)) size = yaml_map_key(source, "scale");
  YAML::Node out_size(YAML::NodeType::Map);
  if (yaml_node_is_map(size)) {
    out_size["width"] = yaml_double_or_default(yaml_map_key(size, "width"), yaml_double_or_default(yaml_map_key(size, "x"), 0.25));
    out_size["depth"] = yaml_double_or_default(yaml_map_key(size, "depth"), yaml_double_or_default(yaml_map_key(size, "y"), 0.25));
    out_size["height"] = yaml_double_or_default(yaml_map_key(size, "height"), yaml_double_or_default(yaml_map_key(size, "z"), 0.25));
  } else {
    out_size["width"] = yaml_double_or_default(yaml_seq_index(size, 0), 0.25);
    out_size["depth"] = yaml_double_or_default(yaml_seq_index(size, 1), 0.25);
    out_size["height"] = yaml_double_or_default(yaml_seq_index(size, 2), 0.25);
  }
  (*target)["size"] = out_size;
}

static bool append_environment_layout_canvas_item(const YAML::Node & source, const std::string & section, std::size_t index, YAML::Node * items)
{
  if (!yaml_node_is_map(source)) return false;
  YAML::Node item(YAML::NodeType::Map);
  item["id"] = canvas_item_id_from_node(source, section, index);
  std::string type = yaml_map_value_or_empty(source, "type");
  if (type.empty()) type = yaml_map_value_or_empty(source, "role");
  if (type.empty()) type = section == "camera" ? "camera" : (section == "zones" || section == "targets" ? "zone" : "object");
  item["type"] = type;
  item["role"] = yaml_map_value_or_empty(source, "role").empty() ? type : yaml_map_value_or_empty(source, "role");
  const std::string display_name = yaml_map_value_or_empty(source, "display_name");
  if (!display_name.empty()) item["display_name"] = display_name;
  const std::string name = yaml_map_value_or_empty(source, "name");
  if (!name.empty()) item["name"] = name;
  item["category"] = section;
  item["source_path"] = "environment_layout.yaml";
  item["editable"] = true;
  bool locked = false;
  if (yaml_read_bool(yaml_map_key(source, "locked"), &locked)) item["locked"] = locked;
  else item["locked"] = false;
  copy_canvas_pose_from_environment_node(source, &item);
  copy_canvas_size_from_environment_node(source, &item);
  const YAML::Node mesh = yaml_map_key(source, "mesh");
  if (yaml_node_is_defined(mesh)) item["mesh"] = YAML::Clone(mesh);
  items->push_back(item);
  return true;
}

[[maybe_unused]] static YAML::Node normalize_environment_layout_canvas_items(const YAML::Node & environment_layout)
{
  YAML::Node normalized(YAML::NodeType::Map);
  normalized["schema_version"] = "workcell_studio_layout/v1";
  YAML::Node items(YAML::NodeType::Sequence);
  const std::array<const char *, 6> sequence_keys = {"items", "assets", "placed_assets", "objects", "zones", "targets"};
  for (const char * key : sequence_keys) {
    const YAML::Node section = yaml_map_key(environment_layout, key);
    if (!yaml_node_is_sequence(section)) continue;
    std::size_t index = 0;
    for (const auto & node : section) {
      append_environment_layout_canvas_item(node, key, index, &items);
      ++index;
    }
  }
  const YAML::Node camera = yaml_map_key(environment_layout, "camera");
  if (yaml_node_is_map(camera)) {
    append_environment_layout_canvas_item(camera, "camera", 0, &items);
  } else if (yaml_node_is_sequence(camera)) {
    std::size_t index = 0;
    for (const auto & node : camera) {
      append_environment_layout_canvas_item(node, "camera", index, &items);
      ++index;
    }
  }
  normalized["items"] = items;
  return normalized;
}

static void probe_mesh_candidates(const fs::path & scene_dir, std::vector<fs::path> * visuals, std::vector<fs::path> * collisions)
{
  const std::vector<fs::path> roots = {
    scene_dir / "assets" / "environment_objects",
    scene_dir / "assets" / "robots",
    scene_dir / "assets" / "end_effectors"
  };
  for (const auto & root : roots) {
    if (!fs::exists(root)) continue;
    for (fs::recursive_directory_iterator it(root), end; it != end; ++it) {
      if (!fs::is_regular_file(it->path()) || it->path().extension() != ".stl") continue;
      const std::string p = it->path().generic_string();
      if (p.find("/meshes/visual/") != std::string::npos) visuals->push_back(it->path());
      else if (p.find("/meshes/collision/") != std::string::npos) collisions->push_back(it->path());
      else if (p.find("/assets/robots/") != std::string::npos || p.find("/assets/end_effectors/") != std::string::npos) visuals->push_back(it->path());
    }
  }
}

WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const fs::path & scene_dir, const std::string & scene_name)
{
  try {
  WorkcellStudioCanvasModel m; m.scene_name = scene_name; m.status = "WARNINGS";
  std::string deterministic_fallback_reason;
  bool deterministic_fallback_layout = false;
  const auto enable_deterministic_fallback = [&](const std::string & reason) {
    if (deterministic_fallback_layout) return;
    deterministic_fallback_layout = true;
    deterministic_fallback_reason = reason;
  };
  const auto add_warning = [&m](const std::string & context, const std::string & detail) {
    m.warnings.push_back("layout/workcell_studio_layout.yaml [" + context + "]: " + detail);
    m.has_warnings = true;
  };
  const auto read_string_or_warn = [&add_warning](const YAML::Node & node, const std::string & context, const std::string & fallback) {
    std::string out;
    if (yaml_read_string(node, &out)) return out;
    if (node.IsDefined()) add_warning(context, "expected scalar string; using default");
    return fallback;
  };
  const auto read_double_or_warn = [&add_warning](const YAML::Node & node, const std::string & context, double fallback) {
    double out = fallback;
    if (yaml_read_double(node, &out)) return out;
    if (node.IsDefined()) add_warning(context, "expected scalar number; using default");
    return fallback;
  };

  YAML::Node env, manifest, task, layout;
  const fs::path env_path = scene_dir / "environment.yaml";
  const fs::path manifest_path = scene_dir / "scene_manifest.yaml";
  const fs::path task_path = scene_dir / "config" / "task_recipe.yaml";
  const fs::path intent_path = scene_dir / "config" / "workcell_builder_task_intent.yaml";
  const fs::path layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  const fs::path legacy_layout_path = scene_dir / "environment_layout.yaml";
  const SceneMetadataSnapshot snapshot = load_scene_metadata_snapshot(scene_dir, scene_name);
  const YamlLoadStatus env_status = snapshot_yaml(snapshot, "environment.yaml", &env);
  const YamlLoadStatus manifest_status = snapshot_yaml(snapshot, "scene_manifest.yaml", &manifest);
  YamlLoadStatus task_status = snapshot_yaml(snapshot, "config/task_recipe.yaml", &task);
  if (!task_status.loaded) task_status = snapshot_yaml(snapshot, "config/workcell_builder_task_intent.yaml", &task);
  const YamlLoadStatus canonical_layout_status = snapshot_yaml(snapshot, "layout/workcell_studio_layout.yaml", &layout);
  const bool env_ok = env_status.loaded;
  const bool manifest_ok = manifest_status.loaded;
  const bool task_ok = task_status.loaded;
  bool layout_ok = false;
  if (!env_ok) m.warnings.push_back(env_status.parse_warning ?
    ("environment.yaml parse warning (" + env_path.string() + "): " + env_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Malformed or missing environment.yaml");
  if (!manifest_ok) m.warnings.push_back(manifest_status.parse_warning ?
    ("scene_manifest.yaml parse warning (" + manifest_path.string() + "): " + manifest_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Missing scene_manifest.yaml");
  if (!task_ok) m.warnings.push_back(task_status.parse_warning ?
    ("task intent parse warning (" + (task_status.exists ? task_path.string() : intent_path.string()) + "): " + task_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Task intent missing");
  m.template_name = manifest_ok ? yaml_map_value_or_empty(manifest, "template_name") : "";
  if (m.template_name.empty()) m.template_name = "unknown_template";
  m.robot_summary = env_ok ? yaml_named_or_scalar(yaml_map_key(env, "robot"), "name") : "";
  if (m.robot_summary.empty()) m.robot_summary = "Robot";
  m.tool_summary = env_ok ? yaml_named_or_scalar(yaml_map_key(env, "end_effector"), "name") : "";
  if (m.tool_summary.empty()) m.tool_summary = "Tool";
  m.pick_source = task_ok ? read_string_or_warn(yaml_map_key(task, "pick_source"), "task_recipe.pick_source", "Task intent missing") : "Task intent missing";
  m.grasp_strategy = task_ok ? read_string_or_warn(yaml_map_key(task, "grasp_strategy"), "task_recipe.grasp_strategy", "Generate task recipe to populate this panel") : "Generate task recipe to populate this panel";
  m.place_target = task_ok ? read_string_or_warn(yaml_map_key(task, "place_target"), "task_recipe.place_target", "Task intent missing") : "Task intent missing";
  m.release_strategy = task_ok ? read_string_or_warn(yaml_map_key(task, "release_strategy"), "task_recipe.release_strategy", "Generate task recipe to populate this panel") : "Generate task recipe to populate this panel";

  const std::string canonical_schema_version = canonical_layout_status.loaded ?
    read_string_or_warn(yaml_map_key(layout, "schema_version"), "schema_version", "") : "";
  const YAML::Node canonical_layout_items = canonical_layout_status.loaded ? yaml_map_key(layout, "items") : YAML::Node();
  const bool canonical_schema_current = (canonical_schema_version == "workcell_studio_layout/v1");
  const bool canonical_schema_legacy = canonical_schema_version.empty() && canonical_layout_items.IsSequence();
  const bool canonical_layout_usable = canonical_layout_status.loaded && (canonical_schema_current || canonical_schema_legacy);
  const bool canonical_layout_has_items = canonical_layout_usable && canonical_layout_items.IsSequence() && canonical_layout_items.size() > 0;

  const auto canonical_layout_has_semantic = [&](std::initializer_list<const char *> semantic_tokens) {
    if (!canonical_layout_has_items) return false;
    for (const auto & node : canonical_layout_items) {
      if (!node || !node.IsMap()) continue;
      for (const char * key : {"id", "type", "role", "category"}) {
        std::string value = yaml_map_value_or_empty(node, key);
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        for (const char * raw_token : semantic_tokens) {
          const std::string token(raw_token == nullptr ? "" : raw_token);
          if (!token.empty() && (value == token || value.find(token) != std::string::npos)) return true;
        }
      }
    }
    return false;
  };

  const auto should_add_default_preview_item = [&](const std::string & id) {
    if (!canonical_layout_has_items) return true;
    if (id == "table") return !canonical_layout_has_semantic({"table", "support_surface", "work_surface"});
    if (id == "pick_zone") return !canonical_layout_has_semantic({"pick_zone"});
    if (id == "place_zone") return !canonical_layout_has_semantic({"place_zone"});
    if (id == "bin_a") return !canonical_layout_has_semantic({"bin", "target_bin"});
    if (id == "camera") return !canonical_layout_has_semantic({"camera", "realsense"});
    if (id == "home_pose") return !canonical_layout_has_semantic({"home_pose", "safety/home"});
    return true;
  };

  const auto push_default_item = [&m, &should_add_default_preview_item](const std::string & id, const std::string & type, const std::string & role,
                                       const std::string & label, const std::string & source_file,
                                       double x, double y, double z,
                                       double roll, double pitch, double yaw,
                                       double width, double depth, double height,
                                       double radius, bool locked) {
    if (!should_add_default_preview_item(id)) return;
    WorkcellStudioCanvasItem item;
    item.id = id;
    item.type = type;
    item.category = type;
    item.role = role;
    item.label = label;
    item.source_file = source_file;
    item.x = x;
    item.y = y;
    item.z = z;
    item.roll = roll;
    item.pitch = pitch;
    item.yaw = yaw;
    item.width = width;
    item.depth = depth;
    item.height = height;
    item.radius = radius;
    item.locked = locked;
    item.editable = !locked;
    item.provenance = WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
    m.items.push_back(item);
  };

  push_default_item("robot_base", "robot_base", "robot", "Robot Base", "environment.yaml", 0, 0, 0, 0, 0, 0, 0.35, 0.35, 0.35, 0, true);
  push_default_item("robot_reach", "reach", "reach", "Robot Reach", "environment.yaml", 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, true);
  push_default_item("table", "table", "table", "Table", "environment.yaml", 0.7, 0.0, 0, 0, 0, 0, 1.0, 0.6, 0.2, 0, false);
  push_default_item("conveyor", "conveyor", "conveyor", "Conveyor", "environment.yaml", -0.8, -0.3, 0, 0, 0, 0, 1.2, 0.3, 0.2, 0, false);
  push_default_item("camera", "camera", "camera", "Camera FOV", "environment.yaml", -0.2, 1.0, 1.2, 0, 0, -1.57, 0.18, 0.18, 0.2, 0, false);
  push_default_item("pick_zone", "zone", "pick_zone", "Pick Source", "config/task_recipe.yaml", 0.55, 0.0, 0, 0, 0, 0, 0.35, 0.35, 0.1, 0, false);
  push_default_item("place_zone", "zone", "place_zone", "Place Target", "config/task_recipe.yaml", 1.0, 0.1, 0, 0, 0, 0, 0.35, 0.35, 0.1, 0, false);
  push_default_item("bin_a", "bin", "bin", "Bin A", "environment.yaml", 1.3, -0.5, 0, 0, 0, 0, 0.32, 0.25, 0.2, 0, false);
  push_default_item("object_a", "object", "object", "Object", "environment.yaml", 0.6, 0.1, 0, 0, 0, 0, 0.08, 0.08, 0.08, 0, false);
  push_default_item("home_pose", "safety", "safety/home", "config/workcell_builder_task_intent.yaml", "config/workcell_builder_task_intent.yaml", -0.4, 0.5, 0, 0, 0, 0, 0.14, 0.14, 0.14, 0, true);


  const auto copy_primitive_metadata = [&](WorkcellStudioCanvasItem & item, const YAML::Node & node) {
    const std::string primitive_type = read_string_or_warn(yaml_map_key(node, "primitive_geometry_type"), "items[].primitive_geometry_type",
      read_string_or_warn(yaml_map_key(node, "geometry_type"), "items[].geometry_type", ""));
    if (!primitive_type.empty() && primitive_type != "mesh") item.primitive_geometry_type = primitive_type;
    item.primitive_radius = read_double_or_warn(yaml_map_key(node, "primitive_radius"), "items[].primitive_radius",
      read_double_or_warn(yaml_map_key(node, "radius"), "items[].radius", item.primitive_radius));
    item.primitive_length = read_double_or_warn(yaml_map_key(node, "primitive_length"), "items[].primitive_length",
      read_double_or_warn(yaml_map_key(node, "length"), "items[].length", item.primitive_length));
    item.material_name = read_string_or_warn(yaml_map_key(node, "material_name"), "items[].material_name", item.material_name);
    const YAML::Node material = yaml_map_key(node, "material");
    const YAML::Node color = yaml_node_is_map(material) ? yaml_map_key(material, "color") : yaml_map_key(node, "material_color");
    if (yaml_node_is_sequence(color) && color.size() >= 3) {
      item.has_material_color = true;
      item.material_r = read_double_or_warn(yaml_seq_index(color, 0), "items[].material.color[0]", item.material_r);
      item.material_g = read_double_or_warn(yaml_seq_index(color, 1), "items[].material.color[1]", item.material_g);
      item.material_b = read_double_or_warn(yaml_seq_index(color, 2), "items[].material.color[2]", item.material_b);
      item.material_a = read_double_or_warn(yaml_seq_index(color, 3), "items[].material.color[3]", item.material_a);
    }
  };

  const auto append_layout_load_message = [&m](const std::string & message) {
    if (message.empty()) return;
    if (!m.layout_load_message.empty()) m.layout_load_message += "\n";
    m.layout_load_message += message;
  };
  const auto layout_parse_reason = [](const YamlLoadStatus & status) {
    if (!status.reason.empty()) return status.reason;
    return std::string("unknown parse error");
  };
  const auto append_legacy_source_items = [](const YAML::Node & root, YAML::Node * items) {
    if (!root || !root.IsMap() || items == nullptr) return;
    const std::array<const char *, 6> legacy_sequence_keys = {
      "items", "assets", "placed_assets", "objects", "zones", "targets"
    };
    for (const char * key : legacy_sequence_keys) {
      const YAML::Node seq = yaml_map_key(root, key);
      if (!seq || !seq.IsSequence()) continue;
      for (const auto & node : seq) {
        if (node && node.IsMap()) items->push_back(YAML::Clone(node));
      }
    }
    const YAML::Node camera = yaml_map_key(root, "camera");
    if (camera && camera.IsMap()) items->push_back(YAML::Clone(camera));
  };

  YAML::Node effective_layout;
  bool layout_source_is_environment_layout = false;
  std::string layout_source_file = "layout/workcell_studio_layout.yaml";
  if (canonical_layout_usable) {
    effective_layout = layout;
    layout_ok = true;
    m.layout_source_path = layout_path.string();
    m.layout_source_kind = "canonical";
    const bool empty_items = !canonical_layout_items.IsSequence() || canonical_layout_items.size() == 0;
    append_layout_load_message((empty_items ?
      "Loaded empty canonical layout metadata from " :
      "Loaded canonical layout from ") + layout_path.string());
  } else {
    if (canonical_layout_status.exists) {
      const std::string reason = canonical_layout_status.loaded ?
        "invalid or missing schema_version" : layout_parse_reason(canonical_layout_status);
      append_layout_load_message("Failed to load layout " + layout_path.string() + ": " + reason + "; using next fallback");
      const std::string parse_reason = canonical_layout_status.parse_warning ? (": " + canonical_layout_status.reason) : "";
      m.warnings.push_back("Malformed layout/workcell_studio_layout.yaml (" + layout_path.string() + ")" + parse_reason +
                           "; falling back safely. Repair guidance: run YAML validation (e.g., `yamllint`) and fix syntax/indentation.");
    }

    YAML::Node legacy_layout;
    const YamlLoadStatus legacy_layout_status = snapshot_yaml(snapshot, "environment_layout.yaml", &legacy_layout);
    if (legacy_layout_status.loaded) {
      YAML::Node legacy_items(YAML::NodeType::Sequence);
      append_legacy_source_items(legacy_layout, &legacy_items);
      if (legacy_items.size() > 0) {
        effective_layout = YAML::Node(YAML::NodeType::Map);
        effective_layout["schema_version"] = "workcell_studio_layout/v1";
        effective_layout["schema"] = "workcell_studio_layout/v1";
        effective_layout["scene_name"] = scene_name;
        effective_layout["scene_path"] = ".";
        effective_layout["items"] = legacy_items;
        layout_ok = true;
        m.layout_source_path = legacy_layout_path.string();
        m.layout_source_kind = "legacy";
        layout_source_is_environment_layout = true;
        layout_source_file = "environment_layout.yaml";
        append_layout_load_message("Imported legacy layout from " + legacy_layout_path.string());
      }
    } else if (legacy_layout_status.exists) {
      append_layout_load_message("Failed to load layout " + legacy_layout_path.string() + ": " +
                                 layout_parse_reason(legacy_layout_status) + "; using next fallback");
    }
  }

  if (layout_ok) {
    layout = effective_layout;
  } else {
    m.layout_source_path.clear();
    m.layout_source_kind = "locked_preview_fallback";
    append_layout_load_message("No editable layout source found; using locked preview fallback");
    if (canonical_layout_status.exists) enable_deterministic_fallback("layout/workcell_studio_layout.yaml is malformed");
    else enable_deterministic_fallback("layout/workcell_studio_layout.yaml is missing");
  }

  const std::string schema_version = layout_ok ? read_string_or_warn(yaml_map_key(layout, "schema_version"), "schema_version", "") : "";
  YAML::Node layout_items = layout_ok ? yaml_map_key(layout, "items") : YAML::Node();
  const bool schema_current = (schema_version == "workcell_studio_layout/v1");
  const bool schema_legacy = schema_version.empty() && layout_items.IsSequence();
  if (layout_ok && (schema_current || schema_legacy)) {
    bool incomplete_placement_metadata = false;
    if (!layout_items.IsDefined() || !layout_items.IsSequence()) {
      if (layout_items.IsDefined()) add_warning("items", "expected sequence; skipping malformed items");
    } else {
      for (const auto & node : layout_items) {
        if (!node.IsMap()) { add_warning("items[]", "expected map item; skipping"); continue; }
        const auto id = read_string_or_warn(yaml_map_key(node, "id"), "items[].id", "");
        if (id.empty()) continue;
        bool matched_existing = false;
        for (auto & item : m.items) {
          if (item.id != id) continue;
          matched_existing = true;
          item.provenance = WorkcellStudioItemProvenance::EditableLayout;
          if (layout_source_is_environment_layout) {
            item.source_file = "environment_layout.yaml";
            bool locked = false;
            item.locked = yaml_read_bool(yaml_map_key(node, "locked"), &locked) ? locked : false;
          }
          copy_primitive_metadata(item, node);
          YAML::Node pose = yaml_map_key(node, "pose");
          if (pose.IsDefined() && pose.IsMap()) {
            YAML::Node xyz = yaml_map_key(pose, "xyz");
            if (xyz.IsDefined() && xyz.IsSequence()) {
              if (xyz.size() < 3) incomplete_placement_metadata = true;
              item.x = read_double_or_warn(yaml_seq_index(xyz, 0), "items[].pose.xyz[0]", item.x);
              item.y = read_double_or_warn(yaml_seq_index(xyz, 1), "items[].pose.xyz[1]", item.y);
              item.z = read_double_or_warn(yaml_seq_index(xyz, 2), "items[].pose.xyz[2]", item.z);
            } else if (xyz.IsDefined()) { add_warning("items[].pose.xyz", "expected sequence; using defaults"); incomplete_placement_metadata = true; }
            else { incomplete_placement_metadata = true; }
            YAML::Node rpy = yaml_map_key(pose, "rpy");
            if (rpy.IsDefined() && rpy.IsSequence()) {
              if (rpy.size() < 3) incomplete_placement_metadata = true;
              item.roll = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].pose.rpy[0]", item.roll);
              item.pitch = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].pose.rpy[1]", item.pitch);
              item.yaw = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].pose.rpy[2]", item.yaw);
            } else if (rpy.IsDefined()) { add_warning("items[].pose.rpy", "expected sequence; using defaults"); incomplete_placement_metadata = true; }
            else { incomplete_placement_metadata = true; }
          } else if (pose.IsDefined()) {
            add_warning("items[].pose", "expected map; using defaults");
            incomplete_placement_metadata = true;
          } else {
            incomplete_placement_metadata = true;
          }
          YAML::Node size = yaml_map_key(node, "size");
          if (yaml_node_is_defined(size)) {
            if (yaml_node_is_map(size)) {
              item.width = read_double_or_warn(yaml_map_key(size, "width"), "items[].size.width", item.width);
              item.depth = read_double_or_warn(yaml_map_key(size, "depth"), "items[].size.depth", item.depth);
              item.height = read_double_or_warn(yaml_map_key(size, "height"), "items[].size.height", item.height);
            } else if (yaml_node_is_sequence(size)) {
              item.width = read_double_or_warn(yaml_seq_index(size, 0), "items[].size[0]", item.width);
              item.depth = read_double_or_warn(yaml_seq_index(size, 1), "items[].size[1]", item.depth);
              item.height = read_double_or_warn(yaml_seq_index(size, 2), "items[].size[2]", item.height);
            } else {
              add_warning("items[].size", "expected map or sequence; using defaults");
            }
          } else {
            const YAML::Node dimensions = yaml_map_key(node, "dimensions");
            if (yaml_node_is_sequence(dimensions)) {
              item.width = read_double_or_warn(yaml_seq_index(dimensions, 0), "items[].dimensions[0]", item.width);
              item.depth = read_double_or_warn(yaml_seq_index(dimensions, 1), "items[].dimensions[1]", item.depth);
              item.height = read_double_or_warn(yaml_seq_index(dimensions, 2), "items[].dimensions[2]", item.height);
            }
          }
        }
        if (!matched_existing) {
          WorkcellStudioCanvasItem extra;
          extra.id = id;
          extra.type = read_string_or_warn(yaml_map_key(node, "type"), "items[].type", "object");
          extra.role = read_string_or_warn(yaml_map_key(node, "role"), "items[].role", "preview");
          const auto category = read_string_or_warn(yaml_map_key(node, "category"), "items[].category", "Custom / Imported");
          extra.category = category;
          if (category == "Pick/Place Zones") extra.type = "zone";
          std::string label = read_string_or_warn(yaml_map_key(node, "display_name"), "items[].display_name", "");
          if (label.empty()) label = read_string_or_warn(yaml_map_key(node, "name"), "items[].name", id);
          extra.label = label.empty() ? id : label;
          std::string source_file = read_string_or_warn(yaml_map_key(node, "source_path"), "items[].source_path", "");
          if (source_file.empty()) source_file = read_string_or_warn(yaml_map_key(node, "source_layer"), "items[].source_layer", layout_source_file);
          extra.source_file = source_file.empty() ? layout_source_file : source_file;
          extra.provenance = WorkcellStudioItemProvenance::EditableLayout;
          copy_primitive_metadata(extra, node);
          YAML::Node pose = yaml_map_key(node, "pose");
          if (pose.IsDefined() && pose.IsMap()) {
            YAML::Node xyz = yaml_map_key(pose, "xyz");
            if (xyz.IsDefined() && xyz.IsSequence()) {
              extra.x = read_double_or_warn(yaml_seq_index(xyz, 0), "items[].pose.xyz[0]", extra.x);
              extra.y = read_double_or_warn(yaml_seq_index(xyz, 1), "items[].pose.xyz[1]", extra.y);
              extra.z = read_double_or_warn(yaml_seq_index(xyz, 2), "items[].pose.xyz[2]", extra.z);
            } else if (xyz.IsDefined()) add_warning("items[].pose.xyz", "expected sequence; using defaults");
            YAML::Node rpy = yaml_map_key(pose, "rpy");
            if (rpy.IsDefined() && rpy.IsSequence()) {
              extra.roll = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].pose.rpy[0]", extra.roll);
              extra.pitch = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].pose.rpy[1]", extra.pitch);
              extra.yaw = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].pose.rpy[2]", extra.yaw);
            } else if (rpy.IsDefined()) add_warning("items[].pose.rpy", "expected sequence; using defaults");
          } else if (pose.IsDefined()) add_warning("items[].pose", "expected map; using defaults");
          YAML::Node size = yaml_map_key(node, "size");
          if (yaml_node_is_defined(size)) {
            if (yaml_node_is_map(size)) {
              extra.width = read_double_or_warn(yaml_map_key(size, "width"), "items[].size.width", extra.width);
              extra.depth = read_double_or_warn(yaml_map_key(size, "depth"), "items[].size.depth", extra.depth);
              extra.height = read_double_or_warn(yaml_map_key(size, "height"), "items[].size.height", extra.height);
            } else if (yaml_node_is_sequence(size)) {
              extra.width = read_double_or_warn(yaml_seq_index(size, 0), "items[].size[0]", extra.width);
              extra.depth = read_double_or_warn(yaml_seq_index(size, 1), "items[].size[1]", extra.depth);
              extra.height = read_double_or_warn(yaml_seq_index(size, 2), "items[].size[2]", extra.height);
            } else {
              add_warning("items[].size", "expected map or sequence; using defaults");
            }
          } else {
            const YAML::Node dimensions = yaml_map_key(node, "dimensions");
            if (yaml_node_is_sequence(dimensions)) {
              extra.width = read_double_or_warn(yaml_seq_index(dimensions, 0), "items[].dimensions[0]", extra.width);
              extra.depth = read_double_or_warn(yaml_seq_index(dimensions, 1), "items[].dimensions[1]", extra.depth);
              extra.height = read_double_or_warn(yaml_seq_index(dimensions, 2), "items[].dimensions[2]", extra.height);
            }
          }
          m.items.push_back(extra);
        }
      }
    }
    if (incomplete_placement_metadata) {
      enable_deterministic_fallback("layout/workcell_studio_layout.yaml has incomplete placement metadata");
    }
  } else if (layout_ok) {
    enable_deterministic_fallback(layout_source_file + " has invalid or missing schema_version");
  }

  if (deterministic_fallback_layout) {
    for (auto & item : m.items) {
      item.provenance = WorkcellStudioItemProvenance::StaticFallbackPreview;
      if (item.id == "robot_base" || item.role == "robot") { item.x = -0.90; item.y = 0.0; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "table" || item.role == "table") { item.x = 0.0; item.y = 0.0; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "conveyor" || item.role == "conveyor") { item.x = -1.30; item.y = -0.60; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "pick_zone" || item.role == "pick_zone") { item.x = -0.35; item.y = 0.15; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "place_zone" || item.role == "place_zone") { item.x = 0.75; item.y = 0.15; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "bin_a" || item.role == "bin") { item.x = 1.10; item.y = -0.35; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "camera" || item.role == "camera") { item.x = -0.10; item.y = 0.95; item.z = 1.45; item.roll = 0.0; item.pitch = -0.40; item.yaw = -1.20; }
      else if (item.id == "home_pose" || item.role.find("safety") != std::string::npos) { item.x = 0.0; item.y = 0.95; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "object_a" || item.role == "object") { item.x = -0.25; item.y = 0.10; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
    }
    m.warnings.push_back("Using deterministic 3D fallback layout because all editable layout sources are missing or invalid.");
    if (!deterministic_fallback_reason.empty()) m.warnings.push_back("Fallback detail: " + deterministic_fallback_reason + ".");
  }

  std::vector<fs::path> probed_visuals;
  std::vector<fs::path> probed_collisions;
  probe_mesh_candidates(scene_dir, &probed_visuals, &probed_collisions);
  const bool safe_generated_visual_mesh_index_available =
    safe_visual_mesh_index_has_matching_visual_items(scene_dir);
  const auto choose_probed = [](const WorkcellStudioCanvasItem & item, const std::vector<fs::path> & pool)->fs::path {
    for (const auto & p : pool) {
      const std::string b = p.stem().string();
      if (b == item.id || b == item.type || b == item.role) return p;
    }
    return fs::path();
  };

  for (auto & item : m.items) {
    item.mesh_available = false;
    item.mesh_path.clear();
    item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
    const auto candidates = gather_mesh_candidates(env, manifest, layout_items, item.id);
    fs::path visual;
    fs::path collision;
    for (const auto & c : candidates) {
      const fs::path resolved = resolve_mesh_candidate(c, scene_dir);
      if (!fs::exists(resolved)) continue;
      const auto text = resolved.generic_string();
      if (text.find("/visual/") != std::string::npos) visual = resolved;
      else if (text.find("/collision/") != std::string::npos) collision = resolved;
      else if (visual.empty()) visual = resolved;
    }
    if (visual.empty()) visual = choose_probed(item, probed_visuals);
    if (collision.empty()) collision = choose_probed(item, probed_collisions);

    if (!visual.empty()) {
      item.mesh_path = visual.generic_string();
      item.mesh_available = true;
      item.mesh_load_warning.clear();
    } else if (!collision.empty()) {
      item.mesh_path = collision.generic_string();
      item.mesh_available = true;
      item.mesh_load_warning = "Mesh preview fallback for " + item.id + ": visual mesh unavailable; using collision mesh";
      m.warnings.push_back(item.mesh_load_warning);
    } else {
      item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
      if (!(safe_generated_visual_mesh_index_available &&
            item.provenance == WorkcellStudioItemProvenance::GeneratedOrLegacyPreview &&
            item.locked)) {
        m.warnings.push_back(item.mesh_load_warning);
      }
    }

    if (layout_items.IsSequence()) {
      for (const auto & node : layout_items) {
        if (!node.IsMap() || yaml_map_value_or_empty(node, "id") != item.id) continue;
        const YAML::Node mesh = yaml_map_key(node, "mesh");
        if (!yaml_node_is_defined(mesh) || mesh_node_disabled(mesh)) {
          item.mesh_available = false;
          item.mesh_path.clear();
          item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
        } else if (yaml_node_is_map(mesh)) {
          item.has_mesh_metadata = true;
          const std::string path = get_optional_string(mesh, "path", "");
          const std::string source_package = get_optional_string(mesh, "source_package", item.mesh_source_package);
          if (!source_package.empty()) item.mesh_source_package = source_package;
          if (path.empty()) {
            item.mesh_available = false;
            item.mesh_path.clear();
            item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
          } else {
            std::string candidate = path;
            if (!source_package.empty() && path.rfind("package://", 0) != 0 && !fs::path(path).is_absolute()) {
              candidate = "package://" + source_package + "/" + path;
            }
            const fs::path resolved_path = resolve_mesh_candidate(candidate, scene_dir);
            item.mesh_path = resolved_path.generic_string();
          }
          const YAML::Node scale = yaml_map_key(mesh, "scale");
          if (yaml_node_is_sequence(scale)) {
            item.mesh_scale_x = read_double_or_warn(yaml_seq_index(scale, 0), "items[].mesh.scale[0]", item.mesh_scale_x);
            item.mesh_scale_y = read_double_or_warn(yaml_seq_index(scale, 1), "items[].mesh.scale[1]", item.mesh_scale_y);
            item.mesh_scale_z = read_double_or_warn(yaml_seq_index(scale, 2), "items[].mesh.scale[2]", item.mesh_scale_z);
          }
          const YAML::Node rpy = yaml_map_key(mesh, "rpy");
          if (yaml_node_is_sequence(rpy)) {
            item.mesh_r = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].mesh.rpy[0]", item.mesh_r);
            item.mesh_p = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].mesh.rpy[1]", item.mesh_p);
            item.mesh_y = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].mesh.rpy[2]", item.mesh_y);
          }
          const YAML::Node origin = yaml_map_key(mesh, "origin_offset");
          if (yaml_node_is_sequence(origin)) {
            item.has_origin_offset = true;
            item.origin_offset_x = read_double_or_warn(yaml_seq_index(origin, 0), "items[].mesh.origin_offset[0]", item.origin_offset_x);
            item.origin_offset_y = read_double_or_warn(yaml_seq_index(origin, 1), "items[].mesh.origin_offset[1]", item.origin_offset_y);
            item.origin_offset_z = read_double_or_warn(yaml_seq_index(origin, 2), "items[].mesh.origin_offset[2]", item.origin_offset_z);
          }
        }
      }
    }
  }

  std::stable_sort(m.items.begin(), m.items.end(), [](const WorkcellStudioCanvasItem & a, const WorkcellStudioCanvasItem & b) {
    return a.id < b.id;
  });
  for (const auto & item : m.items) {
    if (item.provenance == WorkcellStudioItemProvenance::EditableLayout) ++m.provenance_status.editable_layout_count;
    else if (item.provenance == WorkcellStudioItemProvenance::GeneratedOrLegacyPreview) ++m.provenance_status.generated_or_legacy_preview_count;
    else ++m.provenance_status.static_fallback_preview_count;
  }
  m.provenance_status.summary = provenance_summary_text(m.provenance_status);
  if (!m.warnings.empty()) { m.has_warnings = true; m.status = "WARNINGS"; { WorkcellStudioCanvasItem w; w.id="warning"; w.type="warning"; w.role="warning"; w.label="warning"; w.source_file="environment.yaml"; w.x=-1.2; w.y=1.2; w.width=0.1; w.depth=0.1; w.height=0.0; w.warnings=m.warnings; m.items.push_back(w); } }
  else { m.status = "READY"; }
  return m;
  } catch (const YAML::Exception & e) {
    log_task_metadata_loader_warning_once(scene_dir / "layout" / "workcell_studio_layout.yaml", std::string("YAML parse exception in preview loader: ") + e.what());
  } catch (const std::exception & e) {
    log_task_metadata_loader_warning_once(scene_dir / "layout" / "workcell_studio_layout.yaml", std::string("std exception in preview loader: ") + e.what());
  }
  WorkcellStudioCanvasModel fallback;
  fallback.scene_name = scene_name;
  fallback.status = "WARNINGS";
  fallback.has_warnings = true;
  fallback.warnings.push_back("mesh metadata missing or legacy; using primitive preview");
  fallback.provenance_status.summary = provenance_summary_text(fallback.provenance_status);
  return fallback;
}

static std::string lower_scalar_or_empty(const YAML::Node & node)
{
  std::string value;
  if (!yaml_read_string(node, &value)) return "";
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value;
}

static bool layout_item_is_fallback_preview_entry(const YAML::Node & node)
{
  const std::array<std::string, 5> marker_keys = {"source_layer", "source_file", "source_path", "provenance", "preview_source"};
  for (const auto & key : marker_keys) {
    const std::string value = lower_scalar_or_empty(yaml_map_key(node, key.c_str()));
    if (value.find("fallback") != std::string::npos || value.find("static_preview") != std::string::npos) return true;
  }
  return false;
}

static bool layout_item_is_effectively_editable(const YAML::Node & node)
{
  if (!node || !node.IsMap()) return false;

  bool locked = false;
  const YAML::Node locked_node = yaml_map_key(node, "locked");
  if (yaml_read_bool(locked_node, &locked) && locked) return false;
  if (locked_node.IsDefined() && !yaml_read_bool(locked_node, &locked)) return false;

  bool editable = false;
  const YAML::Node editable_node = yaml_map_key(node, "editable");
  if (yaml_read_bool(editable_node, &editable)) return editable;
  if (editable_node.IsDefined()) return false;

  if (layout_item_is_fallback_preview_entry(node)) return false;
  return true;
}


static YAML::Node ensure_map_child(YAML::Node parent, const char * key)
{
  if (!parent[key] || !parent[key].IsMap()) parent[key] = YAML::Node(YAML::NodeType::Map);
  return parent[key];
}

static YAML::Node ensure_sequence_child(YAML::Node parent, const char * key)
{
  if (!parent[key] || !parent[key].IsSequence()) parent[key] = YAML::Node(YAML::NodeType::Sequence);
  return parent[key];
}

static bool yaml_nodes_equal(const YAML::Node & lhs, const YAML::Node & rhs)
{
  return YAML::Dump(lhs) == YAML::Dump(rhs);
}

static bool layout_item_has_bootstrap_placeable_fields(const YAML::Node & item)
{
  if (!item || !item.IsMap()) return false;
  if (yaml_map_value_or_empty(item, "id").empty()) return false;
  const YAML::Node pose = yaml_map_key(item, "pose");
  const YAML::Node xyz = yaml_map_key(pose, "xyz");
  const YAML::Node rpy = yaml_map_key(pose, "rpy");
  const YAML::Node dimensions = yaml_map_key(item, "dimensions");
  if (!pose || !pose.IsMap() || !xyz || !xyz.IsSequence() || xyz.size() < 3) return false;
  if (!rpy || !rpy.IsSequence() || rpy.size() < 3) return false;
  if (!dimensions || !dimensions.IsSequence() || dimensions.size() < 3) return false;
  if (yaml_map_value_or_empty(item, "source").empty()) return false;
  bool editable = false;
  bool locked = true;
  if (!yaml_read_bool(yaml_map_key(item, "editable"), &editable) || !editable) return false;
  if (!yaml_read_bool(yaml_map_key(item, "locked"), &locked) || locked) return false;
  return layout_item_is_effectively_editable(item);
}

static YAML::Node bootstrap_environment_asset_from_layout_item(const YAML::Node & existing, const YAML::Node & layout_item)
{
  YAML::Node out = (existing && existing.IsMap()) ? YAML::Clone(existing) : YAML::Node(YAML::NodeType::Map);
  out["id"] = yaml_map_key(layout_item, "id");
  const YAML::Node type = yaml_map_key(layout_item, "type");
  if (type.IsDefined()) out["type"] = type;
  const YAML::Node category = yaml_map_key(layout_item, "category");
  if (category.IsDefined()) out["category"] = category;
  YAML::Node pose = ensure_map_child(out, "pose");
  pose["xyz"] = YAML::Clone(yaml_map_key(yaml_map_key(layout_item, "pose"), "xyz"));
  pose["rpy"] = YAML::Clone(yaml_map_key(yaml_map_key(layout_item, "pose"), "rpy"));
  out["dimensions"] = YAML::Clone(yaml_map_key(layout_item, "dimensions"));
  out["source"] = yaml_map_key(layout_item, "source");
  out["editable"] = yaml_map_key(layout_item, "editable");
  out["locked"] = yaml_map_key(layout_item, "locked");
  const YAML::Node mesh = yaml_map_key(layout_item, "mesh");
  if (mesh.IsDefined()) out["mesh"] = YAML::Clone(mesh);
  return out;
}

WorkcellStudioEditableLayoutInspection inspect_editable_layout_entries(const fs::path & scene_dir)
{
  WorkcellStudioEditableLayoutInspection out;
  const fs::path layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  out.exists = fs::exists(layout_path);
  YAML::Node layout;
  if (!read_yaml(layout_path, &layout).loaded) return out;
  const std::string schema_version = yaml_map_value_or_empty(layout, "schema_version");
  const YAML::Node items = yaml_map_key(layout, "items");
  const bool schema_current = (schema_version == "workcell_studio_layout/v1");
  const bool schema_legacy = schema_version.empty() && items.IsSequence();
  if (!schema_current && !schema_legacy) return out;
  out.valid = true;
  if (!items || !items.IsSequence()) return out;
  out.has_items_sequence = true;
  for (const auto & node : items) {
    if (!node || !node.IsMap()) continue;
    ++out.total_item_entries;
    if (layout_item_is_effectively_editable(node)) ++out.editable_item_count;
  }
  return out;
}

std::size_t count_editable_layout_entries(const fs::path & scene_dir)
{
  return inspect_editable_layout_entries(scene_dir).editable_item_count;
}

bool is_save_layout_workflow_ready(const fs::path & scene_dir)
{
  const WorkcellStudioEditableLayoutInspection layout = inspect_editable_layout_entries(scene_dir);
  return layout.exists && layout.valid && layout.editable_item_count > 0 &&
    fs::exists(scene_dir / "environment_layout.yaml") &&
    fs::exists(scene_dir / "environment.yaml");
}


static std::string canonical_item_id_from_node(const YAML::Node & node)
{
  std::string id = yaml_map_value_or_empty(node, "id");
  if (id.empty()) id = yaml_map_value_or_empty(node, "name");
  if (id.empty()) id = yaml_map_value_or_empty(node, "asset_id");
  return id;
}

static std::string canonical_item_type_from_node(const YAML::Node & node)
{
  std::string type = yaml_map_value_or_empty(node, "type");
  if (type.empty()) type = yaml_map_value_or_empty(node, "role");
  if (type.empty()) type = yaml_map_value_or_empty(node, "source");
  return type.empty() ? "object" : type;
}

static void copy_pose_from_canonical_node(const YAML::Node & source, YAML::Node * target)
{
  YAML::Node pose = yaml_map_key(source, "pose");
  YAML::Node xyz;
  YAML::Node rpy;
  if (yaml_node_is_map(pose)) {
    xyz = yaml_map_key(pose, "xyz");
    rpy = yaml_map_key(pose, "rpy");
  } else if (yaml_node_is_sequence(pose)) {
    xyz = pose;
  }

  YAML::Node out_pose(YAML::NodeType::Map);
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 0), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 1), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 2), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 0), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 1), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 2), 0.0));
  (*target)["pose"] = out_pose;
}

static void copy_dimensions_from_canonical_node(const YAML::Node & source, YAML::Node * target)
{
  try {
    YAML::Node size = yaml_map_key(source, "size");
    if (!yaml_node_is_defined(size)) size = yaml_map_key(source, "dimensions");
    if (yaml_node_is_map(size)) {
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "width"), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "depth"), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "height"), 0.25));
    } else if (yaml_node_is_sequence(size)) {
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 0), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 1), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 2), 0.25));
    }
  } catch (const YAML::Exception &) {
  } catch (const std::exception &) {
  }
}

static bool append_canonical_layout_item(const YAML::Node & source, const std::string & source_name, YAML::Node * items)
{
  if (!yaml_node_is_map(source)) return false;
  const std::string id = canonical_item_id_from_node(source);
  if (id.empty()) return false;

  YAML::Node item(YAML::NodeType::Map);
  item["id"] = id;
  const std::string type = canonical_item_type_from_node(source);
  item["type"] = type;
  item["category"] = type;
  copy_pose_from_canonical_node(source, &item);
  copy_dimensions_from_canonical_node(source, &item);

  const YAML::Node mesh = yaml_map_key(source, "mesh");
  const std::string mesh_path = yaml_node_is_map(mesh) ? yaml_map_value_or_empty(mesh, "path") : yaml_scalar_or_empty(mesh);
  if (!mesh_path.empty()) {
    YAML::Node mesh_out(YAML::NodeType::Map);
    mesh_out["path"] = mesh_path;
    mesh_out["source"] = source_name;
    item["mesh"] = mesh_out;
  }
  const std::string collision_mesh = yaml_map_value_or_empty(source, "collision_mesh");
  if (!collision_mesh.empty() && !item["mesh"].IsDefined()) {
    YAML::Node mesh_out(YAML::NodeType::Map);
    mesh_out["path"] = collision_mesh;
    mesh_out["source"] = source_name;
    item["mesh"] = mesh_out;
  }
  item["source"] = source_name;
  item["editable"] = true;
  item["locked"] = false;
  items->push_back(item);
  return true;
}

WorkcellStudioStarterLayoutSummary bootstrap_editable_layout_from_trusted_canonical_yaml(const fs::path & scene_dir, const std::string & scene_name)
{
  WorkcellStudioStarterLayoutSummary summary;
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "workcell_studio_layout/v1";
  root["scene_name"] = scene_name;
  YAML::Node items(YAML::NodeType::Sequence);

  YAML::Node env;
  const YamlLoadStatus env_status = read_yaml(scene_dir / "environment.yaml", &env);
  if (env_status.loaded) {
    const YAML::Node placed_objects = yaml_map_key(env, "placed_objects");
    if (yaml_node_is_sequence(placed_objects)) {
      for (const auto & node : placed_objects) {
        ++summary.total_preview_items;
        if (append_canonical_layout_item(node, "environment.yaml", &items)) ++summary.editable_items_created;
        else ++summary.skipped_unsafe_or_missing_metadata_items;
      }
    }
    const YAML::Node workspace = yaml_map_key(env, "workspace");
    const YAML::Node workspace_zones = yaml_node_is_map(workspace) ? yaml_map_key(workspace, "zones") : YAML::Node();
    if (yaml_node_is_sequence(workspace_zones)) {
      for (const auto & node : workspace_zones) {
        ++summary.total_preview_items;
        if (append_canonical_layout_item(node, "environment.yaml", &items)) ++summary.editable_items_created;
        else ++summary.skipped_unsafe_or_missing_metadata_items;
      }
    }
    const YAML::Node camera = yaml_map_key(env, "camera");
    bool camera_enabled = false;
    if (yaml_node_is_map(camera) && yaml_read_bool(yaml_map_key(camera, "enabled"), &camera_enabled) && camera_enabled) {
      ++summary.total_preview_items;
      YAML::Node camera_item = YAML::Clone(camera);
      if (!camera_item["id"]) camera_item["id"] = "camera";
      if (!camera_item["type"]) camera_item["type"] = "camera";
      if (append_canonical_layout_item(camera_item, "environment.yaml", &items)) ++summary.editable_items_created;
      else ++summary.skipped_unsafe_or_missing_metadata_items;
    }
  } else {
    ++summary.skipped_unsafe_or_missing_metadata_items;
  }

  if (summary.editable_items_created == 0) {
    const auto model = build_workcell_studio_canvas_model(scene_dir, scene_name);
    const auto preview_summary = build_starter_layout_entries_from_preview(model);
    summary.total_preview_items += preview_summary.total_preview_items;
    summary.skipped_locked_items += preview_summary.skipped_locked_items;
    summary.skipped_static_fallback_items += preview_summary.skipped_static_fallback_items;
    summary.skipped_unsafe_or_missing_metadata_items += preview_summary.skipped_unsafe_or_missing_metadata_items;
    if (preview_summary.editable_items_created > 0) {
      root = preview_summary.layout;
      summary.editable_items_created = preview_summary.editable_items_created;
      summary.layout = root;
      return summary;
    }
  }

  root["empty_layout_marker"] = summary.editable_items_created == 0;
  root["items"] = items;
  summary.layout = root;
  return summary;
}

WorkcellStudioEnvironmentLayoutBootstrapResult bootstrap_environment_layout_from_editable_layout(
  const fs::path & scene_dir, const std::string & scene_name, const YAML::Node & editable_layout)
{
  WorkcellStudioEnvironmentLayoutBootstrapResult result;
  const YAML::Node layout_items = yaml_map_key(editable_layout, "items");
  if (!layout_items || !layout_items.IsSequence()) {
    result.error = "editable layout has no items sequence";
    return result;
  }

  YAML::Node env_root(YAML::NodeType::Map);
  const fs::path env_layout_path = scene_dir / "environment_layout.yaml";
  result.created = !fs::exists(env_layout_path);
  if (!result.created) {
    try {
      env_root = YAML::LoadFile(env_layout_path.string());
    } catch (const std::exception & e) {
      result.error = std::string("failed to parse environment_layout.yaml: ") + e.what();
      return result;
    }
    if (!env_root || !env_root.IsMap()) env_root = YAML::Node(YAML::NodeType::Map);
  }

  YAML::Node before = YAML::Clone(env_root);
  env_root["schema_version"] = "environment_layout/v1";
  if (!env_root["scene_name"] || !env_root["scene_name"].IsScalar()) env_root["scene_name"] = scene_name;

  YAML::Node placed_assets = ensure_sequence_child(env_root, "placed_assets");

  std::set<std::string> layout_ids;
  std::vector<std::string> layout_id_order;
  YAML::Node bootstrap_by_id(YAML::NodeType::Map);
  for (YAML::const_iterator it = layout_items.begin(); it != layout_items.end(); ++it) {
    const YAML::Node layout_item = *it;
    if (!layout_item_has_bootstrap_placeable_fields(layout_item)) continue;
    const std::string id = yaml_map_value_or_empty(layout_item, "id");
    if (layout_ids.insert(id).second) layout_id_order.push_back(id);
    bootstrap_by_id[id] = YAML::Clone(layout_item);
  }
  if (layout_ids.empty()) {
    result.error = "editable layout has no bootstrappable editable/placeable items";
    return result;
  }

  YAML::Node updated_placed_assets(YAML::NodeType::Sequence);
  std::set<std::string> emitted_ids;
  for (std::size_t i = 0; i < placed_assets.size(); ++i) {
    const YAML::Node existing = placed_assets[i];
    if (!existing || !existing.IsMap()) {
      updated_placed_assets.push_back(existing);
      continue;
    }
    const std::string id = yaml_map_value_or_empty(existing, "id");
    if (!id.empty() && layout_ids.count(id) != 0) {
      updated_placed_assets.push_back(bootstrap_environment_asset_from_layout_item(existing, bootstrap_by_id[id]));
      emitted_ids.insert(id);
    } else {
      updated_placed_assets.push_back(existing);
    }
  }
  for (const auto & id : layout_id_order) {
    if (emitted_ids.count(id) != 0) continue;
    updated_placed_assets.push_back(bootstrap_environment_asset_from_layout_item(YAML::Node(), bootstrap_by_id[id]));
  }
  env_root["placed_assets"] = updated_placed_assets;
  result.placed_assets_written = layout_ids.size();

  if (!result.created && yaml_nodes_equal(before, env_root)) {
    result.ok = true;
    result.wrote = false;
    return result;
  }

  boost::system::error_code ec;
  fs::create_directories(scene_dir, ec);
  if (ec) {
    result.error = std::string("failed to create scene directory: ") + ec.message();
    return result;
  }
  std::ofstream out(env_layout_path.string());
  if (!out.good()) {
    result.error = "failed to open environment_layout.yaml for write";
    return result;
  }
  out << env_root << "\n";
  out.close();
  if (!out.good()) {
    result.error = "failed to write environment_layout.yaml";
    return result;
  }
  result.ok = true;
  result.wrote = true;
  return result;
}

static bool has_safe_starter_layout_metadata(const WorkcellStudioCanvasItem & preview_item)
{
  if (!preview_item.has_mesh_metadata) return false;
  if (preview_item.mesh_path.empty()) return false;
  if (preview_item.source_file.empty()) return false;
  if (!preview_item.mesh_load_warning.empty()) return false;
  if (!preview_item.alignment_warning.empty()) return false;
  if (!preview_item.warnings.empty()) return false;
  return true;
}

static std::string lowercase_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value;
}

static bool contains_any_visual_helper_token(const std::string & value)
{
  if (value.empty()) return false;
  const std::string lower = lowercase_copy(value);
  const std::array<const char *, 14> helper_tokens = {
    "reach", "roi", "fov", "warning", "blocker", "axis", "grid",
    "label", "overlay", "helper", "annotation", "gizmo", "marker", "safety"
  };
  for (const char * token : helper_tokens) {
    if (lower.find(token) != std::string::npos) return true;
  }
  return false;
}

static bool is_preview_item_visual_helper_only(const WorkcellStudioCanvasItem & preview_item)
{
  return contains_any_visual_helper_token(preview_item.id) ||
    contains_any_visual_helper_token(preview_item.type) ||
    contains_any_visual_helper_token(preview_item.role) ||
    contains_any_visual_helper_token(preview_item.category);
}



static std::string sanitize_preview_id_for_editable_layout_id(const std::string & preview_id)
{
  std::string sanitized;
  sanitized.reserve(preview_id.size());
  bool last_was_separator = false;
  for (const unsigned char c : preview_id) {
    if (std::isalnum(c)) {
      sanitized.push_back(static_cast<char>(std::tolower(c)));
      last_was_separator = false;
    } else if (!last_was_separator) {
      sanitized.push_back('_');
      last_was_separator = true;
    }
  }
  while (!sanitized.empty() && sanitized.front() == '_') sanitized.erase(sanitized.begin());
  while (!sanitized.empty() && sanitized.back() == '_') sanitized.pop_back();
  if (sanitized.empty()) sanitized = "preview_item";
  return sanitized;
}

[[maybe_unused]] static std::string editable_layout_copy_id_from_preview_id(const std::string & preview_id)
{
  return "editable_" + sanitize_preview_id_for_editable_layout_id(preview_id);
}

static YAML::Node new_workcell_studio_layout_root(const std::string & scene_name)
{
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "workcell_studio_layout/v1";
  root["scene_name"] = scene_name;
  root["items"] = YAML::Node(YAML::NodeType::Sequence);
  return root;
}

static bool read_sequence3(const YAML::Node & node, double * a, double * b, double * c)
{
  if (!node || !node.IsSequence() || node.size() < 3) return false;
  double aa = *a;
  double bb = *b;
  double cc = *c;
  if (!yaml_read_double(yaml_seq_index(node, 0), &aa)) return false;
  if (!yaml_read_double(yaml_seq_index(node, 1), &bb)) return false;
  if (!yaml_read_double(yaml_seq_index(node, 2), &cc)) return false;
  *a = aa;
  *b = bb;
  *c = cc;
  return true;
}

static YAML::Node normalized_pose_from_node(const YAML::Node & node)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double r = 0.0;
  double p = 0.0;
  double yy = 0.0;
  const YAML::Node pose = yaml_map_key(node, "pose");
  if (pose.IsMap()) {
    if (!read_sequence3(yaml_map_key(pose, "xyz"), &x, &y, &z)) {
      yaml_read_double(yaml_map_key(pose, "x"), &x);
      yaml_read_double(yaml_map_key(pose, "y"), &y);
      yaml_read_double(yaml_map_key(pose, "z"), &z);
    }
    if (!read_sequence3(yaml_map_key(pose, "rpy"), &r, &p, &yy)) {
      yaml_read_double(yaml_map_key(pose, "roll"), &r);
      yaml_read_double(yaml_map_key(pose, "pitch"), &p);
      yaml_read_double(yaml_map_key(pose, "yaw"), &yy);
    }
  } else if (pose.IsSequence() && pose.size() >= 6) {
    yaml_read_double(yaml_seq_index(pose, 0), &x);
    yaml_read_double(yaml_seq_index(pose, 1), &y);
    yaml_read_double(yaml_seq_index(pose, 2), &z);
    yaml_read_double(yaml_seq_index(pose, 3), &r);
    yaml_read_double(yaml_seq_index(pose, 4), &p);
    yaml_read_double(yaml_seq_index(pose, 5), &yy);
  } else {
    read_sequence3(yaml_map_key(node, "xyz"), &x, &y, &z);
    read_sequence3(yaml_map_key(node, "rpy"), &r, &p, &yy);
  }
  YAML::Node out(YAML::NodeType::Map);
  out["xyz"].push_back(x);
  out["xyz"].push_back(y);
  out["xyz"].push_back(z);
  out["rpy"].push_back(r);
  out["rpy"].push_back(p);
  out["rpy"].push_back(yy);
  return out;
}

static YAML::Node normalized_dimensions_from_node(const YAML::Node & node)
{
  double width = 0.25;
  double depth = 0.25;
  double height = 0.25;
  const YAML::Node dimensions = yaml_map_key(node, "dimensions");
  const YAML::Node size = yaml_map_key(node, "size");
  if (!read_sequence3(dimensions, &width, &depth, &height)) {
    if (!read_sequence3(size, &width, &depth, &height) && size.IsMap()) {
      yaml_read_double(yaml_map_key(size, "width"), &width);
      yaml_read_double(yaml_map_key(size, "depth"), &depth);
      yaml_read_double(yaml_map_key(size, "height"), &height);
    }
  }
  YAML::Node out(YAML::NodeType::Sequence);
  out.push_back(width);
  out.push_back(depth);
  out.push_back(height);
  return out;
}

static std::string first_string_field(const YAML::Node & node, const std::vector<const char *> & keys)
{
  for (const char * key : keys) {
    const std::string value = yaml_map_value_or_empty(node, key);
    if (!value.empty()) return value;
  }
  return "";
}

static bool node_bool_field_is_true_or_absent(const YAML::Node & node, const char * key)
{
  const YAML::Node value_node = yaml_map_key(node, key);
  if (!value_node.IsDefined()) return true;
  bool value = false;
  return yaml_read_bool(value_node, &value) && value;
}

static bool node_bool_field_is_false_or_absent(const YAML::Node & node, const char * key)
{
  const YAML::Node value_node = yaml_map_key(node, key);
  if (!value_node.IsDefined()) return true;
  bool value = false;
  return yaml_read_bool(value_node, &value) && !value;
}

static bool should_skip_source_node_as_locked(const YAML::Node & node)
{
  bool locked = false;
  if (yaml_read_bool(yaml_map_key(node, "locked"), &locked) && locked) return true;
  return !node_bool_field_is_false_or_absent(node, "generated_locked");
}

static YAML::Node normalize_editable_source_node(const YAML::Node & node, const std::string & id_hint, const std::string & type_hint, const std::string & source_used)
{
  const std::string id = !id_hint.empty() ? id_hint : first_string_field(node, {"id", "name", "camera_id"});
  if (id.empty()) return YAML::Node();
  std::string type = !type_hint.empty() ? type_hint : first_string_field(node, {"type", "category", "class", "role"});
  if (type.empty()) type = "object";

  YAML::Node item(YAML::NodeType::Map);
  item["id"] = id;
  item["type"] = type;
  item["category"] = type;
  const std::string role = first_string_field(node, {"role", "class", "type"});
  if (!role.empty()) item["role"] = role;
  const std::string label = first_string_field(node, {"display_name", "label", "name"});
  if (!label.empty()) item["display_name"] = label;
  item["pose"] = normalized_pose_from_node(node);
  item["dimensions"] = normalized_dimensions_from_node(node);

  const YAML::Node mesh = yaml_map_key(node, "mesh");
  if (mesh && !mesh.IsNull()) item["mesh"] = YAML::Clone(mesh);
  else {
    const std::string mesh_path = first_string_field(node, {"mesh_path", "visual_mesh", "collision_mesh", "source_path"});
    if (!mesh_path.empty()) {
      YAML::Node out_mesh(YAML::NodeType::Map);
      out_mesh["path"] = mesh_path;
      item["mesh"] = out_mesh;
    }
  }
  item["source"] = source_used;
  item["editable"] = true;
  item["locked"] = false;
  return item;
}

static bool append_unique_item(YAML::Node * items, const YAML::Node & item, std::set<std::string> * ids)
{
  if (!item || !item.IsMap()) return false;
  const std::string id = yaml_map_value_or_empty(item, "id");
  if (id.empty() || ids->count(id) != 0) return false;
  ids->insert(id);
  items->push_back(item);
  return true;
}

static void append_source_sequence_items(const YAML::Node & seq, const std::string & type_hint, const std::string & source_used,
                                         YAML::Node * items, std::set<std::string> * ids,
                                         std::size_t * skipped_locked_items)
{
  if (!seq || !seq.IsSequence()) return;
  for (const auto & node : seq) {
    if (!node || !node.IsMap()) continue;
    if (should_skip_source_node_as_locked(node) || !node_bool_field_is_true_or_absent(node, "editable") || !node_bool_field_is_true_or_absent(node, "placeable")) {
      ++(*skipped_locked_items);
      continue;
    }
    append_unique_item(items, normalize_editable_source_node(node, "", type_hint, source_used), ids);
  }
}

static void append_environment_object_map_items(const YAML::Node & map, const std::string & source_used,
                                                YAML::Node * items, std::set<std::string> * ids,
                                                std::size_t * skipped_locked_items)
{
  if (!map || !map.IsMap()) return;
  for (const auto & entry : map) {
    const std::string id = entry.first.as<std::string>("");
    const YAML::Node node = entry.second;
    if (!node || !node.IsMap()) continue;
    if (should_skip_source_node_as_locked(node)) {
      ++(*skipped_locked_items);
      continue;
    }
    YAML::Node item = normalize_editable_source_node(node, id, "object", source_used);
    std::vector<std::string> mesh_candidates;
    add_mesh_candidate(yaml_map_key(yaml_map_key(yaml_map_key(node, "visual"), "geometry"), "filepath"), &mesh_candidates);
    const YAML::Node links = yaml_map_key(node, "links");
    if (links.IsMap()) {
      for (const auto & link : links) {
        add_mesh_candidate(yaml_map_key(yaml_map_key(yaml_map_key(link.second, "visual"), "geometry"), "filepath"), &mesh_candidates);
      }
    }
    if (!mesh_candidates.empty()) {
      YAML::Node mesh(YAML::NodeType::Map);
      mesh["path"] = mesh_candidates.front();
      item["mesh"] = mesh;
    }
    append_unique_item(items, item, ids);
  }
}

static std::string preview_source_layer_for_provenance(const WorkcellStudioCanvasItem & preview_item)
{
  switch (preview_item.provenance) {
    case WorkcellStudioItemProvenance::EditableLayout:
      return "editable_layout";
    case WorkcellStudioItemProvenance::StaticFallbackPreview:
      return "static_fallback_preview";
    case WorkcellStudioItemProvenance::GeneratedOrLegacyPreview:
    default:
      return "locked_generated_urdf_visual";
  }
}

static std::string editable_copy_id_for_preview_item(const WorkcellStudioCanvasItem & preview_item)
{
  if (preview_item.id.empty()) return "editable_preview_copy";
  return preview_item.id + "__editable_copy";
}

static YAML::Node preview_item_to_layout_item(const WorkcellStudioCanvasItem & preview_item)
{
  YAML::Node item(YAML::NodeType::Map);
  item["id"] = editable_copy_id_for_preview_item(preview_item);
  item["type"] = preview_item.type;
  if (!preview_item.category.empty()) item["category"] = preview_item.category;
  else item["category"] = preview_item.type;
  if (!preview_item.role.empty()) item["role"] = preview_item.role;
  if (!preview_item.label.empty()) item["display_name"] = preview_item.label;

  YAML::Node pose(YAML::NodeType::Map);
  pose["xyz"].push_back(preview_item.x);
  pose["xyz"].push_back(preview_item.y);
  pose["xyz"].push_back(preview_item.z);
  pose["rpy"].push_back(preview_item.roll);
  pose["rpy"].push_back(preview_item.pitch);
  pose["rpy"].push_back(preview_item.yaw);
  item["pose"] = pose;

  item["dimensions"].push_back(preview_item.width);
  item["dimensions"].push_back(preview_item.depth);
  item["dimensions"].push_back(preview_item.height);
  if (!preview_item.primitive_geometry_type.empty()) item["primitive_geometry_type"] = preview_item.primitive_geometry_type;
  if (preview_item.primitive_radius > 0.0) item["primitive_radius"] = preview_item.primitive_radius;
  if (preview_item.primitive_length > 0.0) item["primitive_length"] = preview_item.primitive_length;

  YAML::Node mesh(YAML::NodeType::Map);
  if (!preview_item.mesh_path.empty()) mesh["path"] = preview_item.mesh_path;
  if (!preview_item.source_file.empty()) mesh["source"] = preview_item.source_file;
  if (!preview_item.mesh_source_package.empty()) mesh["source_package"] = preview_item.mesh_source_package;
  if (preview_item.mesh_scale_x != 1.0 || preview_item.mesh_scale_y != 1.0 || preview_item.mesh_scale_z != 1.0) {
    mesh["scale"].push_back(preview_item.mesh_scale_x);
    mesh["scale"].push_back(preview_item.mesh_scale_y);
    mesh["scale"].push_back(preview_item.mesh_scale_z);
  }
  if (mesh.size() > 0) item["mesh"] = mesh;

  if (!preview_item.source_package.empty()) item["source_package"] = preview_item.source_package;
  if (!preview_item.mesh_source_package.empty()) item["mesh_source_package"] = preview_item.mesh_source_package;
  item["source"] = "preview_model";
  item["preview_source_id"] = preview_item.id;
  item["source_layer"] = "editable_layout";
  item["editable"] = true;
  item["locked"] = false;

  YAML::Node provenance(YAML::NodeType::Map);
  const std::string source_layer = preview_source_layer_for_provenance(preview_item);
  provenance["mode"] = "created_from_generated_preview";
  provenance["source_layer"] = source_layer;
  provenance["preview_source_id"] = preview_item.id;
  provenance["copy_kind"] = "editable_layout_copy";
  provenance["editable_layout_copy"] = true;
  provenance["original_source_id"] = preview_item.id;
  provenance["original_source_layer"] = source_layer;
  if (!preview_item.source_file.empty()) provenance["original_source_file"] = preview_item.source_file;
  provenance["copied_from"] = "preview_model";
  item["provenance"] = provenance;
  return item;
}

WorkcellStudioStarterLayoutSummary build_starter_layout_entries_from_preview(const WorkcellStudioCanvasModel & model)
{
  WorkcellStudioStarterLayoutSummary summary;
  summary.total_preview_items = model.items.size();

  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "workcell_studio_layout/v1";
  root["scene_name"] = model.scene_name;
  YAML::Node items(YAML::NodeType::Sequence);
  std::set<std::string> ids;
  for (const auto & preview_item : model.items) {
    if (preview_item.provenance == WorkcellStudioItemProvenance::StaticFallbackPreview) {
      ++summary.skipped_static_fallback_items;
      continue;
    }
    if (is_preview_item_visual_helper_only(preview_item) || !has_safe_starter_layout_metadata(preview_item)) {
      ++summary.skipped_unsafe_or_missing_metadata_items;
      continue;
    }
    if (append_unique_item(&items, preview_item_to_layout_item(preview_item), &ids)) {
      ++summary.editable_items_created;
    }
  }
  root["empty_layout_marker"] = summary.editable_items_created == 0;
  root["items"] = items;
  summary.layout = root;
  return summary;
}

WorkcellStudioEditableLayoutBootstrapResult bootstrap_editable_layout_from_scene_sources(
  const fs::path & scene_dir,
  const std::string & scene_name,
  const WorkcellStudioCanvasModel & preview_model)
{
  WorkcellStudioEditableLayoutBootstrapResult result;
  result.expected_output_dir = scene_dir / "layout";
  result.expected_output_file = result.expected_output_dir / "workcell_studio_layout.yaml";
  result.expected_output_dir_exists = fs::exists(result.expected_output_dir);
  result.expected_output_file_exists = fs::exists(result.expected_output_file);
  result.layout = new_workcell_studio_layout_root(scene_name);

  const auto finish_if_items = [&result](const std::string & source_used, const YAML::Node & candidate) {
    const YAML::Node items = yaml_map_key(candidate, "items");
    if (!items || !items.IsSequence() || items.size() == 0) return false;
    result.source_used = source_used;
    result.editable_items_created = items.size();
    result.layout = candidate;
    result.layout["empty_layout_marker"] = false;
    return true;
  };

  // 1. Preserve existing editable items from layout/workcell_studio_layout.yaml.
  YAML::Node existing_layout;
  const YamlLoadStatus existing_status = read_yaml(result.expected_output_file, &existing_layout);
  if (existing_status.loaded) {
    const std::string schema_version = yaml_map_value_or_empty(existing_layout, "schema_version");
    const YAML::Node existing_items = yaml_map_key(existing_layout, "items");
    if ((schema_version == "workcell_studio_layout/v1" || (schema_version.empty() && existing_items.IsSequence())) && existing_items.IsSequence()) {
      YAML::Node candidate = new_workcell_studio_layout_root(scene_name);
      YAML::Node candidate_items = yaml_map_key(candidate, "items");
      std::set<std::string> ids;
      for (const auto & node : existing_items) {
        if (!node || !node.IsMap()) continue;
        if (layout_item_is_effectively_editable(node)) {
          YAML::Node item = YAML::Clone(node);
          item["editable"] = true;
          item["locked"] = false;
          if (!yaml_map_key(item, "source")) item["source"] = "layout/workcell_studio_layout.yaml";
          append_unique_item(&candidate_items, item, &ids);
        } else {
          bool locked = false;
          if (yaml_read_bool(yaml_map_key(node, "locked"), &locked) && locked) ++result.skipped_locked_items;
          else if (layout_item_is_fallback_preview_entry(node)) ++result.skipped_static_fallback_items;
        }
      }
      candidate["items"] = candidate_items;
      if (finish_if_items("layout/workcell_studio_layout.yaml", candidate)) return result;
    }
  }

  // 2. Use editable/placeable items from environment_layout.yaml.
  const fs::path environment_layout_path = scene_dir / "environment_layout.yaml";
  YAML::Node environment_layout;
  const YamlLoadStatus environment_layout_status = read_yaml(environment_layout_path, &environment_layout);
  if (!environment_layout_status.exists) result.blockers.push_back("no environment_layout.yaml");
  if (environment_layout_status.loaded) {
    YAML::Node candidate = new_workcell_studio_layout_root(scene_name);
    YAML::Node candidate_items = yaml_map_key(candidate, "items");
    std::set<std::string> ids;
    append_source_sequence_items(yaml_map_key(environment_layout, "items"), "object", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment_layout, "assets"), "object", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment_layout, "placed_assets"), "object", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment_layout, "objects"), "object", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment_layout, "zones"), "zone", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment_layout, "targets"), "place_target", "environment_layout.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    const YAML::Node camera = yaml_map_key(environment_layout, "camera");
    if (camera && camera.IsMap() && node_bool_field_is_true_or_absent(camera, "editable")) {
      append_unique_item(&candidate_items, normalize_editable_source_node(camera, first_string_field(camera, {"id", "camera_id", "name"}), "camera", "environment_layout.yaml"), &ids);
    }
    candidate["items"] = candidate_items;
    if (finish_if_items("environment_layout.yaml", candidate)) return result;
  }

  // 3. Use placed objects, cameras, and zones from environment.yaml.
  const fs::path environment_path = scene_dir / "environment.yaml";
  YAML::Node environment;
  const YamlLoadStatus environment_status = read_yaml(environment_path, &environment);
  if (!environment_status.exists) result.blockers.push_back("no environment.yaml");
  if (environment_status.loaded) {
    YAML::Node candidate = new_workcell_studio_layout_root(scene_name);
    YAML::Node candidate_items = yaml_map_key(candidate, "items");
    std::set<std::string> ids;
    append_source_sequence_items(yaml_map_key(environment, "placed_objects"), "object", "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_environment_object_map_items(yaml_map_key(environment, "objects"), "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment, "cameras"), "camera", "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    const YAML::Node camera = yaml_map_key(environment, "camera");
    if (camera && camera.IsMap()) {
      bool enabled = true;
      yaml_read_bool(yaml_map_key(camera, "enabled"), &enabled);
      if (enabled) append_unique_item(&candidate_items, normalize_editable_source_node(camera, first_string_field(camera, {"id", "camera_id", "name"}), "camera", "environment.yaml"), &ids);
    }
    append_source_sequence_items(yaml_map_key(environment, "zones"), "zone", "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(environment, "task_zones"), "zone", "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(yaml_map_key(environment, "workspace"), "zones"), "zone", "environment.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    candidate["items"] = candidate_items;
    if (finish_if_items("environment.yaml", candidate)) return result;
  }

  // 4. Use scene assets and zones from cell_definition.yaml.
  const fs::path cell_definition_path = scene_dir / "cell_definition.yaml";
  YAML::Node cell_definition;
  const YamlLoadStatus cell_definition_status = read_yaml(cell_definition_path, &cell_definition);
  if (!cell_definition_status.exists) result.blockers.push_back("no cell_definition.yaml");
  if (cell_definition_status.loaded) {
    YAML::Node candidate = new_workcell_studio_layout_root(scene_name);
    YAML::Node candidate_items = yaml_map_key(candidate, "items");
    std::set<std::string> ids;
    append_source_sequence_items(yaml_map_key(cell_definition, "assets"), "object", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(yaml_map_key(cell_definition, "scene"), "assets"), "object", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(yaml_map_key(cell_definition, "environment"), "assets"), "object", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(cell_definition, "zones"), "zone", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(yaml_map_key(cell_definition, "environment"), "zones"), "zone", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    append_source_sequence_items(yaml_map_key(yaml_map_key(cell_definition, "task"), "destinations"), "place_target", "cell_definition.yaml", &candidate_items, &ids, &result.skipped_locked_items);
    candidate["items"] = candidate_items;
    if (finish_if_items("cell_definition.yaml", candidate)) return result;
  }

  // 5. Use only safe preview metadata from build_workcell_studio_canvas_model.
  YAML::Node candidate = new_workcell_studio_layout_root(scene_name);
  YAML::Node candidate_items = yaml_map_key(candidate, "items");
  std::set<std::string> ids;
  std::size_t preview_static_fallback = 0;
  std::size_t preview_unsafe = 0;
  for (const auto & preview_item : preview_model.items) {
    if (preview_item.provenance == WorkcellStudioItemProvenance::StaticFallbackPreview) {
      ++preview_static_fallback;
      ++result.skipped_static_fallback_items;
      continue;
    }
    if (is_preview_item_visual_helper_only(preview_item) || !has_safe_starter_layout_metadata(preview_item)) {
      ++preview_unsafe;
      ++result.skipped_unsafe_or_missing_metadata_items;
      continue;
    }
    append_unique_item(&candidate_items, preview_item_to_layout_item(preview_item), &ids);
  }
  candidate["items"] = candidate_items;
  if (finish_if_items("preview_model", candidate)) return result;

  if (!preview_model.items.empty() && preview_static_fallback > 0 && preview_static_fallback == preview_model.items.size()) {
    result.blockers.push_back("preview fallback-only");
  }
  if (preview_unsafe > 0) result.blockers.push_back("unsafe/helper/missing mesh metadata");

  result.layout["empty_layout_marker"] = true;
  result.source_used.clear();
  result.editable_items_created = 0;
  return result;
}

}  // namespace workcell_builder
