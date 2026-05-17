#pragma once

#include <QDialog>
#include <QTableWidget>
#include <string>
#include <vector>

#include "object_placement_model.hpp"
#include "object_placement_yaml_io.hpp"

namespace workcell_builder
{

class ObjectPlacementDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ObjectPlacementDialog(QWidget * parent = nullptr);
  void set_objects(const std::vector<PlacedObject> & objects);
  void set_scene_context(const std::string & scene_name, const std::string & environment_yaml_path);
  void set_active_scene_name(const std::string & scene_name);
  void set_active_environment_yaml_path(const std::string & environment_yaml_path);
  std::vector<PlacedObject> objects() const;

private:
  void rebuild_table();
  void add_default_object(const std::string & source_type);
  void import_rviz_pose_feedback();
  void apply_and_generate_preview();

  std::string resolve_scene_name(bool * used_fallback = nullptr) const;

  ObjectPlacementModel model_;
  QTableWidget * table_{nullptr};
  QTableWidget * task_zone_table_{nullptr};
  std::vector<TaskZone> task_zones_;
  std::string active_scene_name_;
  std::string active_environment_yaml_path_;
};

}  // namespace workcell_builder
