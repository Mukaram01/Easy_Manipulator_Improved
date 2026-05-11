#pragma once

#include <QDialog>
#include <QTableWidget>
#include <vector>

#include "object_placement_model.hpp"

namespace workcell_builder
{

class ObjectPlacementDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ObjectPlacementDialog(QWidget * parent = nullptr);
  void set_objects(const std::vector<PlacedObject> & objects);
  std::vector<PlacedObject> objects() const;

private:
  void rebuild_table();
  void add_default_object(const std::string & source_type);

  ObjectPlacementModel model_;
  QTableWidget * table_{nullptr};
};

}  // namespace workcell_builder
