#pragma once

#include <string>
#include <vector>

namespace workcell_builder
{
enum class PrimitiveType { kBox, kTable, kBinTray, kConveyorPlaceholder, kFixturePlate };

struct PrimitiveSpec
{
  std::string object_name;
  PrimitiveType type = PrimitiveType::kBox;
  double length_m = 0.1;
  double width_m = 0.1;
  double height_m = 0.1;
  double wall_thickness_m = 0.01;
  double table_top_thickness_m = 0.02;
  double table_leg_size_m = 0.04;
};

std::string sanitize_object_name(const std::string & input);
bool validate_primitive_spec(const PrimitiveSpec & spec, std::string * reason);
bool write_ascii_stl(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
bool write_box_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
bool write_table_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
bool write_bin_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
bool write_fixture_plate_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
bool write_conveyor_placeholder_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error);
}
