#include "generated_stl_writer.hpp"
#include "object_placement_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <tuple>
#include <vector>

namespace workcell_builder
{
namespace
{
using Tri = std::tuple<double, double, double, double, double, double, double, double, double>;

void append_box(std::vector<Tri> * out, double cx, double cy, double cz, double lx, double ly, double lz)
{
  const double x0 = cx - lx / 2.0, x1 = cx + lx / 2.0;
  const double y0 = cy - ly / 2.0, y1 = cy + ly / 2.0;
  const double z0 = cz - lz / 2.0, z1 = cz + lz / 2.0;
  const double v[8][3] = {{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},{x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
  const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
  for (const auto & t : f) out->emplace_back(v[t[0]][0],v[t[0]][1],v[t[0]][2],v[t[1]][0],v[t[1]][1],v[t[1]][2],v[t[2]][0],v[t[2]][1],v[t[2]][2]);
}

bool write_mesh(const PrimitiveSpec & spec, const std::string & out_path, const std::vector<Tri> & tris, std::string * error)
{
  std::ofstream out(out_path);
  if (!out) { if (error) *error = "Failed to open STL path"; return false; }
  out << "solid " << sanitize_object_name(spec.object_name) << "\n";
  for (const auto & t : tris) {
    out << "  facet normal 0 0 0\n    outer loop\n";
    out << "      vertex " << std::get<0>(t) << " " << std::get<1>(t) << " " << std::get<2>(t) << "\n";
    out << "      vertex " << std::get<3>(t) << " " << std::get<4>(t) << " " << std::get<5>(t) << "\n";
    out << "      vertex " << std::get<6>(t) << " " << std::get<7>(t) << " " << std::get<8>(t) << "\n";
    out << "    endloop\n  endfacet\n";
  }
  out << "endsolid " << sanitize_object_name(spec.object_name) << "\n";
  return true;
}
}
bool validate_primitive_spec(const PrimitiveSpec & spec, std::string * reason)
{
  if (sanitize_object_name(spec.object_name).empty()) { if (reason) *reason = "invalid object name"; return false; }
  for (double v : {spec.length_m, spec.width_m, spec.height_m}) {
    if (!(v > 0.0) || std::isnan(v)) { if (reason) *reason = "dimensions must be positive finite values"; return false; }
  }
  return true;
}

bool write_box_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{ std::vector<Tri> t; append_box(&t, 0, 0, spec.height_m / 2.0, spec.length_m, spec.width_m, spec.height_m); return write_mesh(spec, out_path, t, error); }
bool write_fixture_plate_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{ return write_box_mesh(spec, out_path, error); }
bool write_conveyor_placeholder_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{ std::vector<Tri> t; append_box(&t,0,0,spec.height_m/2.0,spec.length_m,spec.width_m,spec.height_m); append_box(&t,-spec.length_m*0.4,0,spec.height_m*0.9,spec.length_m*0.1,spec.width_m*0.9,spec.height_m*0.8); append_box(&t,spec.length_m*0.4,0,spec.height_m*0.9,spec.length_m*0.1,spec.width_m*0.9,spec.height_m*0.8); return write_mesh(spec,out_path,t,error); }
bool write_table_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{ std::vector<Tri> t; double top=std::min(spec.table_top_thickness_m,spec.height_m*0.5); append_box(&t,0,0,spec.height_m-top/2.0,spec.length_m,spec.width_m,top); double leg_h=std::max(0.01,spec.height_m-top); double s=std::max(0.01,spec.table_leg_size_m); for(double sx:{-1,1}) for(double sy:{-1,1}) append_box(&t,sx*(spec.length_m/2-s/2),sy*(spec.width_m/2-s/2),leg_h/2,s,s,leg_h); return write_mesh(spec,out_path,t,error); }
bool write_bin_mesh(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{ std::vector<Tri> t; double w=std::max(0.001,spec.wall_thickness_m); append_box(&t,0,0,w/2,spec.length_m,spec.width_m,w); append_box(&t,0,spec.width_m/2-w/2,spec.height_m/2,w?spec.length_m:spec.length_m,w,spec.height_m); append_box(&t,0,-spec.width_m/2+w/2,spec.height_m/2,spec.length_m,w,spec.height_m); append_box(&t,spec.length_m/2-w/2,0,spec.height_m/2,w,spec.width_m-2*w,spec.height_m); append_box(&t,-spec.length_m/2+w/2,0,spec.height_m/2,w,spec.width_m-2*w,spec.height_m); return write_mesh(spec,out_path,t,error); }

bool write_ascii_stl(const PrimitiveSpec & spec, const std::string & out_path, std::string * error)
{
  std::string reason; if (!validate_primitive_spec(spec, &reason)) { if (error) *error = reason; return false; }
  switch (spec.type) {
    case PrimitiveType::kBox: return write_box_mesh(spec, out_path, error);
    case PrimitiveType::kTable: return write_table_mesh(spec, out_path, error);
    case PrimitiveType::kBinTray: return write_bin_mesh(spec, out_path, error);
    case PrimitiveType::kConveyorPlaceholder: return write_conveyor_placeholder_mesh(spec, out_path, error);
    case PrimitiveType::kFixturePlate: return write_fixture_plate_mesh(spec, out_path, error);
  }
  if (error) {
    *error = "unsupported primitive";
  }
  return false;
}
}
