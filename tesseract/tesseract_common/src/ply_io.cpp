/**
 * @file ply_io.cpp
 * @brief Implementation of helpers for reading and writing simple ASCII PLY meshes
 */

#include <tesseract_common/ply_io.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <sstream>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
namespace
{
bool readNextLine(std::istream& in, std::string& line)
{
  while (std::getline(in, line))
  {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (!line.empty())
      return true;
  }
  return false;
}
}  // namespace

int loadSimplePlyFile(const std::string& path,
                      VectorVector3d& vertices,
                      Eigen::VectorXi& faces,
                      bool triangulate)
{
  vertices.clear();
  faces.resize(0);

  std::ifstream file(path);
  if (!file.is_open())
    return -1;

  std::string line;
  std::string format;
  std::size_t vertex_count = 0;
  std::size_t face_count = 0;
  bool header_complete = false;

  while (std::getline(file, line))
  {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    if (line.empty())
      continue;

    std::istringstream ss(line);
    std::string keyword;
    ss >> keyword;
    if (keyword == "comment" || keyword == "obj_info")
    {
      continue;
    }
    else if (keyword == "format")
    {
      ss >> format;
      if (format != "ascii")
        return -1;
    }
    else if (keyword == "element")
    {
      std::string element_name;
      ss >> element_name;
      if (element_name == "vertex")
      {
        ss >> vertex_count;
      }
      else if (element_name == "face")
      {
        ss >> face_count;
      }
    }
    else if (keyword == "end_header")
    {
      header_complete = true;
      break;
    }
  }

  if (!header_complete)
    return -1;

  vertices.reserve(vertex_count);
  for (std::size_t i = 0; i < vertex_count; ++i)
  {
    if (!readNextLine(file, line))
      return -1;

    std::istringstream ss(line);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(ss >> x >> y >> z))
      return -1;

    vertices.emplace_back(x, y, z);
  }

  std::vector<int> indices;
  indices.reserve(face_count * 3);
  int triangle_count = 0;

  for (std::size_t i = 0; i < face_count; ++i)
  {
    if (!readNextLine(file, line))
      return -1;

    std::istringstream ss(line);
    int vertices_per_face = 0;
    if (!(ss >> vertices_per_face))
      return -1;

    if (vertices_per_face < 3)
      return -1;

    std::vector<int> local_indices(static_cast<std::size_t>(vertices_per_face));
    for (int& idx : local_indices)
    {
      if (!(ss >> idx))
        return -1;
      if (idx < 0 || static_cast<std::size_t>(idx) >= vertices.size())
        return -1;
    }

    if (vertices_per_face == 3)
    {
      indices.insert(indices.end(), local_indices.begin(), local_indices.end());
      ++triangle_count;
    }
    else
    {
      if (!triangulate)
        return -1;

      for (int j = 1; j < vertices_per_face - 1; ++j)
      {
        indices.push_back(local_indices[0]);
        indices.push_back(local_indices[j]);
        indices.push_back(local_indices[j + 1]);
        ++triangle_count;
      }
    }
  }

  faces.resize(static_cast<Eigen::Index>(indices.size()));
  for (Eigen::Index i = 0; i < faces.size(); ++i)
    faces[i] = indices[static_cast<std::size_t>(i)];

  return triangle_count;
}

bool writeSimplePlyFile(const std::string& path,
                        const VectorVector3d& vertices,
                        const Eigen::VectorXi& faces,
                        int face_count)
{
  if (face_count < 0)
    return false;

  const std::size_t required_size = static_cast<std::size_t>(face_count) * 3;
  if (faces.size() < static_cast<Eigen::Index>(required_size))
    return false;

  std::ofstream file(path);
  if (!file.is_open())
    return false;

  file << "ply\n";
  file << "format ascii 1.0\n";
  file << "element vertex " << vertices.size() << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";
  file << "element face " << face_count << "\n";
  file << "property list uchar int vertex_indices\n";
  file << "end_header\n";

  for (const auto& vertex : vertices)
    file << vertex.x() << ' ' << vertex.y() << ' ' << vertex.z() << '\n';

  for (int i = 0; i < face_count; ++i)
  {
    const int base = i * 3;
    file << "3 " << faces[base] << ' ' << faces[base + 1] << ' ' << faces[base + 2] << '\n';
  }

  file.flush();
  return file.good();
}

}  // namespace tesseract_common
