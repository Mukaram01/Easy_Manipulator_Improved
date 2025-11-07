/**
 * @file ply_io.h
 * @brief Simple helper functions for reading and writing ASCII PLY triangle meshes
 *
 * @author OpenAI
 * @date 2024
 */
#ifndef TESSERACT_COMMON_PLY_IO_H
#define TESSERACT_COMMON_PLY_IO_H

#include <string>

#include <tesseract_common/eigen_types.h>

namespace tesseract_common
{
/**
 * @brief Load an ASCII PLY mesh file containing vertex and face data
 * @param path Path to the PLY file to load
 * @param vertices Output list of mesh vertices
 * @param faces Output flattened index array (triplets describe a face)
 * @param triangulate If true, faces with more than three vertices are triangulated with a fan
 * @return The number of triangle faces parsed on success, otherwise -1
 */
int loadSimplePlyFile(const std::string& path,
                      VectorVector3d& vertices,
                      Eigen::VectorXi& faces,
                      bool triangulate = false);

/**
 * @brief Write a mesh to an ASCII PLY file
 * @param path Path to the destination file
 * @param vertices Vertices of the mesh
 * @param faces Flattened face indices (triplets per face)
 * @param face_count Number of faces stored in the index buffer
 * @return True if the file was successfully written, otherwise false
 */
bool writeSimplePlyFile(const std::string& path,
                        const VectorVector3d& vertices,
                        const Eigen::VectorXi& faces,
                        int face_count);

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PLY_IO_H
