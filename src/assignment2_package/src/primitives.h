#pragma once

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>

namespace collision_primitives {

/**
 * @brief Create a triangular prism mesh (base b, height h, length L)
 * @param b Base width of the triangular face (meters)
 * @param h Height of the triangular face (meters)
 * @param L Length (depth) of the prism (meters)
 * @return shape_msgs::Mesh representing the prism
 */
inline shape_msgs::Mesh makeTriangularPrism(double b, double h, double L)
{
  shape_msgs::Mesh mesh;
  mesh.vertices.resize(6);
  // Bottom triangle (z = 0)
  mesh.vertices[0].x = -b/2; mesh.vertices[0].y =  0.0; mesh.vertices[0].z = 0.0;
  mesh.vertices[1].x =  b/2; mesh.vertices[1].y =  0.0; mesh.vertices[1].z = 0.0;
  mesh.vertices[2].x =  0.0; mesh.vertices[2].y =    h; mesh.vertices[2].z = 0.0;
  // Top triangle (z = L)
  for (size_t i = 0; i < 3; ++i) {
    mesh.vertices[i+3] = mesh.vertices[i];
    mesh.vertices[i+3].z = L;
  }

  auto addTri = [&](int a, int b, int c) {
    shape_msgs::MeshTriangle tri;
    tri.vertex_indices[0] = a;
    tri.vertex_indices[1] = b;
    tri.vertex_indices[2] = c;
    mesh.triangles.push_back(tri);
  };

  // End caps
  addTri(0, 1, 2);
  addTri(3, 5, 4);
  // Side faces
  addTri(0, 1, 4);
  addTri(0, 4, 3);

  addTri(1, 2, 5);
  addTri(1, 5, 4);

  addTri(2, 0, 3);
  addTri(2, 3, 5);

  return mesh;
}

} // namespace collision_primitives
