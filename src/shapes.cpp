/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "geometric_shapes/shapes.h"
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <octomap/octomap.h>
#include <console_bridge/console.h>

const std::string shapes::Sphere::STRING_NAME = "sphere";
const std::string shapes::Box::STRING_NAME = "box";
const std::string shapes::Cylinder::STRING_NAME = "cylinder";
const std::string shapes::Cone::STRING_NAME = "cone";
const std::string shapes::Mesh::STRING_NAME = "mesh";
const std::string shapes::Plane::STRING_NAME = "plane";
const std::string shapes::OcTree::STRING_NAME = "octree";

shapes::Shape::Shape()
{
  type = UNKNOWN_SHAPE;
}

shapes::Shape::~Shape()
{
}

shapes::Sphere::Sphere() : Shape()
{
  type   = SPHERE;
  radius = 0.0;
}

shapes::Sphere::Sphere(double r) : Shape()
{
  type   = SPHERE;
  radius = r;
}

shapes::Cylinder::Cylinder() : Shape()
{
  type   = CYLINDER;
  length = radius = 0.0;
}

shapes::Cylinder::Cylinder(double r, double l) : Shape()
{
  type   = CYLINDER;
  length = l;
  radius = r;
}

shapes::Cone::Cone() : Shape()
{
  type   = CONE;
  length = radius = 0.0;
}

shapes::Cone::Cone(double r, double l) : Shape()
{
  type   = CONE;
  length = l;
  radius = r;
}

shapes::Box::Box() : Shape()
{
  type = BOX;
  size[0] = size[1] = size[2] = 0.0;
}

shapes::Box::Box(double x, double y, double z) : Shape()
{
  type = BOX;
  size[0] = x;
  size[1] = y;
  size[2] = z;
}

shapes::Mesh::Mesh() : Shape()
{
  type = MESH;
  vertex_count = 0;
  vertices = NULL;
  triangle_count = 0;
  triangles = NULL;
  triangle_normals = NULL;
  vertex_normals = NULL;
}

shapes::Mesh::Mesh(unsigned int v_count, unsigned int t_count) : Shape()
{
  type = MESH;
  vertex_count = v_count;
  vertices = new double[v_count * 3];
  triangle_count = t_count;
  triangles = new unsigned int[t_count * 3];
  triangle_normals = new double[t_count * 3];
  vertex_normals = new double[v_count * 3];
}

shapes::Mesh::~Mesh()
{
  if (vertices)
    delete[] vertices;
  if (triangles)
    delete[] triangles;
  if (triangle_normals)
    delete[] triangle_normals;
  if (vertex_normals)
    delete[] vertex_normals;
}

shapes::Plane::Plane() : Shape()
{
  type = PLANE;
  a = b = c = d = 0.0;
}

shapes::Plane::Plane(double pa, double pb, double pc, double pd) : Shape()
{
  type = PLANE;
  a = pa; b = pb; c = pc; d = pd;
}

shapes::OcTree::OcTree() : Shape()
{
  type = OCTREE;
}

shapes::OcTree::OcTree(const boost::shared_ptr<const octomap::OcTree> &t) : octree(t)
{
  type = OCTREE;
}

shapes::Shape* shapes::Sphere::clone() const
{
  return new Sphere(radius);
}

shapes::Shape* shapes::Cylinder::clone() const
{
  return new Cylinder(radius, length);
}

shapes::Shape* shapes::Cone::clone() const
{
  return new Cone(radius, length);
}

shapes::Shape* shapes::Box::clone() const
{
  return new Box(size[0], size[1], size[2]);
}

shapes::Shape* shapes::Mesh::clone() const
{
  Mesh *dest = new Mesh(vertex_count, triangle_count);
  unsigned int n = 3 * vertex_count;
  for (unsigned int i = 0 ; i < n ; ++i)
    dest->vertices[i] = vertices[i];
  if (vertex_normals)
    for (unsigned int i = 0 ; i < n ; ++i)
      dest->vertex_normals[i] = vertex_normals[i];
  else
  {
    delete[] dest->vertex_normals;
    dest->vertex_normals = NULL;
  }
  n = 3 * triangle_count;
  for (unsigned int i = 0 ; i < n ; ++i)
    dest->triangles[i] = triangles[i];
  if (triangle_normals)
    for (unsigned int i = 0 ; i < n ; ++i)
      dest->triangle_normals[i] = triangle_normals[i];
  else
  {
    delete[] dest->triangle_normals;
    dest->triangle_normals = NULL;
  }
  return dest;
}

shapes::Shape* shapes::Plane::clone() const
{
  return new Plane(a, b, c, d);
}

shapes::Shape* shapes::OcTree::clone() const
{
  return new OcTree(octree);
}

void shapes::OcTree::scaleAndPadd(double scale, double padd)
{
  logWarn("OcTrees cannot be scaled or padded");
}

void shapes::Plane::scaleAndPadd(double scale, double padding)
{
  logWarn("Planes cannot be scaled or padded");
}

void shapes::Shape::scale(double scale)
{
  scaleAndPadd(scale, 0.0);
}

void shapes::Shape::padd(double padding)
{
  scaleAndPadd(1.0, padding);
}

void shapes::Sphere::scaleAndPadd(double scale, double padding)
{
  radius = radius * scale + padding;
}

void shapes::Cylinder::scaleAndPadd(double scale, double padding)
{
  radius = radius * scale + padding;
  length = length * scale + 2.0 * padding;
}

void shapes::Cone::scaleAndPadd(double scale, double padding)
{
  radius = radius * scale + padding;
  length = length * scale + 2.0 * padding;
}

void shapes::Box::scaleAndPadd(double scale, double padding)
{
  double p2 = padding * 2.0;
  size[0] = size[0] * scale + p2;
  size[1] = size[1] * scale + p2;
  size[2] = size[2] * scale + p2;
}

void shapes::Mesh::scaleAndPadd(double scale, double padding)
{
  // find the center of the mesh
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (unsigned int i = 0 ; i < vertex_count ; ++i)
  {
    unsigned int i3 = i * 3;
    sx += vertices[i3];
    sy += vertices[i3 + 1];
    sz += vertices[i3 + 2];
  }
  sx /= (double)vertex_count;
  sy /= (double)vertex_count;
  sz /= (double)vertex_count;

  // scale the mesh
  for (unsigned int i = 0 ; i < vertex_count ; ++i)
  {
    unsigned int i3 = i * 3;

    // vector from center to the vertex
    double dx = vertices[i3] - sx;
    double dy = vertices[i3 + 1] - sy;
    double dz = vertices[i3 + 2] - sz;

    // length of vector
    double norm = sqrt(dx * dx + dy * dy + dz * dz);
    if (norm > 1e-6)
    {
      double fact = scale + padding/norm;
      vertices[i3] = sx + dx * fact;
      vertices[i3 + 1] = sy + dy * fact;
      vertices[i3 + 2] = sz + dz * fact;
    }
    else
    {
      double ndx = ((dx > 0) ? dx+padding : dx-padding);
      double ndy = ((dy > 0) ? dy+padding : dy-padding);
      double ndz = ((dz > 0) ? dz+padding : dz-padding);
      vertices[i3] = sx + ndx;
      vertices[i3 + 1] = sy + ndy;
      vertices[i3 + 2] = sz + ndz;
    }
  }
}

void shapes::Shape::print(std::ostream &out) const
{
  out << this << std::endl;
}

void shapes::Sphere::print(std::ostream &out) const
{
  out << "Sphere[radius=" << radius << "]" << std::endl;
}

void shapes::Cylinder::print(std::ostream &out) const
{
  out << "Cylinder[radius=" << radius << ", length=" << length << "]" << std::endl;
}

void shapes::Cone::print(std::ostream &out) const
{
  out << "Cone[radius=" << radius << ", length=" << length << "]" << std::endl;
}

void shapes::Box::print(std::ostream &out) const
{
  out << "Box[x=length=" << size[0] << ", y=width=" << size[1] << "z=height=" << size[2] << "]" << std::endl;
}

void shapes::Mesh::print(std::ostream &out) const
{
  out << "Mesh[vertices=" << vertex_count << ", triangles=" << triangle_count << "]" << std::endl;
}

void shapes::Plane::print(std::ostream &out) const
{
  out << "Plane[a=" << a << ", b=" << b << ", c=" << c << ", d=" << d << "]" << std::endl;
}

void shapes::OcTree::print(std::ostream &out) const
{
  if (octree)
  {
    double minx, miny, minz, maxx, maxy, maxz;
    octree->getMetricMin(minx, miny, minz);
    octree->getMetricMax(maxx, maxy, maxz);
    out << "OcTree[depth = " << octree->getTreeDepth() << ", resolution = " << octree->getResolution()
        << " inside box (minx=" << minx << ", miny=" << miny << ", minz=" << minz << ", maxx=" << maxx
        << ", maxy=" << maxy << ", maxz=" << maxz << ")]" << std::endl;
  }
  else
    out << "OcTree[NULL]" << std::endl;
}

bool shapes::Shape::isFixed() const
{
  return false;
}

bool shapes::OcTree::isFixed() const
{
  return true;
}

bool shapes::Plane::isFixed() const
{
  return true;
}

void shapes::Mesh::computeTriangleNormals()
{
  if (triangle_count && !triangle_normals)
    triangle_normals = new double[triangle_count * 3];

  // compute normals
  for (unsigned int i = 0 ; i < triangle_count ; ++i)
  {
    unsigned int i3 = i * 3;
    Eigen::Vector3d s1(vertices[triangles[i3] * 3] - vertices[triangles[i3 + 1] * 3],
                       vertices[triangles[i3] * 3 + 1] - vertices[triangles[i3 + 1] * 3 + 1],
                       vertices[triangles[i3] * 3 + 2] - vertices[triangles[i3 + 1] * 3 + 2]);
    Eigen::Vector3d s2(vertices[triangles[i3 + 1] * 3] - vertices[triangles[i3 + 2] * 3],
                       vertices[triangles[i3 + 1] * 3 + 1] - vertices[triangles[i3 + 2] * 3 + 1],
                       vertices[triangles[i3 + 1] * 3 + 2] - vertices[triangles[i3 + 2] * 3 + 2]);
    Eigen::Vector3d normal = s1.cross(s2);
    normal.normalize();
    triangle_normals[i3    ] = normal.x();
    triangle_normals[i3 + 1] = normal.y();
    triangle_normals[i3 + 2] = normal.z();
  }
}

void shapes::Mesh::computeVertexNormals()
{
  if (!triangle_normals)
    computeTriangleNormals();
  if (vertex_count && !vertex_normals)
    vertex_normals = new double[vertex_count * 3];
  EigenSTL::vector_Vector3d avg_normals(vertex_count, Eigen::Vector3d(0, 0, 0));

  for (unsigned int tIdx = 0; tIdx < triangle_count; ++tIdx)
  {
    unsigned int tIdx3 = 3 * tIdx;
    unsigned int tIdx3_1 = tIdx3 + 1;
    unsigned int tIdx3_2 = tIdx3 + 2;

    unsigned int v1 = triangles [tIdx3];
    unsigned int v2 = triangles [tIdx3_1];
    unsigned int v3 = triangles [tIdx3_2];

    avg_normals[v1][0] += triangle_normals [tIdx3];
    avg_normals[v1][1] += triangle_normals [tIdx3_1];
    avg_normals[v1][2] += triangle_normals [tIdx3_2];

    avg_normals[v2][0] += triangle_normals [tIdx3];
    avg_normals[v2][1] += triangle_normals [tIdx3_1];
    avg_normals[v2][2] += triangle_normals [tIdx3_2];

    avg_normals[v3][0] += triangle_normals [tIdx3];
    avg_normals[v3][1] += triangle_normals [tIdx3_1];
    avg_normals[v3][2] += triangle_normals [tIdx3_2];
  }
  for (std::size_t i = 0 ; i < avg_normals.size() ; ++i)
  {
    if (avg_normals[i].squaredNorm () > 0.0)
      avg_normals[i].normalize();
    unsigned int i3 = i * 3;
    vertex_normals[i3] = avg_normals[i][0];
    vertex_normals[i3 + 1] = avg_normals[i][1];
    vertex_normals[i3 + 2] = avg_normals[i][2];
  }
}

void shapes::Mesh::mergeVertices(double threshold)
{
  const double thresholdSQR = threshold * threshold;

  std::vector<unsigned int> vertex_map(vertex_count);
  EigenSTL::vector_Vector3d orig_vertices(vertex_count);
  EigenSTL::vector_Vector3d compressed_vertices;

  for (unsigned int vIdx = 0; vIdx < vertex_count ; ++vIdx)
  {
    orig_vertices[vIdx][0] = vertices[3 * vIdx];
    orig_vertices[vIdx][1] = vertices[3 * vIdx + 1];
    orig_vertices[vIdx][2] = vertices[3 * vIdx + 2];
    vertex_map[vIdx] = vIdx;
  }

  for (unsigned int vIdx1 = 0; vIdx1 < vertex_count; ++vIdx1)
  {
    if (vertex_map[vIdx1] != vIdx1)
      continue;

    vertex_map[vIdx1] = compressed_vertices.size();
    compressed_vertices.push_back(orig_vertices[vIdx1]);

    for (unsigned int vIdx2 = vIdx1 + 1 ; vIdx2 < vertex_count ; ++vIdx2)
    {
      double distanceSQR = (orig_vertices[vIdx1] - orig_vertices[vIdx2]).squaredNorm();
      if (distanceSQR <= thresholdSQR)
        vertex_map[vIdx2] = vertex_map[vIdx1];
    }
  }

  if (compressed_vertices.size() == orig_vertices.size())
    return;

  // redirect triangles to new vertices!
  for (unsigned int tIdx = 0; tIdx < triangle_count; ++tIdx)
  {
    unsigned int i3 = 3 * tIdx;
    triangles[i3] =  vertex_map[triangles [i3]];
    triangles[i3 + 1] = vertex_map[triangles [i3 + 1]];
    triangles[i3 + 2] = vertex_map[triangles [i3 + 2]];
  }

  vertex_count = compressed_vertices.size();
  delete[] vertices;
  vertices = new double[vertex_count * 3];

  for (unsigned int vIdx = 0; vIdx < vertex_count ; ++vIdx)
  {
    unsigned int i3 = 3 * vIdx;
    vertices[i3] = compressed_vertices[vIdx][0];
    vertices[i3 + 1] = compressed_vertices[vIdx][1];
    vertices[i3 + 2] = compressed_vertices[vIdx][2];
  }

  if (triangle_normals)
    computeTriangleNormals();
  if (vertex_normals)
    computeVertexNormals();
}
