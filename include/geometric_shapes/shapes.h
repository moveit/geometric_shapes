/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/*  Author: Ioan Sucan */

#ifndef GEOMETRIC_SHAPES_SHAPES_
#define GEOMETRIC_SHAPES_SHAPES_

#include <cstdlib>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <string>

namespace octomap
{
class OcTree;
}

/** \brief Definition of various shapes. No properties such as
    position are included. These are simply the descriptions and
    dimensions of shapes. */
namespace shapes
{

/** \brief A list of known shape types */
enum ShapeType { UNKNOWN_SHAPE, SPHERE, CYLINDER, CONE, BOX, PLANE, MESH, OCTREE };


/** \brief A basic definition of a shape. Shapes are considered centered at origin */
class Shape
{
public:
  Shape();
  virtual ~Shape();

  /** \brief Create a copy of this shape */
  virtual Shape* clone() const = 0;

  /** \brief Print information about this shape */
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief Scale this shape by a factor */
  void scale(double scale);

  /** \brief Add padding to this shape */
  void padd(double padding);

  /** \brief Scale and padd this shape */
  virtual void scaleAndPadd(double scale, double padd) = 0;

  /** \brief Return a flag indicating whether this shape can be scaled and/or padded */
  virtual bool isFixed() const;

  /** \brief The type of the shape */
  ShapeType type;
};

/** \brief Definition of a sphere */
class Sphere : public Shape
{
public:
  Sphere();

  /** \brief The radius of the shpere */
  Sphere(double r);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The radius of the sphere */
  double radius;
};

/** \brief Definition of a cylinder
 * Length is along z axis.  Origin is at center of mass. */
class Cylinder : public Shape
{
public:
  Cylinder();

  /** \brief The radius and the length of the cylinder */
  Cylinder(double r, double l);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The length of the cylinder */
  double length;

  /** \brief The radius of the cylinder */
  double radius;
};

/** \brief Definition of a cone
 * Tip is on positive z axis.  Center of base is on negative z axis.  Origin is
 * halway between tip and center of base. */
class Cone : public Shape
{
public:
  Cone();
  Cone(double r, double l);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The length (height) of the cone */
  double length;

  /** \brief The radius of the cone */
  double radius;
};

/** \brief Definition of a box
 * Aligned with the XYZ axes. */
class Box : public Shape
{
public:
  Box();
  Box(double x, double y, double z);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief x, y, z dimensions of the box (axis-aligned) */
  double size[3];
};

/** \brief Definition of a triangle mesh
 * By convention the "center" of the shape is at the origin.  For a mesh this
 * implies that the AABB of the mesh is centered at the origin.  Some methods
 * may not work with arbitrary meshes whose AABB is not centered at the origin.
 * */
class Mesh : public Shape
{
public:

  Mesh();
  Mesh(unsigned int v_count, unsigned int t_count);
  virtual ~Mesh();

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual void scaleAndPadd(double scale, double padd);
  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief The normals to each triangle can be computed from the vertices using cross products. This function performs this computation and allocates memory for normals if needed */
  void computeTriangleNormals();

  /** \brief The normals to each vertex, averaged from the triangle normals. computeTriangleNormals() is automatically called if needed. */
  void computeVertexNormals();

  /** \brief Merge vertices that are very close to each other, up to a threshold*/
  void mergeVertices(double threshold);

  /** \brief The number of available vertices */
  unsigned int  vertex_count;

  /** \brief The position for each vertex vertex k has values at
   * index (3k, 3k+1, 3k+2) = (x,y,z) */
  double       *vertices;

  /** \brief The number of triangles formed with the vertices */
  unsigned int  triangle_count;

  /** \brief The vertex indices for each triangle
   * triangle k has vertices at index (3k, 3k+1, 3k+2) = (v1, v2, v3) */
  unsigned int *triangles;

  /** \brief The normal to each triangle; unit vector represented
      as (x,y,z); If missing from the mesh, these vectors can be computed using computeTriangleNormals() */
  double       *triangle_normals;

  /** \brief The normal to each vertex; unit vector represented
      as (x,y,z); If missing from the mesh, these vectors can be computed using computeVertexNormals()  */
  double       *vertex_normals;
};

/** \brief Definition of a plane with equation ax + by + cz + d = 0 */
class Plane : public Shape
{
public:

  Plane();
  Plane(double pa, double pb, double pc, double pd);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;
  virtual void scaleAndPadd(double scale, double padd);
  virtual bool isFixed() const;

  /** \brief The plane equation is ax + by + cz + d = 0 */
  double a, b, c, d;
};

/** \brief Representation of an octomap::OcTree as a Shape */
class OcTree : public Shape
{
public:
  OcTree();
  OcTree(const boost::shared_ptr<const octomap::OcTree> &t);

  /** \brief The type of the shape, as a string */
  static const std::string STRING_NAME;

  virtual Shape* clone() const;
  virtual void print(std::ostream &out = std::cout) const;
  virtual void scaleAndPadd(double scale, double padd);
  virtual bool isFixed() const;

  boost::shared_ptr<const octomap::OcTree> octree;
};


/** \brief Shared pointer to a Shape */
typedef boost::shared_ptr<Shape> ShapePtr;

/** \brief Shared pointer to a const Shape */
typedef boost::shared_ptr<const Shape> ShapeConstPtr;

}

#endif
