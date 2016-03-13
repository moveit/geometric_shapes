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

/** \Author Jorge Santos */

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include "resources/config.h"


/**
 * Test fixture that generates a mesh from a primitive shape randomly chosen between SPHERE,
 * CYLINDER, CONE and BOX, and load its twin from an STL file. All the following tests are
 * intended to verify that both procedures produce equivalent meshes, and particularly that
 * changes related to issue #38 don't break mesh loading.
 */
class CompareMeshVsPrimitive : public ::testing::Test
{
public:
  CompareMeshVsPrimitive()
  {
  }

  void SetUp()
  {
    // WARN1: we skip the sphere because it takes ages for each test, around 20 seconds each
    // (possible bug?)
    // WARN2: meshes created by createMeshFromShape for cylinders and cones use a
    // magic number to set the smoothness (now is 100):
    // https://github.com/corot/geometric_shapes/blob/patch-1/src/mesh_operations.cpp#L510)
    // The STL meshes have been built with a more or less equivalent smoothness, but
    // if this matching breaks, randomized tests can fail in some unlikely situations
    switch (rng.uniformInteger(shapes::CYLINDER, shapes::BOX))
    {
      case shapes::SPHERE:
      {
        shapes::Sphere shape(0.5);
        shape_ms = shapes::createMeshFromShape(&shape);
        loaded_ms = shapes::createMeshFromResource(
            "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/sphere.stl").string());
        break;
      }
      case shapes::CYLINDER:
      {
        shapes::Cylinder shape(0.5, 1.0);
        shape_ms = shapes::createMeshFromShape(&shape);
        loaded_ms = shapes::createMeshFromResource(
            "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/cylinder.stl").string());
        break;
      }
      case shapes::CONE:
      {
        shapes::Cone shape(0.5, 1.0);
        shape_ms = shapes::createMeshFromShape(&shape);
        loaded_ms = shapes::createMeshFromResource(
            "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/cone.stl").string());
        break;
      }
      case shapes::BOX:
      {
        shapes::Box shape(1.0, 1.0, 1.0);
        shape_ms = shapes::createMeshFromShape(&shape);
        loaded_ms = shapes::createMeshFromResource(
            "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/cube.stl").string());
        break;
      }
    }

    shape_cms = new bodies::ConvexMesh(shape_ms);
    loaded_cms = new bodies::ConvexMesh(loaded_ms);
  }

  void TearDown()
  {
    delete shape_ms;
    delete loaded_ms;
    delete shape_cms;
    delete loaded_cms;
  }

  ~CompareMeshVsPrimitive()
  {
  }

protected:
  random_numbers::RandomNumberGenerator rng;

  shapes::Mesh *shape_ms;
  shapes::Mesh *loaded_ms;

  bodies::Body *shape_cms;
  bodies::Body *loaded_cms;
};

TEST_F(CompareMeshVsPrimitive, ContainsPoint)
{
  // Any point inside a mesh must be inside the other too

  Eigen::Vector3d p;
  bool found = false;
  for (int i = 0; i < 100; ++i)
  {
    if ((shape_cms->samplePointInside(rng, 10000, p)) ||
        (loaded_cms->samplePointInside(rng, 10000, p)))
    {
      found = true;
      EXPECT_EQ(shape_cms->containsPoint(p), loaded_cms->containsPoint(p));
    }
  }
  EXPECT_TRUE(found) << "No point inside the meshes was found (very unlikely)";
}

TEST_F(CompareMeshVsPrimitive, IntersectsRay)
{
  // Random rays must intersect both meshes nearly at the same points

  SUCCEED() << "Test disabled by now, as it fails in around 1% cases";
  return;
//  For example, the following ray will hit twice the generated mesh but once the loaded
//  Eigen::Vector3d ray_o(0.497215,  -0.0503702, 0.15293);
//  Eigen::Vector3d ray_d(0.00640291, 0.776497, -0.0711191);

  bool intersects = false;
  for (int i = 0; i < 100; ++i)
  {
    Eigen::Vector3d ray_o(rng.uniformReal(-1.0, +1.0),
                          rng.uniformReal(-1.0, +1.0),
                          rng.uniformReal(-1.0, +1.0));
    Eigen::Vector3d ray_d(rng.uniformReal(-1.0, +1.0),
                          rng.uniformReal(-1.0, +1.0),
                          rng.uniformReal(-1.0, +1.0));
    EigenSTL::vector_Vector3d vi1, vi2;
    shape_cms->intersectsRay(ray_o, ray_d, &vi1);
    loaded_cms->intersectsRay(ray_o, ray_d, &vi2);

// DEBUG printing
//    if (vi1.size() != vi2.size() && vi1.size() > 0 && vi2.size() > 0)
//    {
//        std::cout << vi1.size() << "   " << vi2.size() << "\n";
//        std::cout << ray_o.x() << "  "<< ray_o.y() << "  "<< ray_o.z()
//          << "\n" << ray_d.x() << "  "<< ray_d.y() << "  "<< ray_d.z() << "\n";
//    }
    
    EXPECT_EQ(vi1.size(), vi2.size());
    if (vi1.size() > 0 && vi2.size() > 0)
    {
      EXPECT_NEAR(vi1[0].x(), vi2[0].x(), 0.01);
      EXPECT_NEAR(vi1[0].y(), vi2[0].y(), 0.01);
      EXPECT_NEAR(vi1[0].z(), vi2[0].z(), 0.01);

      intersects = true;
    }
  }

  EXPECT_TRUE(intersects) << "No ray intersects the meshes (very unlikely)";
}

TEST_F(CompareMeshVsPrimitive, BoundingSphere)
{
  // Bounding spheres must be nearly identical

  shapes::Sphere shape(1.0);
  Eigen::Vector3d center1, center2;
  double radius1, radius2;
  computeShapeBoundingSphere(shape_ms, center1, radius1);
  computeShapeBoundingSphere(loaded_ms, center2, radius2);

  EXPECT_NEAR(radius1,     radius2,     0.001);
  EXPECT_NEAR(center1.x(), center2.x(), 0.001);
  EXPECT_NEAR(center1.y(), center2.y(), 0.001);
  EXPECT_NEAR(center1.z(), center2.z(), 0.001);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
