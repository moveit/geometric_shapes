/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Open Robotics
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

/** \Author Ioan Sucan */
/** \Author Martin Pecka */

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include "resources/config.h"

#define EXPECT_VECTORS_EQUAL(v1, v2, error)                                                                            \
  EXPECT_NEAR((v1)[0], (v2)[0], (error));                                                                              \
  EXPECT_NEAR((v1)[1], (v2)[1], (error));                                                                              \
  EXPECT_NEAR((v1)[2], (v2)[2], (error));

#define CHECK_NO_INTERSECTION(body, origin, direction)                                                                 \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    const auto result = (body).intersectsRay(o, d, &intersections, 2);                                                 \
    EXPECT_FALSE(result);                                                                                              \
    EXPECT_EQ(0, intersections.size());                                                                                \
  }

#define CHECK_INTERSECTS(body, origin, direction, numIntersections)                                                    \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    const auto result = (body).intersectsRay(o, d, &intersections, 2);                                                 \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ((numIntersections), intersections.size());                                                               \
  }

#define CHECK_INTERSECTS_ONCE(body, origin, direction, intersection, error)                                            \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    Eigen::Vector3d i intersection;                                                                                    \
    const auto result = (body).intersectsRay(o, d, &intersections, 2);                                                 \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ(1, intersections.size());                                                                                \
    EXPECT_VECTORS_EQUAL(intersections.at(0), i, (error));                                                             \
  }

#define CHECK_INTERSECTS_TWICE(body, origin, direction, intersc1, intersc2, error)                                     \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    Eigen::Vector3d i1 intersc1;                                                                                       \
    Eigen::Vector3d i2 intersc2;                                                                                       \
    const auto result = (body).intersectsRay(o, d, &intersections, 2);                                                 \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ(2, intersections.size());                                                                                \
    if (fabs(static_cast<double>((intersections.at(0) - i1).norm())) < (error))                                        \
    {                                                                                                                  \
      EXPECT_VECTORS_EQUAL(intersections.at(0), i1, (error));                                                          \
      EXPECT_VECTORS_EQUAL(intersections.at(1), i2, (error));                                                          \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      EXPECT_VECTORS_EQUAL(intersections.at(0), i2, (error));                                                          \
      EXPECT_VECTORS_EQUAL(intersections.at(1), i1, (error));                                                          \
    }                                                                                                                  \
  }

TEST(SphereRayIntersection, SimpleRay1)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);
  sphere.setScale(1.05);

  CHECK_INTERSECTS_TWICE(sphere, (5, 0, 0), (-1, 0, 0), (1.05, 0, 0), (-1.05, 0, 0), 1e-6)
}

TEST(SphereRayIntersection, SimpleRay2)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);
  sphere.setScale(1.05);

  CHECK_NO_INTERSECTION(sphere, (5, 0, 0), (1, 0, 0))
}

TEST(BoxRayIntersection, SimpleRay1)
{
  shapes::Box shape(1.0, 1.0, 3.0);
  bodies::Box box(&shape);
  box.setScale(0.95);

  CHECK_INTERSECTS_TWICE(box, (10, 0.449, 0), (-1, 0, 0), (0.475, 0.449, 0), (-0.475, 0.449, 0), 1e-4)
}

TEST(BoxRayIntersection, SimpleRay2)
{
  shapes::Box shape(0.9, 0.01, 1.2);
  bodies::Box box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0, 0.005, 0.6);
  box.setPose(pose);

  const Eigen::Vector3d ray_d(0, -5.195, -0.77);

  CHECK_INTERSECTS(box, (0, 5, 1.6), (ray_d.normalized()), 2)
}

TEST(BoxRayIntersection, SimpleRay3)
{
  shapes::Box shape(0.02, 0.4, 1.2);
  bodies::Box box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0.45, -0.195, 0.6);
  box.setPose(pose);

  const Eigen::Vector3d ray_d(0, 1.8, -0.669);

  CHECK_NO_INTERSECTION(box, (0, -2, 1.11), (ray_d.normalized()))
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
