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

TEST(SphereRayIntersection, SimpleRay1)
{
  shapes::Sphere shape(1.0);
  bodies::Body* sphere = new bodies::Sphere(&shape);
  sphere->setScale(1.05);

  Eigen::Vector3d ray_o(5, 0, 0);
  Eigen::Vector3d ray_d(-1, 0, 0);
  EigenSTL::vector_Vector3d p;
  bool intersect = sphere->intersectsRay(ray_o, ray_d, &p);

  delete sphere;
  EXPECT_TRUE(intersect);
  EXPECT_EQ(2, (int)p.size());
  EXPECT_NEAR(p[0].x(), 1.05, 1e-6);
  EXPECT_NEAR(p[1].x(), -1.05, 1e-6);
}

TEST(SphereRayIntersection, SimpleRay2)
{
  shapes::Sphere shape(1.0);
  bodies::Body* sphere = new bodies::Sphere(&shape);
  sphere->setScale(1.05);

  Eigen::Vector3d ray_o(5, 0, 0);
  Eigen::Vector3d ray_d(1, 0, 0);
  EigenSTL::vector_Vector3d p;
  bool intersect = sphere->intersectsRay(ray_o, ray_d, &p);

  delete sphere;
  EXPECT_FALSE(intersect);
  EXPECT_EQ(0, (int)p.size());
}

TEST(BoxRayIntersection, SimpleRay1)
{
  shapes::Box shape(1.0, 1.0, 3.0);
  bodies::Body* box = new bodies::Box(&shape);
  box->setScale(0.95);

  Eigen::Vector3d ray_o(10, 0.449, 0);
  Eigen::Vector3d ray_d(-1, 0, 0);
  EigenSTL::vector_Vector3d p;

  bool intersect = box->intersectsRay(ray_o, ray_d, &p);

  //    for (unsigned int i = 0; i < p.size() ; ++i)
  //        printf("intersection at %f, %f, %f\n", p[i].x(), p[i].y(), p[i].z());

  delete box;
  EXPECT_TRUE(intersect);
}

TEST(BoxRayIntersection, SimpleRay2)
{
  shapes::Box shape(0.9, 0.01, 1.2);
  bodies::Body* box = new bodies::Box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0, 0.005, 0.6);
  box->setPose(pose);

  Eigen::Vector3d ray_o(0, 5, 1.6);
  Eigen::Vector3d ray_d(0, -5.195, -0.77);
  EigenSTL::vector_Vector3d p;

  bool intersect = box->intersectsRay(ray_o, ray_d, &p);
  EXPECT_TRUE(intersect);

  intersect = box->intersectsRay(ray_o, ray_d.normalized(), &p);
  EXPECT_TRUE(intersect);

  delete box;
}

TEST(BoxRayIntersection, SimpleRay3)
{
  shapes::Box shape(0.02, 0.4, 1.2);
  bodies::Body* box = new bodies::Box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0.45, -0.195, 0.6);
  box->setPose(pose);

  Eigen::Vector3d ray_o(0, -2, 1.11);
  Eigen::Vector3d ray_d(0, 1.8, -0.669);
  EigenSTL::vector_Vector3d p;

  bool intersect = box->intersectsRay(ray_o, ray_d, &p);
  EXPECT_FALSE(intersect);

  intersect = box->intersectsRay(ray_o, ray_d.normalized(), &p);
  EXPECT_FALSE(intersect);

  delete box;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
