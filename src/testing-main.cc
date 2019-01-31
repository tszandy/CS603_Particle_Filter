//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    testing-main.cc
\brief   Unit Testing for particle filter parts.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <stdio.h>

#include "eigen3/Eigen/Dense"
#include "gtest/gtest.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "particle-filter.h"

using COMPSCI603::Line;
using Eigen::Vector2f;

TEST(LineIntersection, Intersects) {
  {
    const Line l(Vector2f(-1, 0), Vector2f(1, 0));
    const Vector2f p2(0, -1);
    const Vector2f p3(0, 1);
    Vector2f intersection(0, 0);
    ASSERT_TRUE(l.Intersection(p2, p3, &intersection));
    ASSERT_FLOAT_EQ(intersection.x(), 0);
    ASSERT_FLOAT_EQ(intersection.y(), 0);
  }
  {
    const Line l(Vector2f(-1, -1), Vector2f(3, 3));
    const Vector2f p2(2, -5);
    const Vector2f p3(-2, 5);
    Vector2f intersection(0, 0);
    ASSERT_TRUE(l.Intersection(p2, p3, &intersection));
    ASSERT_FLOAT_EQ(intersection.x(), 0);
    ASSERT_FLOAT_EQ(intersection.y(), 0);
  }
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
