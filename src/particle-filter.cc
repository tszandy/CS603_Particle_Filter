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
\file    particle-filter.cc
\brief   COMPSCI603 Particle Filter Implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/math_util.h"

#include "particle-filter.h"

using math_util::DegToRad;
using math_util::Sq;
using std::string;
using std::fabs;
using std::min;
using std::max;
using std::vector;
using Eigen::Rotation2Df;
using Eigen::Vector2f;

namespace COMPSCI603 {

float Cross(const Vector2f& v1, const Vector2f& v2) {
  return (v1.x() * v2.y() - v1.y() * v2.x());
}

bool Line::Intersection(const Vector2f& p2,
                        const Vector2f& p3,
                        Vector2f* intersection) const {
  // Bounding-box broad phase check.
  if (min(p0.x(), p1.x()) > max(p2.x(), p3.x())) return false;
  if (max(p0.x(), p1.x()) < min(p2.x(), p3.x())) return false;
  if (min(p0.y(), p1.y()) > max(p2.y(), p3.y())) return false;
  if (max(p0.y(), p1.y()) < min(p2.y(), p3.y())) return false;
  // Narrow-phase check.
  const Vector2f d1 = p1 - p0;
  const Vector2f d2 = p3 - p2;
  if (Cross(d1, p3 - p0) * Cross(d1, p2 - p0) >= 0.0) return false;
  if (Cross(d2, p1 - p2) * Cross(d2, p0 - p2) >= 0.0) return false;
  // Okay, the line segments definitely intersect.
  const float d = Cross(d2, -d1);
  // Just an extra check, should never happen
  if (d == 0.0f) return false;
  const float tb = Cross(p0 -p2, p0-p1) / d;
  *intersection = p2 + tb * d2;
  return true;
}

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedScan(const Vector2f& loc,
                                      const float angle,
                                      const vector<float>& ranges,
                                      float range_min,
                                      float range_max,
                                      float angle_min,
                                      float angle_max,
                                      vector<Vector2f>* scan_ptr) {
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Placeholders to remove unused private field warning
  // Remove or edit these in your own code
  prev_odom_angle_ = 0;
  odom_initialized_ = false;
}

void ParticleFilter::Resample() {
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
}

void ParticleFilter::Initialize(const VectorMap& map,
                                const Vector2f& loc,
                                const float angle) {
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
}


}  // namespace COMPSCI603
