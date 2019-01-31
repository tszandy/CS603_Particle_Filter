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
\file    slam_frontend.h
\brief   COMPSCI603 Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"

#ifndef __SLAM_FRONTEND_H__
#define __SLAM_FRONTEND_H__

namespace COMPSCI603 {

struct Line {
  Eigen::Vector2f p0;
  Eigen::Vector2f p1;
  Line() {}
  Line(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1) : p0(p0), p1(p1) {}
  bool Intersection(const Eigen::Vector2f& p2,
                    const Eigen::Vector2f& p3,
                    Eigen::Vector2f* intersection) const;
};

typedef std::vector<Line> VectorMap;

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const VectorMap& map,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted scan from current location.
  void GetPredictedScan(const Eigen::Vector2f& loc,
                        const float angle,
                        const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        std::vector<Eigen::Vector2f>* scan);

 private:
  // Add any other private members you need here. Do not forget to initialize
  // them!

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
};
}  // namespace COMPSCI603

#endif   // __SLAM_FRONTEND_H__
