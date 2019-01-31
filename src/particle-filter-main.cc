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
\file    particle-filter-main.cc
\brief   Main entry point for COMPSCI603 assignment on particle filter based
         mobile robot localization
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"
#include "shared/util/timer.h"

#include "particle-filter.h"

using ros::Time;
using std::string;
using std::vector;
using Eigen::Vector2f;

DEFINE_string(laser_topic, "laser", "Name of ROS topic for image data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(init_topic, "init", "Name of ROS topic for initialization");
DEFINE_string(map, "maps/NSH4.txt", "Name of vector map file");
DEFINE_string(input, "", "Name of ROS bag file to load");
DEFINE_bool(fast, false, "Replay as fast as possible");
DECLARE_string(helpon);
DECLARE_int32(v);

COMPSCI603::VectorMap map_;
COMPSCI603::ParticleFilter particle_filter_;
ros::Publisher particles_publisher_;
ros::Publisher map_publisher_;
ros::Publisher trajectory_publisher_;
ros::Publisher laser_publisher_;
ros::Publisher predicted_scan_publisher_;
geometry_msgs::PoseArray particles_msg_;
visualization_msgs::Marker map_msg_;
visualization_msgs::Marker trajectory_msg_;
visualization_msgs::Marker predicted_scan_msg_;
sensor_msgs::LaserScan last_laser_msg_;
tf::TransformBroadcaster* tf_broadcaster_;

geometry_msgs::Point EigenToRosPoint(const Vector2f& p) {
  geometry_msgs::Point p2;
  p2.x = p.x();
  p2.y = p.y();
  p2.z = 0;
  return p2;
}

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;
  particles_msg_.header = header;
  map_msg_.header = header;
  map_msg_.type = visualization_msgs::Marker::LINE_LIST;
  map_msg_.action = visualization_msgs::Marker::ADD;
  map_msg_.color.a = 1;
  map_msg_.color.r = 66.0 / 255.0;
  map_msg_.color.g = 134.0 / 255.0;
  map_msg_.color.b = 244.0 / 255.0;
  map_msg_.pose.position.x = 0;
  map_msg_.pose.position.y = 0;
  map_msg_.pose.position.z = 0;
  map_msg_.pose.orientation.x = 0.0;
  map_msg_.pose.orientation.y = 0.0;
  map_msg_.pose.orientation.z = 0.0;
  map_msg_.pose.orientation.w = 1.0;
  map_msg_.scale.x = 0.1;
  map_msg_.scale.y = 1;
  map_msg_.scale.z = 1;
  map_msg_.ns = "map";
  map_msg_.id = 0;

  trajectory_msg_.header = header;
  trajectory_msg_.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_msg_.action = visualization_msgs::Marker::ADD;
  trajectory_msg_.color.a = 1;
  trajectory_msg_.color.r = 178.0 / 255.0;
  trajectory_msg_.color.g = 178.0 / 255.0;
  trajectory_msg_.color.b = 178.0 / 255.0;
  trajectory_msg_.pose.position.x = 0;
  trajectory_msg_.pose.position.y = 0;
  trajectory_msg_.pose.position.z = 0;
  trajectory_msg_.pose.orientation.x = 0.0;
  trajectory_msg_.pose.orientation.y = 0.0;
  trajectory_msg_.pose.orientation.z = 0.0;
  trajectory_msg_.pose.orientation.w = 1.0;
  trajectory_msg_.scale.x = 0.1;
  trajectory_msg_.scale.y = 1;
  trajectory_msg_.scale.z = 1;
  trajectory_msg_.ns = "trajectory";
  trajectory_msg_.id = 0;

  predicted_scan_msg_.header = header;
  predicted_scan_msg_.type = visualization_msgs::Marker::LINE_LIST;
  predicted_scan_msg_.action = visualization_msgs::Marker::ADD;
  predicted_scan_msg_.color.a = 0.2;
  predicted_scan_msg_.color.r = 67.0 / 255.0;
  predicted_scan_msg_.color.g = 178.0 / 255.0;
  predicted_scan_msg_.color.b = 89.0 / 255.0;
  predicted_scan_msg_.pose.position.x = 0;
  predicted_scan_msg_.pose.position.y = 0;
  predicted_scan_msg_.pose.position.z = 0;
  predicted_scan_msg_.pose.orientation.x = 0.0;
  predicted_scan_msg_.pose.orientation.y = 0.0;
  predicted_scan_msg_.pose.orientation.z = 0.0;
  predicted_scan_msg_.pose.orientation.w = 1.0;
  predicted_scan_msg_.scale.x = 0.1;
  predicted_scan_msg_.scale.y = 1;
  predicted_scan_msg_.scale.z = 1;
  predicted_scan_msg_.ns = "predicted_scan";
  predicted_scan_msg_.id = 0;
}

void PublishVisualization() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.016) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vector<COMPSCI603::Particle> particles;
  particle_filter_.GetParticles(&particles);
  particles_msg_.poses.clear();
  for (const COMPSCI603::Particle& p : particles) {
    geometry_msgs::Pose pose;
    pose.position.x = p.loc.x();
    pose.position.y = p.loc.y();
    pose.position.z = 0;
    pose.orientation.w = cos(0.5 * p.angle);
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(0.5 * p.angle);
    particles_msg_.poses.push_back(pose);
  }
  particles_msg_.header.stamp = ros::Time::now();
  particles_publisher_.publish(particles_msg_);
  map_msg_.header.stamp = ros::Time::now();
  map_publisher_.publish(map_msg_);
  last_laser_msg_.header.stamp = ros::Time::now();
  laser_publisher_.publish(last_laser_msg_);

  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  particle_filter_.GetLocation(&robot_loc, &robot_angle);

  vector<Vector2f> predicted_scan;
  particle_filter_.GetPredictedScan(
      robot_loc,
      robot_angle,
      last_laser_msg_.ranges,
      last_laser_msg_.range_min,
      last_laser_msg_.range_max,
      last_laser_msg_.angle_min,
      last_laser_msg_.angle_max,
      &predicted_scan);
  predicted_scan_msg_.points.clear();
  for (const Vector2f& p : predicted_scan) {
    predicted_scan_msg_.points.push_back(EigenToRosPoint(robot_loc));
    predicted_scan_msg_.points.push_back(EigenToRosPoint(p));
  }
  predicted_scan_msg_.header.stamp = ros::Time::now();
  predicted_scan_publisher_.publish(predicted_scan_msg_);

  trajectory_msg_.points.push_back(EigenToRosPoint(robot_loc));
  trajectory_msg_.header.stamp = ros::Time::now();
  trajectory_publisher_.publish(trajectory_msg_);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(robot_loc.x(), robot_loc.y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, robot_angle);
  transform.setRotation(q);
  tf_broadcaster_->sendTransform(tf::StampedTransform(transform,
                                                      ros::Time::now(),
                                                      "map",
                                                      "base_laser"));
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  particle_filter_.ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishVisualization();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  particle_filter_.ObserveOdometry(odom_loc, odom_angle);
  PublishVisualization();
}

void InitCallback(const nav_msgs::Odometry& msg) {
  const Vector2f init_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float init_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  printf("Initialize: (%f,%f) %f\n", init_loc.x(), init_loc.y(), init_angle);
  particle_filter_.Initialize(map_, init_loc, init_angle);
}


void ProcessBagfile(const char* filename, ros::NodeHandle* n) {
  rosbag::Bag bag;
  try {
    bag.open(filename,rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read %s, reason:\n %s\n", filename, exception.what());
    return;
  }
  printf("Processing %s\n", filename);

  vector<string> topics;
  topics.push_back(FLAGS_laser_topic.c_str());
  topics.push_back(FLAGS_odom_topic.c_str());
  topics.push_back(FLAGS_init_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  double bag_t_start = -1;
  double bag_t_last = -1;
  double t_last = GetMonotonicTime();
  // Iterate for every message.
  for (rosbag::View::iterator it = view.begin();
       it != view.end() && ros::ok();
       ++it) {
    const rosbag::MessageInstance& message = *it;
    const double bag_t_now = message.getTime().toSec();
    if (bag_t_start < 0.0) {
      bag_t_start = bag_t_now;
      bag_t_last = bag_t_now;
    }
    const double t_now = GetMonotonicTime();
    if (!FLAGS_fast) {
      const double delta_t = (bag_t_now - bag_t_last) - (t_now - t_last);
      Sleep(std::max(0.0, delta_t));
    }
    t_last = t_now;
    bag_t_last = bag_t_now;
    ros::spinOnce();
    {
      sensor_msgs::LaserScanPtr laser_msg =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_msg != NULL) {
        LaserCallback(*laser_msg);
      }
    }
    {
      nav_msgs::OdometryPtr odom_msg =
          message.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL) {
        if (message.getTopic() == FLAGS_odom_topic) {
          OdometryCallback(*odom_msg);
        } else if (message.getTopic() == FLAGS_init_topic) {
          InitCallback(*odom_msg);
        }
      }
    }
  }
}

void LoadMap(const string& file) {
  FILE* fid = fopen(file.c_str(), "r");
  if (fid == NULL) {
    fprintf(stderr, "ERROR: Unable to load map %s\n", file.c_str());
    exit(1);
  }
  float x1(0), y1(0), x2(0), y2(0);
  while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
    map_.push_back(COMPSCI603::Line(Vector2f(x1, y1), Vector2f(x2, y2)));
  }
  fclose(fid);

  for (const COMPSCI603::Line& l : map_) {
    map_msg_.points.push_back(EigenToRosPoint(l.p0));
    map_msg_.points.push_back(EigenToRosPoint(l.p1));
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }
  // Initialize ROS.
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle n;
  tf_broadcaster_ = new tf::TransformBroadcaster();
  LoadMap(FLAGS_map);
  InitializeMsgs();
  particles_publisher_ = n.advertise<geometry_msgs::PoseArray>("particles", 1);
  map_publisher_ = n.advertise<visualization_msgs::Marker>("map", 1);
  trajectory_publisher_ =
      n.advertise<visualization_msgs::Marker>("trajectory", 1);
  predicted_scan_publisher_ =
      n.advertise<visualization_msgs::Marker>("predicted_scan", 1);
  laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("laser", 1);
  ProcessBagfile(FLAGS_input.c_str(), &n);
  delete tf_broadcaster_;
  return 0;
}
