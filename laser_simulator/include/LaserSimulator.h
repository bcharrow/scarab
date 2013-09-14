/*
  Copyright (C) 2013 Nathan Michael

  This file is part of laser_simulator, a laser simulator for ROS.

  mesh80211s is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __LASERSIMULATOR__
#define __LASERSIMULATOR__

#include <vector>
#include <map>
#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <laser_simulator/PoseStampedNamed.h>
#include <laser_simulator/PoseStampedNamedArray.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>

typedef struct Model
{
  double xdim, ydim, zdim;
  Model() : xdim(0), ydim(0), zdim(0) {}
  Model(double x, double y, double z) :
    xdim(x), ydim(y), zdim(z) {}
} model_t;

class LaserSimulator
{
public:
  LaserSimulator();
  ~LaserSimulator();

  void SetPose(const geometry_msgs::Pose& pose);
  void SetFrameID(const std::string& frame_id);
  void LoadOccupancyGrid(const nav_msgs::OccupancyGrid& map, double depth);
  void UpdatePoseArray(const laser_simulator::PoseStampedNamedArray& pose_array);
  int LoadLaserModel(const ros::NodeHandle& n);
  int LoadDynamicModels(const ros::NodeHandle& n);
  void GetScan(std::vector<float>& ranges);
  void SetLaserOffset(const geometry_msgs::Pose& offset);
  void SetOccupiedThreshold(const int8_t t) { assert(t>=0 && t<=100); occupied_threshold = t; }
  void SetLaserNoiseStdDev(const double noise) { noise_sd = noise; }

  double GetMinimumAngle() { assert(initialized); return minimum_angle; }
  double GetMaximumAngle() { assert(initialized); return maximum_angle; }
  double GetAngleIncrement() { assert(initialized); return angle_increment; }
  double GetTimeIncrement() { assert(initialized); return time_increment; }
  double GetScanTime() { assert(initialized); return scan_time; }
  double GetMinimumRange() { assert(initialized); return minimum_range; }
  double GetMaximumRange() { assert(initialized); return maximum_range; }
  int GetScanCount() { assert(initialized); return number_of_ranges; }

  double GetNoiseStdDev() { return noise_sd; }

private:
  bool initialized;
  int8_t occupied_threshold;

  double minimum_angle;
  double maximum_angle;
  double angle_increment;
  double time_increment;
  double scan_time;
  double minimum_range;
  double maximum_range;
  int number_of_ranges;

  double noise_sd;

  std::string frame_id;

  geometry_msgs::Pose pose, offset;
  nav_msgs::OccupancyGrid map;
  std::list< std::pair<float, float> > scan_points;

  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point;
  typedef K::Segment_3 Segment;
  typedef CGAL::Triangle_3<K> Triangle;
  typedef std::list<Triangle>::iterator Iterator;
  typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef Tree::Object_and_primitive_id Object_and_primitive_id;

  std::list<Triangle> triangles;
  Tree tree;
  Tree dynamic_tree;

  std::map<std::string, model_t*> models;
};
#endif
