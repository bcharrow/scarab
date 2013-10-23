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

#include "LaserSimulator.h"
#include <cmath>
#include <time.h>
#include <armadillo>
#include <boost/tuple/tuple.hpp>

LaserSimulator::LaserSimulator() : occupied_threshold(0), noise_sd(0.0)
{
  return;
}

LaserSimulator::~LaserSimulator()
{
  for (std::map<std::string, model_t*>::iterator i = models.begin();
       i != models.end(); ++i)
    delete i->second;

  return;
}

void LaserSimulator::SetPose(const geometry_msgs::Pose& pose_)
{
  pose = pose_;
}

void LaserSimulator::SetFrameID(const std::string& frame_id_)
{
  frame_id = frame_id_;
}

void LaserSimulator::SetLaserOffset(const geometry_msgs::Pose& offset_)
{
  offset = offset_;
}

void LaserSimulator::LoadOccupancyGrid(const nav_msgs::OccupancyGrid& map,
                                       double depth)
{
  triangles.clear();

  double x = map.info.origin.position.x;
  double y = map.info.origin.position.y;
  double z = map.info.origin.position.z;

  double delta = 0.5*map.info.resolution;
  double resolution = map.info.resolution;

  // Note we do not clear the triangles here as we may wish
  // to load multiple occupancy grids
  std::vector<signed char>::const_iterator k = map.data.begin();
  for (unsigned int i = 0; i < map.info.height; i++)
    for (unsigned int j = 0; j < map.info.width; j++, ++k)
      {
        if (*k > occupied_threshold)
          {
            double center_x = x + j*resolution + delta;
            double center_y = y + i*resolution + delta;

            Point face_1_1(center_x - delta, center_y + delta, z);
            Point face_1_2(center_x - delta, center_y - delta, z);
            Point face_1_3(center_x - delta, center_y - delta, z + depth);
            Point face_1_4(center_x - delta, center_y + delta, z + depth);

            triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
            triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));

            Point face_2_1(center_x - delta, center_y - delta, z);
            Point face_2_2(center_x + delta, center_y - delta, z);
            Point face_2_3(center_x + delta, center_y - delta, z + depth);
            Point face_2_4(center_x - delta, center_y - delta, z + depth);

            triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
            triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));

            Point face_3_1(center_x + delta, center_y - delta, z);
            Point face_3_2(center_x + delta, center_y + delta, z);
            Point face_3_3(center_x + delta, center_y + delta, z + depth);
            Point face_3_4(center_x + delta, center_y - delta, z + depth);

            triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
            triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));

            Point face_4_1(center_x + delta, center_y + delta, z);
            Point face_4_2(center_x - delta, center_y + delta, z);
            Point face_4_3(center_x - delta, center_y + delta, z + depth);
            Point face_4_4(center_x + delta, center_y + delta, z + depth);

            triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
            triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));
          }
      }
  tree.rebuild(triangles.begin(), triangles.end());

  return;
}

int param_exit(const std::string& error)
{
  ROS_WARN("%s: Unable to find parameter: %s",
           ros::this_node::getName().c_str(), error.c_str());

  return -1;
}

int LaserSimulator::LoadLaserModel(const ros::NodeHandle& n)
{
  if (!n.hasParam("min_angle_deg")) return param_exit("min_angle_deg");
  if (!n.hasParam("max_angle_deg")) return param_exit("max_angle_deg");
  if (!n.hasParam("rate")) return param_exit("rate");
  if (!n.hasParam("scan_count")) return param_exit("scan_count");
  if (!n.hasParam("min_range")) return param_exit("min_range");
  if (!n.hasParam("max_range")) return param_exit("max_range");

  double minimum_angle_deg, maximum_angle_deg, scan_rate, scan_count;
  n.getParam("min_angle_deg", minimum_angle_deg);
  n.getParam("max_angle_deg", maximum_angle_deg);
  n.getParam("rate", scan_rate);
  n.getParam("scan_count", scan_count);

  scan_time = 1.0/scan_rate;
  time_increment = scan_time/scan_count;
  minimum_angle = minimum_angle_deg*M_PI/180;
  maximum_angle = maximum_angle_deg*M_PI/180;

  n.getParam("scan_count", number_of_ranges);
  assert(number_of_ranges > 0);
  angle_increment = (maximum_angle - minimum_angle)/double(number_of_ranges - 1);

  n.getParam("min_range", minimum_range);
  n.getParam("max_range", maximum_range);

  for (int i = 0; i < number_of_ranges; i++)
    scan_points.push_back(std::make_pair<float, float>(maximum_range*cos(minimum_angle + i*angle_increment),
                                                       maximum_range*sin(minimum_angle + i*angle_increment)));

  initialized = true;

  return 0;
}

int LaserSimulator::LoadDynamicModels(const ros::NodeHandle& n)
{
  std::map<std::string, boost::tuple<double, double, double> > types;

  if (n.hasParam("types"))
    {
      XmlRpc::XmlRpcValue t;

      n.getParam("types", t);

      if (t.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("types parameter needs to be an array");
          return -1;
        }
      else
        {
          if (t.size() == 0)
            {
              ROS_ERROR("no values in types array");
              return -1;
            }
          else
            {
              for (int i = 0; i < t.size(); i++)
                {
                  if (t[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    {
                      ROS_ERROR("types entry %d is not a structure, stopping", i);
                      return -1;
                    }
                  if (!t[i].hasMember("id"))
                    {
                      ROS_ERROR("types entry %d has no 'id' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("xdim"))
                    {
                      ROS_ERROR("models entry %d has no 'xdim' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("ydim"))
                    {
                      ROS_ERROR("types entry %d has no 'ydim' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("zdim"))
                    {
                      ROS_ERROR("types entry %d has no 'zdim' member", i);
                      return -1;
                    }

                  std::string id = (std::string)t[i]["id"];
                  double xdim = (double)t[i]["xdim"];
                  double ydim = (double)t[i]["ydim"];
                  double zdim = (double)t[i]["zdim"];

                  if (types.count(id) > 0)
                    {
                      ROS_ERROR("type %s is not unique", id.c_str());
                      return -1;
                    }

                  types.insert(std::make_pair<std::string,
                               boost::tuple<double, double, double> >
                               (id, boost::make_tuple(xdim, ydim, zdim)));
                }
            }
        }
    }
  else
    {
      ROS_ERROR("no types parameter provided");
      return -1;
    }

  if (n.hasParam("models"))
    {
      XmlRpc::XmlRpcValue m;

      n.getParam("models", m);
      if (m.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("models parameter needs to be an array");
          return -1;
        }
      else
        {
          if (m.size() == 0)
            {
              ROS_ERROR("no values in models array");
              return -1;
            }
          else
            {
              for (int i = 0; i < m.size(); i++)
                {
                  if (m[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    {
                      ROS_ERROR("models entry %d is not a structure, stopping", i);
                      return -1;
                    }
                  if (!m[i].hasMember("id"))
                    {
                      ROS_ERROR("models entry %d has no 'id' member", i);
                      return -1;
                    }
                  if (!m[i].hasMember("type"))
                    {
                      ROS_ERROR("models entry %d has no 'type' member", i);
                      return -1;
                    }

                  std::string id = (std::string)m[i]["id"];
                  std::string type = (std::string)m[i]["type"];

                  if (models.count(id) > 0)
                    {
                      ROS_ERROR("model %s is not unique", id.c_str());
                      return -1;
                    }

                  if (types.count(type) == 0)
                    {
                      ROS_ERROR("model %s has unknown type %s",
                                id.c_str(), type.c_str());
                      return -1;
                    }

                  models[id] = new Model(types[type].get<0>(),
                                         types[type].get<1>(),
                                         types[type].get<2>());
                }
            }
        }
    }
  else
    {
      ROS_ERROR("no models parameter provided");
      return -1;
    }

  return 0;
}

void LaserSimulator::GetScan(std::vector<float>& ranges)
{
  srand(time(NULL)); // initialize random number generator

  arma::colvec s(3);
  arma::colvec t(3);
  t(0) = pose.position.x + offset.position.x;
  t(1) = pose.position.y + offset.position.y;
  t(2) = pose.position.z + offset.position.z;

  double a = pose.orientation.w;
  double b = pose.orientation.x;
  double c = pose.orientation.y;
  double d = pose.orientation.z;

  arma::mat R(3, 3);
  R(0, 0) = a*a + b*b - c*c - d*d;
  R(0, 1) = 2.0*b*c - 2.0*a*d;
  R(0, 2) = 2.0*b*d + 2.0*a*c;
  R(1, 0) = 2.0*b*c + 2.0*a*d;
  R(1, 1) = a*a - b*b + c*c - d*d;
  R(1, 2) = 2.0*c*d - 2.0*a*b;
  R(2, 0) = 2.0*b*d - 2.0*a*c;
  R(2, 1) = 2.0*c*d + 2.0*a*b;
  R(2, 2) = a*a - b*b - c*c + d*d;

  arma::colvec p(3);
  p(2) = 0;

  arma::colvec pi(3);
  Point p1(t(0), t(1), t(2));
  unsigned int k = 0;
  std::list<Object_and_primitive_id> intersections;
  for (std::list<std::pair<float, float> >::iterator i = scan_points.begin();
       i != scan_points.end(); ++i, k++)
    {
      p(0) = (*i).first;
      p(1) = (*i).second;

      s = R*p + t;

      Point p2(s(0), s(1), s(2));

      Segment segment_query(p1, p2);
      double max_range = maximum_range;

      if (tree.do_intersect(segment_query))
        {
          intersections.clear();
          tree.all_intersections(segment_query,
                                 std::back_inserter(intersections));

          for (std::list<Object_and_primitive_id>::iterator j = intersections.begin();
               j != intersections.end(); ++j)
            {
              Point point;
              if (!CGAL::assign(point, (*j).first))
                continue;
              pi(0) = point.x();
              pi(1) = point.y();
              pi(2) = point.z();
              double range = arma::norm(pi - t, 2);
              if (range < max_range)
                max_range = range;
            }
        }

      if (!dynamic_tree.empty())
        {
          if (dynamic_tree.do_intersect(segment_query))
            {
              intersections.clear();
              dynamic_tree.all_intersections(segment_query,
                                             std::back_inserter(intersections));

              for (std::list<Object_and_primitive_id>::iterator j =
                     intersections.begin(); j != intersections.end(); ++j)
                {
                  Point point;
                  if (!CGAL::assign(point, (*j).first))
                    continue;
                  pi(0) = point.x();
                  pi(1) = point.y();
                  pi(2) = point.z();
                  double range = arma::norm(pi - t, 2);
                  if (range < max_range)
                    max_range = range;
                }
            }
        }


      if (noise_sd > 0.0 && max_range < maximum_range)
        {
          double U = rand() / double(RAND_MAX);
          double V = rand() / double(RAND_MAX);
          // Box-Muller method
          double sn_var = sqrt (-2.0*log(U)) * cos(2.0*M_PI*V);
          ranges[k] = max_range + noise_sd * sn_var;
        }
      else
        {
          ranges[k] = max_range;
        }
    }

  return;
}

void LaserSimulator::UpdatePoseArray(const laser_simulator::PoseStampedNamedArray& pose_array)
{
  std::list<Triangle> dynamic_triangles;

  for (unsigned int i = 0; i < pose_array.poses.size(); ++i)
  {
    if (pose_array.poses[i].child_frame_id == frame_id)
      continue;

    if (models.count(pose_array.poses[i].child_frame_id) == 0)
      {
        printf("%s: Unknown model in odom_array message (%s) - skipping\n",
               ros::this_node::getName().c_str(),
               pose_array.poses[i].child_frame_id.c_str());
        continue;
      }

    double x_delta = 0.5*models[pose_array.poses[i].child_frame_id]->xdim;
    double y_delta = 0.5*models[pose_array.poses[i].child_frame_id]->ydim;
    double z_delta = 0.5*models[pose_array.poses[i].child_frame_id]->zdim;

    double center_x = pose_array.poses[i].pose.position.x;
    double center_y = pose_array.poses[i].pose.position.y;
    double center_z = pose_array.poses[i].pose.position.z;

    double dxm = center_x - x_delta;
    double dxp = center_x + x_delta;

    double dym = center_y - y_delta;
    double dyp = center_y + y_delta;

    double dzm = center_z - z_delta;
    double dzp = center_z + z_delta;

    Point face_1_1(dxm, dyp, dzm);
    Point face_1_2(dxm, dym, dzm);
    Point face_1_3(dxm, dym, dzp);
    Point face_1_4(dxm, dyp, dzp);

    dynamic_triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
    dynamic_triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));

    Point face_2_1(dxm, dym, dzm);
    Point face_2_2(dxp, dym, dzm);
    Point face_2_3(dxp, dym, dzp);
    Point face_2_4(dxm, dym, dzp);

    dynamic_triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
    dynamic_triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));

    Point face_3_1(dxp, dym, dzm);
    Point face_3_2(dxp, dyp, dzm);
    Point face_3_3(dxp, dyp, dzp);
    Point face_3_4(dxp, dym, dzp);

    dynamic_triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
    dynamic_triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));

    Point face_4_1(dxp, dyp, dzm);
    Point face_4_2(dxm, dyp, dzm);
    Point face_4_3(dxm, dyp, dzp);
    Point face_4_4(dxp, dyp, dzp);

    dynamic_triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
    dynamic_triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));
  }

  dynamic_tree.clear();
  if (dynamic_triangles.size() > 0)
    dynamic_tree.rebuild(dynamic_triangles.begin(), dynamic_triangles.end());
}
