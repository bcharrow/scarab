#include "matcher.hpp"

#include <queue>

using namespace std;
using namespace Eigen;
using namespace mrsl;

GridMap::GridMap(double minx, double maxx, double miny, double maxy,
                 double meters_per_pixel)
  : min_x_(minx), min_y_(miny), meters_per_pixel_(meters_per_pixel) {
  width_ = ceil((maxx - minx) / meters_per_pixel_);
  height_ = ceil((maxy - miny) / meters_per_pixel_);
  grid_ = new uint8_t[width_ * height_];
}

void GridMap::scores3D(const RowMatrix2d &points,
                       int delta_xi, int num_x,
                       int delta_yi, int num_y,
                       double delta_t, int num_t, double inc_t,
                       vector<ArrayXXi> *scores) const {
  // Iterate over theta and get scores for each point
  scores->resize(num_t);
  for (int t = 0; t < num_t; ++t) {
    double theta = delta_t + t * inc_t;
    scores2D(points, delta_xi, num_x, delta_yi, num_y, theta, &scores->at(t));
  }
}

void GridMap::scores2D(const RowMatrix2d &points,
                       int delta_xi, int num_x,
                       int delta_yi, int num_y,
                       double theta, ArrayXXi *scores_ptr) const {
  ArrayXXi &scores = *scores_ptr;
  scores.resize(num_x, num_y);
  scores.setZero();

  double ct = cos(theta), st = sin(theta);
  for (int i = 0; i < points.cols(); ++i) {
    // Rotate point, convert it to pixel coordinates and *then* translate it.
    // If you translate in world coordinates rounding can do weird things.
    const Eigen::Vector2d &point = points.col(i);
    double x = ct * point(0) - st * point(1);
    double y = st * point(0) + ct * point(1);
    int xi0, yi0;
    // Get grid coordinates of point
    getSubscript(x, y, &xi0, &yi0);

    // Shift by initial offset
    xi0 += delta_xi;
    yi0 += delta_yi;

    // Iterate over translation of point and update scores
    for (int dyi = 0; dyi < num_y; ++dyi) {
      int yi = yi0 + dyi;
      for (int dxi = 0; dxi < num_x; ++dxi) {
        int xi = xi0 + dxi;
        int ll = static_cast<int>(get(xi, yi));
        scores(dxi, dyi) += ll;
      }
    }    
  }
}

ScanMatcher::ScanMatcher(const Params &p)
  : p_(p), map_(NULL) {

}

ScanMatcher::~ScanMatcher() {
}


void mrsl::projectScan(const sensor_msgs::LaserScan &scan, RowMatrix2d *points) {
  // Project to robot frame
  points->resize(2, scan.ranges.size());
  double theta = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    points->operator()(0, i) = cos(theta) * scan.ranges[i];
    points->operator()(1, i) = sin(theta) * scan.ranges[i];
    theta += scan.angle_increment;
  }
  if (abs(theta - scan.angle_increment - scan.angle_max) > 1e-5) {
    ROS_ERROR("Bad angle: %f %f", theta-scan.angle_increment, scan.angle_max);
    ROS_BREAK();
  }
}

void mrsl::transformPoints(const Vector3d& xyt, const RowMatrix2d &local,
                     RowMatrix2d *points) {
  double ct = cos(xyt(2)), st = sin(xyt(2));
  points->resize(2, local.cols());
  for (int i = 0; i < local.cols(); ++i) {
    points->operator()(0, i) = ct * local(0, i) - st * local(1, i) + xyt(0);
    points->operator()(1, i) = st * local(0, i) + ct * local(1, i) + xyt(1);
  }
}

void ScanMatcher::setScan(const Vector3d &pose,
                          const sensor_msgs::LaserScan &scan) {
  // Project the scan
  RowMatrix2d local_points, points;
  projectScan(scan, &local_points);
  transformPoints(pose, local_points, &points);

  double min_x = points.row(0).minCoeff();
  double min_y = points.row(1).minCoeff();
  double max_x = points.row(0).maxCoeff();
  double max_y = points.row(1).maxCoeff();

  // Create map
  // TODO: Allow several scans to comprise map
  GridMap *newmap
    = new GridMap(min_x - 1, max_x + 1, min_y - 1, max_y + 1, p_.grid_res);
  newmap->fill(0);

  // TODO: Lookup table
  // Project range measurement and update likelihood of neighboring points.
  // Update probabilities in 3 sigma radius around point
  const int max_offset = ceil(p_.sensor_sd / p_.grid_res) * 3;
  // Don't normalize by standard deviation; all values are scaled equally and
  // this results in a better spread
  const double normalizer = 1.0;
  const double exp_factor = (p_.grid_res * p_.grid_res) /
    (2 * p_.sensor_sd * p_.sensor_sd);

  for (int i = 0; i < points.cols(); ++i) {
    int xi, yi;
    newmap->getSubscript(points(0, i), points(1, i), &xi, &yi);
    for (int delta_y = -max_offset; delta_y <= max_offset; ++delta_y) {
      for (int delta_x = -max_offset; delta_x <= max_offset; ++delta_x) {
        int dist = delta_x * delta_x + delta_y * delta_y;
        double prob = normalizer * exp(-dist * exp_factor);
        uint8_t val = static_cast<uint8_t>(prob * 255.0);
        uint8_t orig = newmap->get(xi + delta_x, yi + delta_y);
        newmap->set(xi + delta_x, yi + delta_y, max(val, orig));
      }
    }
  }
  setMap(newmap);
}

void ScanMatcher::setMap(GridMap *map) {
  if (map->metersPerPixel() != p_.grid_res) {
    throw invalid_argument("Non-matching resolution");
  }
  map_.reset(map);
}

Gaussian3d ScanMatcher::matchScan(const Eigen::Vector3d &pose_xyt,
                                     const sensor_msgs::LaserScan &scan,
                                     const SearchWindow &window) {
  RowMatrix2d local_points, points;
  projectScan(scan, &local_points);
  transformPoints(pose_xyt, local_points, &points);
  // Project laser scan points
  return match(points, window);
}

Gaussian3d ScanMatcher::match(const RowMatrix2d &points,
                              const SearchWindow &unaligned_win) {
  static int it = 0;
  // Align search window with map grid
  double res = map_->metersPerPixel();
  SearchWindow win = unaligned_win.makeAligned(res);
  int sx = round(win.range_x / res);
  int sy = round(win.range_y / res);
  int num_x = 2 * sx + 1;
  int num_y = 2 * sy + 1;
  int num_t = 2 * round(win.range_t / win.inc_t) + 1;

  vector<ArrayXXi> scores;
  map_->scores3D(points, -sx, num_x, -sy, num_y,
                 -win.range_t, num_t, win.inc_t, &scores);

  // Process scores to get best guess
  Vector3i inds = Vector3i::Zero();
  int max_likeli = std::numeric_limits<int>::min();
  for (size_t ti = 0; ti < scores.size(); ++ti) {
    ArrayXXi &dxdy_scores = scores.at(ti);
    for (int xi = 0; xi < dxdy_scores.rows(); ++xi) {
      for (int yi = 0; yi < dxdy_scores.cols(); ++yi) {
        int score = dxdy_scores(xi, yi);
        ROS_DEBUG_NAMED("costsurface", "%i %f %f %f %i",
                        it,
                        -win.range_x + xi * res,
                        -win.range_y + yi * res,
                        -win.range_t + ti * win.inc_t,
                        score);
        if (score > max_likeli) {
          inds(0) = xi;
          inds(1) = yi;
          inds(2) = ti;
          max_likeli = score;
        }
      }
    }
  }

  Vector3d transform(-win.range_x + inds(0) * res,
                     -win.range_y + inds(1) * res,
                     -win.range_t + inds(2) * win.inc_t);
  ++it;
  return Gaussian3d(transform, Matrix3d::Identity());

}

MatchVisualizer::MatchVisualizer() {
      marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/scan_match", 1, true);

      visualization_msgs::Marker arr;
      arr.action = visualization_msgs::Marker::ADD;
      arr.type = visualization_msgs::Marker::CUBE_LIST;
      arr.header.stamp = ros::Time();
      arr.lifetime = ros::Duration(40.0);
      arr.scale.x = 0.1;
      arr.scale.y = 0.1;
      arr.scale.z = 0.1;

      scan1_ = arr;
      scan1_.id = 0;
      scan1_.header.frame_id ="/debug_laser";
      scan1_.ns = "laser_scan1";
      scan1_.color.a = 1.0;
      scan1_.color.r = 1.0;

      scan2_ = arr;
      scan2_.id = 0;
      scan2_.header.frame_id ="/debug_laser";
      scan2_.ns = "laser_scan2";
      scan2_.color.a = 1.0;
      scan2_.color.b = 1.0;
    }

void MatchVisualizer::visualize(const sensor_msgs::LaserScan &scan1,
                                const sensor_msgs::LaserScan &scan2,
                                const Vector3d &match_transform) {
  RowMatrix2d scan1_xy, scan2_local_points, scan2_xy;
  projectScan(scan1, &scan1_xy);

  projectScan(scan2, &scan2_local_points);
  Eigen::Vector3d pose_xyt;

  pose_xyt = match_transform;
  transformPoints(pose_xyt, scan2_local_points, &scan2_xy);

  scan1_.points.resize(scan1_xy.cols());
  for (int i = 0; i < scan1_xy.cols(); ++i) {
    scan1_.points[i].x = scan1_xy(0, i);
    scan1_.points[i].y = scan1_xy(1, i);
  }

  scan2_.points.resize(scan2_xy.cols());
  for (int i = 0; i < scan2_xy.cols(); ++i) {
    scan2_.points[i].x = scan2_xy(0, i);
    scan2_.points[i].y = scan2_xy(1, i);
  }

  visualization_msgs::MarkerArray arr;
  arr.markers.push_back(scan1_);
  arr.markers.push_back(scan2_);
  marker_pub_.publish(arr);

  ros::Time ts = ros::Time::now();

  tf::Transform tform;
  tform.setOrigin(tf::Vector3(pose_xyt(0), pose_xyt(1), 0.));
  tform.setRotation(tf::createQuaternionFromYaw(pose_xyt(2)));
  broadcaster_.sendTransform(tf::StampedTransform(tform, ts,
                                                  "/debug_laser",
                                                  "/debug_transform"));
}
