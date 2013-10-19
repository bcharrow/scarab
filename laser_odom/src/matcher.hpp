#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

namespace mrsl {

template<int D>
class Gaussian {
public:
  typedef Eigen::Matrix<double, D, 1> VectorD;
  typedef Eigen::Matrix<double, D, D> MatrixD;

  Gaussian(const VectorD &mean, const MatrixD &cov) : mean_(mean), cov_(cov) {

  }

  const MatrixD& cov() const { return cov_; }
  const VectorD& mean() const { return mean_; }

private:
  Eigen::Matrix<double, D, 1> mean_;
  Eigen::Matrix<double, D, D> cov_;
};

typedef Gaussian<3> Gaussian3d;

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> RowMatrix2d;

class GridMap {
public:
  GridMap(double minx, double maxx, double miny, double maxy,
          double meters_per_pixel);

  ~GridMap() {
    delete[] grid_;
  }

  double metersPerPixel() const { return meters_per_pixel_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double minX() const { return min_x_; }
  double minY() const { return min_y_; }

  void getCoord(int xi, int yi, double *x, double *y) const {
    *x = (xi + 0.5) * meters_per_pixel_ + min_x_;
    *y = (yi + 0.5) * meters_per_pixel_ + min_y_;
  }

  void getSubscript(double x, double y, int *xi, int *yi) const {
    *xi = (x - min_x_) / meters_per_pixel_;
    *yi = (y - min_y_) / meters_per_pixel_;
  }

  bool valid(int xi, int yi) const {
    return !(xi >= width_ || yi >= height_ || xi < 0 || yi < 0);
  }

  uint8_t get(int xi, int yi) const {
    if (valid(xi, yi)) {
      int ind = yi * width_ + xi;
      return grid_[ind];
    } else {
      return 0;
    }
  }

  void set(int xi, int yi, uint8_t val) {
    if (valid(xi, yi)) {
      int ind = yi * width_ + xi;
      grid_[ind] = val;
    } else {
      ROS_WARN("Setting coordinates outside map");
    }
  }

  void inc(int xi, int yi, uint8_t val) {
    if (valid(xi, yi)) {
      int ind = yi * width_ + xi;
      grid_[ind] += val;
    } else {
      ROS_WARN("Incrementing coordinates outside map");
    }
  }

  void fill(uint8_t val) {
    for (int i = 0; i < width_ * height_; ++i) {
      grid_[i] = val;
    }
  }

  // Get likelihood of points for various translations after a rotation.
  //
  // Points are x-shifted in pixel coordinates by (delta_xi + i) where i is
  // in the half open interval [0, num_x).  Similar for y.
  //
  // scores(xi, yi) = Likelihood points after x-shifted by (delta_xi + xi) *
  // metersPerPixel() and y-shifted by (delta_yi + yi) * metersPerPixel()
  void scores2D(const RowMatrix2d &points,
                int delta_xi, int num_x,
                int delta_yi, int num_y,
                double theta, Eigen::ArrayXXi *scores) const;

  // Evaluate scores2D with several different angles
  //
  // scores[i] scores2d() with theta = delta_t + i * inc_t
  void scores3D(const RowMatrix2d &points,
                int delta_xi, int num_x, int int_y, int num_yi,
                double delta_t, int num_t, double inc_t,
                std::vector<Eigen::ArrayXXi> *scores) const;

private:
  GridMap() {};
  GridMap(const GridMap &map);
  void operator=(const GridMap&);

  // Lower left coordinate of map in meters
  double min_x_, min_y_;
  double meters_per_pixel_;
  // Size of grid_ in pixels
  int width_, height_;
  // Probability of occupancy; 0 means 0 probability, 255 means 1.0
  uint8_t *grid_;
};

void projectScan(const sensor_msgs::LaserScan &scan, RowMatrix2d *points);
void transformPoints(const Eigen::Vector3d& xyt, const RowMatrix2d &local,
                     RowMatrix2d *points);

class ScanMatcher {
public:
  struct Params {
    Params() : grid_res(0.02), sensor_sd(0.1) {}
    double grid_res;
    double sensor_sd;
  };

  struct SearchWindow {
    SearchWindow() : range_x(0.4),
                     range_y(0.4),
                     range_t(angles::from_degrees(20)),
                     inc_t(angles::from_degrees(1)) {}
    SearchWindow makeAligned(double res) const {
      SearchWindow w;
      w.range_x = round(range_x / res) * res;
      w.range_y = round(range_y / res) * res;
      w.range_t = round(range_t / inc_t) * inc_t;
      w.inc_t = inc_t;
      return w;
    }

    double range_x;
    double range_y;
    double range_t;
    double inc_t;
  };

  ScanMatcher(const Params &p);
  ~ScanMatcher();

  void setScan(const Eigen::Vector3d &pose_xyt,
               const sensor_msgs::LaserScan &scan);
  void setMap(GridMap *map);

  const GridMap& map() const { return *map_; }
  Gaussian3d matchScan(const Eigen::Vector3d &pose_xyt,
                       const sensor_msgs::LaserScan &scan,
                       const SearchWindow &window);
  Gaussian3d match(const RowMatrix2d &points, const SearchWindow &win);

private:
  Params p_;
  boost::scoped_ptr<GridMap> map_;
};

// Class to visualize scan matches in rviz.
class MatchVisualizer {
public:
  MatchVisualizer();
  void visualize(const sensor_msgs::LaserScan &scan1,
                 const sensor_msgs::LaserScan &scan2,
                 const Eigen::Vector3d &match_transform);
private:
  visualization_msgs::Marker scan1_, scan2_;
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  tf::TransformBroadcaster broadcaster_;
};

};
