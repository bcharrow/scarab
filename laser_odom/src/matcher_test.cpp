#include <gtest/gtest.h>

#include "matcher.hpp"

using namespace mrsl;
using namespace std;
using namespace Eigen;

void printMap(const GridMap &map) {
  static int id = 0;
  stringstream ss;
  for (int i = 0; i < map.height(); ++i) {
    for (int j = 0; j < map.width(); ++j) {
      ss << static_cast<int>(map.get(j, i)) << " ";
    }
  }
  ROS_DEBUG_NAMED("map", "%i %i %i %s",
                  id, map.width(), map.height(), ss.str().c_str());
  ++id;
}

TEST(Matcher, alignment) {
  ScanMatcher::Params p;
  p.grid_res = 0.1;
  p.sensor_sd = 1.0;
  ScanMatcher matcher(p);

  RowMatrix2d points = RowMatrix2d::Zero(2, 2);
  points(0, 0) = 0.5;
  points(1, 0) = 0.15;

  points(0, 1) = 0.5;
  points(1, 1) = 0.5;

  // points(0, 2) = 0.6;
  // points(1, 2) = 0.2;

  GridMap *gm = new GridMap(-2.0, 2.0, -2.0, 2.0, p.grid_res);
  gm->fill(0);
  
  for (int i = 0; i < points.cols(); ++i) {
    int xi, yi;
    gm->getSubscript(points(0, i), points(1, i), &xi, &yi);
    gm->getCoord(xi, yi, &points(0, i), &points(1, i));
    gm->set(xi, yi, 40 - i);
  }
  matcher.setMap(gm);

  printMap(matcher.map());
  
  RowMatrix2d shift = points;
  // shift.row(0) += RowVectorXd::Constant(points.cols(), 3 * p.grid_res);
  // shift.row(1) += RowVectorXd::Constant(points.cols(), 2 * p.grid_res);
  ROS_INFO_STREAM("Points: "  << points.transpose());
  ROS_INFO_STREAM("Shift: "  << shift.transpose());
  ScanMatcher::SearchWindow wind;
  wind.range_x = 0.1;
  wind.range_y = 0.1;
  wind.range_t = 0.1;  
  wind.inc_t = 0.1;  

  ROS_WARN("RAW");
  matcher.match(shift, wind);
}

int main(int argc, char **argv){
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
