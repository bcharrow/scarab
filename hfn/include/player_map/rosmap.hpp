#ifndef ROSMAP_HPP
#define ROSMAP_HPP

#include <vector>
#include <set>

#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/OccupancyGrid.h>

#include "player_map/map.h"
#include <Eigen/StdVector>
namespace scarab {

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Path;

double pathLength(const scarab::Path &path);

class OccupancyMap {
public:
  OccupancyMap();
  ~OccupancyMap();

  static OccupancyMap* FromMapServer(const char *srv_name,
    const int free_threshold=0, const int occupied_threshold=100);

  void setMap(map_t *map);
  void setMap(const nav_msgs::OccupancyGrid &grid);
  void updateCSpace(double max_occ_dist, double lethal_occ_dist,
                    double cost_occ_prob = 0.0, double cost_occ_dist = 0.0);

  nav_msgs::OccupancyGrid getCSpace();
  nav_msgs::OccupancyGrid getCostMap();

  double minX();
  double minY();
  double maxX();
  double maxY();

  double lethalOccDist() const { return lethal_occ_dist_; }
  double maxOccDist() const { return max_occ_dist_; }

  const map_cell_t* getCell(double x, double y) const;

  // True if cell is free and far away from obstacles
  bool safePoint(double x, double y) const; // Use lethalOccDist()
  bool safePoint(double x, double y, double occ_dist) const;

  bool lineOfSight(double x1, double y1, double x2, double y2,
                   double max_occ_dist = 0.0, bool allow_unknown = false) const;
  Path astar(double x1, double y1, double x2, double y2,
             double max_occ_dist = 0.0, bool allow_unknown = false);
  bool nearestPoint(double x, double y, double max_occ_dist,
                    double *out_x, double *out_y) const;
  // TODO: Unify these two APIs

  // Get a list of endpoints
  const Path& prepareShortestPaths(double x, double y, double min_distance,
                                   double max_distance, double max_occ_dist,
                                   bool allow_unknown = false);
  // Get the path whose endpoint is ind from last call to prepareShortestPaths()
  Path buildShortestPath(int ind);

  // Calculate single source shortest paths to all endpoints
  // Returns a vector, element at index i is true if path from dests[i] exists
  // Use buildShortestPath to construct path; index matches point in dests
  void prepareAllShortestPaths(double x, double y, double max_occ_dist,
                               bool allow_unknown = false);
  // Get shortest path
  Path shortestPath(double x, double y);

  void setThresholds(int free, int occ);
  void setCostFactors(double occ_prob, double occ_dist);

private:
  struct Node {
    Node() {}
    Node(const std::pair<int, int> &c, float d, float h) :
      coord(c), true_cost(d), heuristic(h) { }
    std::pair<int, int> coord;
    float true_cost;
    float heuristic;
  };

  struct NodeCompare {
    bool operator()(const Node &lnode, const Node &rnode) {
      return make_pair(lnode.heuristic, lnode.coord) <
        make_pair(rnode.heuristic, rnode.coord);
    }
  };

  void initializeSearch(double startx, double starty);
  bool nextNode(double max_occ_dist, Node *curr_node, bool allow_unknown);
  void addNeighbors(const Node &node, double max_occ_dist, bool allow_unknown);
  void buildPath(int i, int j, Path *path);

  map_t *map_;
  int ncells_;
  int starti_, startj_;
  int stopi_, stopj_;
  int max_free_threshold_, min_occupied_threshold_;
  double max_occ_dist_, lethal_occ_dist_;
  boost::scoped_array<float> costs_;
  boost::scoped_array<int> prev_i_;
  boost::scoped_array<int> prev_j_;
  // Priority queue mapping cost to index
  boost::scoped_ptr<std::set<Node, NodeCompare> > Q_;
  Path endpoints_;
};

} // end namespace scarab
#endif
