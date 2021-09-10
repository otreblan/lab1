#pragma once

#include "SpatialBase.h"

#include <vector>
#include <limits>

namespace utec {
namespace spatial {

/**
 * BasicSpatial implementation
 */
template <typename Point>
class BasicSpatial : public SpatialBase<Point> {
 private:
  std::vector<Point> v;

 public:
  BasicSpatial() {};

  // O(1)
  void insert(const Point& new_point) override
  {
    v.push_back(new_point);
  }

  // El punto de referencia no necesariamente es parte del dataset
  // O(n)
  Point nearest_neighbor(const Point& reference) override
  {
    Point nearest = reference;
    double min_distance = std::numeric_limits<double>::infinity();

    for(const auto& p: v)
    {
      double distance = reference.distance(p);
      if(distance < min_distance)
      {
        min_distance = distance;
        nearest = p;
      }
    }

    return nearest;
  }
};

}  // namespace spatial
}  // namespace utec

// vim: set et ts=2 sw=2:
