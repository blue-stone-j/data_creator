#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

namespace dator
{
struct Point3D
{
  double x, y, z;
};

struct Ray
{
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};

struct Plane
{
  Eigen::Vector3d normal; // normal vector (a, b, c)
  double d; // plane offset
  double theta; // angle (in radians) as the unique parameter
};
} // namespace dator

#endif // COMMON_H