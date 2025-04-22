#ifndef COMMON_H
#define COMMON_H

#include <random>
#include <Eigen/Dense>


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

#endif // COMMON_H