#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "sample/cylinder.h"


std::vector<Point3D> generateCylinderPoints(double radius, double height, int num_circumference, int num_height)
{
  std::vector<Point3D> points;
  double dTheta = 2.0 * M_PI / num_circumference; // Angle step size
  double dz     = height / (num_height - 1); // Height step size

  for (int i = 0; i < num_circumference; ++i)
  {
    double theta = i * dTheta;
    double x     = radius * std::cos(theta);
    double y     = radius * std::sin(theta);

    for (int j = 0; j < num_height; ++j)
    {
      double z = j * dz;
      points.push_back({x, y, z});
    }
  }

  return points;
}

std::optional<Eigen::Vector3d> intersectRayWithCylinder(
    const Eigen::Vector3d &ray_origin,
    const Eigen::Vector3d &ray_dir, // must be normalized
    const Eigen::Vector3d &cyl_center,
    const Eigen::Vector3d &cyl_axis, // must be normalized
    double radius)
{
  Eigen::Vector3d w = ray_origin - cyl_center;

  Eigen::Vector3d d_perp = ray_dir - ray_dir.dot(cyl_axis) * cyl_axis;
  Eigen::Vector3d w_perp = w - w.dot(cyl_axis) * cyl_axis;

  double A = d_perp.squaredNorm();
  double B = 2.0 * d_perp.dot(w_perp);
  double C = w_perp.squaredNorm() - radius * radius;

  double discriminant = B * B - 4 * A * C;

  if (discriminant < 0 || A < 1e-8) return std::nullopt; // no intersection or degenerate case

  double sqrtD = std::sqrt(discriminant);
  double t1    = (-B - sqrtD) / (2 * A);
  double t2    = (-B + sqrtD) / (2 * A);

  double t = (t1 >= 0) ? t1 : (t2 >= 0) ? t2 : -1.0;
  if (t < 0) return std::nullopt; // both intersections behind the ray

  return ray_origin + t * ray_dir;
}


std::vector<Eigen::Vector3d> sampleCylinderCrossSectionPoints(
    const Eigen::Vector3d &ray_origin, // center of radial rays (on the plane)
    const Eigen::Vector3d &plane_normal, // normal of plane
    const Eigen::Vector3d &cyl_center, // any point on cylinder axis
    const Eigen::Vector3d &cyl_axis, // cylinder axis (not perpendicular to plane)
    double cyl_radius,
    int N // number of samples
)
{
  std::random_device rd;
  std::mt19937 rng(rd());
  // std::uniform_real_distribution<> dist(-0.02, 0.02);
  std::normal_distribution<> dist(0, 0.02);

  std::vector<Eigen::Vector3d> points;

  // Step 1: basis of plane
  Eigen::Vector3d u;
  if (std::abs(plane_normal.z()) < 1.0)
  {
    u = plane_normal.cross(Eigen::Vector3d::UnitZ()).normalized();
  }
  else
  {
    u = plane_normal.cross(Eigen::Vector3d::UnitY()).normalized();
  }
  Eigen::Vector3d v = plane_normal.cross(u);

  // Step 2: normalized axis
  Eigen::Vector3d axis = cyl_axis.normalized();

  for (int i = 0; i < N; ++i)
  {
    double theta        = 2 * M_PI * i / N;
    Eigen::Vector3d dir = std::cos(theta) * u + std::sin(theta) * v;

    // Ray-cylinder intersection
    Eigen::Vector3d w      = ray_origin - cyl_center;
    Eigen::Vector3d d_perp = dir - dir.dot(axis) * axis;
    Eigen::Vector3d w_perp = w - w.dot(axis) * axis;

    double A = d_perp.squaredNorm();
    double B = 2.0 * d_perp.dot(w_perp);
    double C = w_perp.squaredNorm() - cyl_radius * cyl_radius;

    double D = B * B - 4 * A * C;
    if (D < 0 || A < 1e-8) continue; // no intersection

    double sqrtD = std::sqrt(D);
    double t1    = (-B - sqrtD) / (2 * A);
    double t2    = (-B + sqrtD) / (2 * A);

    double t = (t1 >= 0) ? t1 : (t2 >= 0) ? t2 : -1.0;
    if (t >= 0)
    {
      t += dist(rng);
      Eigen::Vector3d p = ray_origin + t * dir;
      points.push_back(p);
    }
  }

  return points;
}