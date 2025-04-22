
#include "sample/plane.h"

namespace dator
{
namespace sample
{
std::vector<Plane> samplePlanesThroughLine(
    const Eigen::Vector3d &point_on_line,
    const Eigen::Vector3d &line_direction,
    int num_samples, double start_angle)
{
  std::vector<Plane> planes;
  Eigen::Vector3d d = line_direction.normalized();

  // Step 1: find orthonormal basis {n1, n2} perpendicular to d
  Eigen::Vector3d temp = (std::abs(d.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  Eigen::Vector3d n1   = (temp - temp.dot(d) * d).normalized(); // orthogonal to d
  Eigen::Vector3d n2   = d.cross(n1); // completes orthonormal basis

  // Step 2: sample normal vectors
  for (int i = 0; i < num_samples; ++i)
  {
    double theta           = 2.0 * M_PI * i / num_samples + start_angle; // unique parameter
    Eigen::Vector3d normal = std::cos(theta) * n1 + std::sin(theta) * n2;
    double offset          = normal.dot(point_on_line); // d = n · p0

    planes.push_back({normal, offset, theta});
  }

  return planes;
}
} // namespace sample
} // namespace dator