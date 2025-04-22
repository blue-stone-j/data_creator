#ifndef PLANE_H
#define PLANE_H

#include "common.h"

namespace dator
{
namespace sample
{
std::vector<Plane> samplePlanesThroughLine(const Eigen::Vector3d &point_on_line,
                                           const Eigen::Vector3d &line_direction,
                                           int num_samples, double start_angle = 0.0);
}
} // namespace dator

#endif // PLANE_H