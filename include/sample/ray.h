#ifndef RAY_H
#define RAY_H

#include "common.h"

namespace dator
{
namespace sample
{
std::vector<Ray> sampleRadialLines(const Eigen::Vector3d &origin, const Eigen::Vector3d &normal, int N);
}
} // namespace dator

#endif // RAY_H