#ifndef RAY_H
#define RAY_H

#include "common.h"

std::vector<Ray> sampleRadialLines(const Eigen::Vector3d &origin, const Eigen::Vector3d &normal, int N);

#endif // RAY_H