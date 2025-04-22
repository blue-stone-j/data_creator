#ifndef CYLINDER_H
#define CYLINDER_H

#include "common.h"

// generate points on a cylinder
std::vector<Point3D> generateCylinderPoints(double radius, double height, int num_circumference, int num_height);

// generate intersection of ray and cylinder
std::optional<Eigen::Vector3d> intersectRayWithCylinder(
    const Eigen::Vector3d &ray_origin,
    const Eigen::Vector3d &ray_dir, // must be normalized
    const Eigen::Vector3d &cyl_center,
    const Eigen::Vector3d &cyl_axis, // must be normalized
    double radius);

// generate intersection points of plane and cylinder
std::vector<Eigen::Vector3d> sampleCylinderCrossSectionPoints(
    const Eigen::Vector3d &ray_origin, // center of radial rays (on the plane)
    const Eigen::Vector3d &plane_normal, // normal of plane
    const Eigen::Vector3d &cyl_center, // any point on cylinder axis
    const Eigen::Vector3d &cyl_axis, // cylinder axis (not perpendicular to plane)
    double cyl_radius,
    int N // number of samples
);
#endif