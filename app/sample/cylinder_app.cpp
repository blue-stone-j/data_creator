#include <iostream>
#include <fstream>
#include <iomanip>
#include "sample/cylinder.h"
#include "sample/plane.h"
#include "sample/ray.h"

#include "save.h"

int main(int argc, char **argv)
{
  std::vector<Eigen::Vector3d> points;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3d cyl_center(5, -1, 0);
  Eigen::Vector3d cyl_axis(0.05, -0.2, 1);
  cyl_axis.normalize();
  double cyl_radius = 0.15;

  // Define the line: point + direction
  Eigen::Vector3d point_on_line(0.0, 0.0, 0.0);
  Eigen::Vector3d line_direction(1.0, 0.0, 0.0); // x-axis

  // for (int i = 0; i < 10; ++i)
  {
    // double start_angle = 0.1 * i;
    double start_angle = 0.1;
    int N              = 30; // number of sampled planes
    auto planes        = samplePlanesThroughLine(point_on_line, line_direction, N, start_angle);

    std::cout << std::fixed << std::setprecision(5);
    // std::random_device rd;
    // std::mt19937 rng(rd());
    // std::uniform_real_distribution<> dist(-0.05, 0.05);

    for (int i = 0; i < N; ++i)
    {
      const auto &plane = planes[i];
      std::cout << "Plane " << i << " (theta = " << plane.theta
                << " rad): " << plane.normal.transpose()
                << " · (x, y, z) = " << plane.d << "\n";

      auto points_t = sampleCylinderCrossSectionPoints(point_on_line, plane.normal, cyl_center, cyl_axis, cyl_radius, 1000);

      for (auto &point : points_t)
      {
        if (point.z() < -3 || point.z() > 2)
        { continue; }
        points.emplace_back(point);
      }
    }
    // points.insert(points.end(), points_t.begin(), points_t.end());
  }

  savePointsToFile(points, "cylinder.txt");
  savePointCloudToPCD(points, "cylinder.pcd");

  float x = std::atan(cyl_axis.x() / cyl_axis.z()) * 180 / M_PI;
  float y = std::atan(cyl_axis.y() / cyl_axis.z()) * 180 / M_PI;
  std::cout << x << ", " << y << std::endl;

  std::cout << cyl_axis.x() << " " << cyl_axis.y() << " " << cyl_axis.z() << " " << cyl_center.x() << " " << cyl_center.y() << " " << cyl_center.z() << std::endl;

  return 0;
}
