#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "sample/cylinder.h"
#include "sample/plane.h"
#include "sample/ray.h"

#include "save.h"


TEST(cylinder, cylinder)
{
  // Cylinder parameters
  double radius         = 1.0;
  double height         = 2.0;
  int num_circumference = 100; // Number of points around the circle
  int num_height        = 50; // Number of points along the height

  // Generate cylinder surface points
  std::vector<dator::Point3D> points = dator::sample::generateCylinderPoints(radius, height, num_circumference, num_height);

  // Save points to a file
  savePointsToFile(points, "cylinder_points.txt");
}

TEST(cylinder, ray)
{
  std::ofstream ofs("cylinder_ray.ply");
  std::vector<Eigen::Vector3d> points;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Define the line: point + direction
  Eigen::Vector3d point_on_line(0.0, 0.0, 0.0);
  Eigen::Vector3d line_direction(1.0, 0.0, 0.0); // x-axis
  int N       = 30; // sample 8 planes
  auto planes = dator::sample::samplePlanesThroughLine(point_on_line, line_direction, N);

  Eigen::Vector3d cyl_center(1, 1, 0);
  Eigen::Vector3d cyl_axis(0, 0, 1);
  double cyl_radius = 0.15;

  std::cout << std::fixed << std::setprecision(3);
  for (int i = 0; i < N; ++i)
  {
    const auto &plane = planes[i];
    std::cout << "Plane " << i << " (theta = " << plane.theta
              << " rad): " << plane.normal.transpose()
              << " · (x, y, z) = " << plane.d << "\n";

    auto rays = dator::sample::sampleRadialLines(point_on_line, plane.normal, 200);
    for (const auto &ray : rays)
    {
      auto hit = dator::sample::intersectRayWithCylinder(
          ray.origin, ray.direction, cyl_center, cyl_axis, cyl_radius);
      if (hit)
      {
        Eigen::Vector3d point = *hit;
        points.emplace_back(point);
      }
    }
  }

  ofs << "ply\nformat ascii 1.0\n";
  ofs << "element vertex " << points.size() << "\n";
  ofs << "property float x\nproperty float y\nproperty float z\nend_header\n";
  for (const auto &pt : points)
  {
    ofs << pt.x() << " " << pt.y() << " " << pt.z() << "\n";
  }
  ofs.close();

  for (auto point : points)
  {
    cloud.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }
  savePointCloudToPCD(cloud, "cylinder_ray.pcd");
}

TEST(cylinder, plane)
{
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}