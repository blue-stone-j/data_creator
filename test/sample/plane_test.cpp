#include <gtest/gtest.h>

#include "sample/plane.h"

TEST(plane, line)
{
  // Define the line: point + direction
  Eigen::Vector3d point_on_line(0.0, 0.0, 0.0);
  Eigen::Vector3d line_direction(0.0, 0.0, 1.0); // z-axis

  int N       = 8; // sample 8 planes
  auto planes = dator::sample::samplePlanesThroughLine(point_on_line, line_direction, N);

  std::cout << std::fixed << std::setprecision(3);
  for (int i = 0; i < N; ++i)
  {
    const auto &plane = planes[i];
    std::cout << "Plane " << i
              << " (theta = " << plane.theta << " rad): "
              << plane.normal.transpose()
              << " · (x, y, z) = " << plane.d << "\n";
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}