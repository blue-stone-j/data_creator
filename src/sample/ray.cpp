
#include "sample/ray.h"


std::vector<Ray> sampleRadialLines(const Eigen::Vector3d &origin,
                                   const Eigen::Vector3d &normal,
                                   int N)
{
  std::vector<Ray> rays;
  Eigen::Vector3d n = normal.normalized( );

  Eigen::Vector3d u;
  if (std::abs(n.z( )) < 1.0)
  {
    u = n.cross(Eigen::Vector3d::UnitZ( )).normalized( );
  }
  else
  {
    u = n.cross(Eigen::Vector3d::UnitY( )).normalized( );
  }
  Eigen::Vector3d v = n.cross(u);

  for (int i = 0; i < N; ++i)
  {
    double theta        = 2 * M_PI * i / N;
    Eigen::Vector3d dir = std::cos(theta) * u + std::sin(theta) * v;
    rays.push_back({origin, dir});
  }

  return rays;
}