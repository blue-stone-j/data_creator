#ifndef SAVE_H
#define SAVE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

static void savePointsToFile(const std::vector<Point3D> &points, const std::string &filename)
{
  std::ofstream file(filename);
  if (!file)
  {
    std::cerr << "Error: Could not open file for writing!\n";
    return;
  }

  for (const auto &p : points)
  {
    file << p.x << " " << p.y << " " << p.z << "\n";
  }

  file.close();
  std::cout << "Points saved to " << filename << std::endl;
}

static void savePointsToFile(const std::vector<Eigen::Vector3d> &points, const std::string &filename)
{
  std::ofstream file(filename);
  if (!file)
  {
    std::cerr << "Error: Could not open file for writing!\n";
    return;
  }

  for (const auto &p : points)
  {
    file << p.x() << " " << p.y() << " " << p.z() << "\n";
  }

  file.close();
  std::cout << "Points saved to " << filename << std::endl;
}

static void savePointsToFile(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &filename)
{
  std::ofstream file(filename);
  if (!file)
  {
    std::cerr << "Error: Could not open file for writing!\n";
    return;
  }

  for (const auto &p : cloud)
  {
    file << p.x << " " << p.y << " " << p.z << "\n";
  }

  file.close();
  std::cout << "Points saved to " << filename << std::endl;
}

static void savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &filename)
{
  pcl::io::savePCDFileBinary(filename, cloud);
  std::cout << "Point cloud saved to " << filename << std::endl;
}
static void savePointCloudToPCD(const std::vector<Eigen::Vector3d> &points, const std::string &filename)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto point : points)
  {
    cloud.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }
  pcl::io::savePCDFileBinary(filename, cloud);
  std::cout << "Point cloud saved to " << filename << std::endl;
}

static void savePointCloudToPLY(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &filename)
{
  std::ofstream ofs(filename);

  ofs << "ply\nformat ascii 1.0\n";
  ofs << "element vertex " << cloud.size() << "\n";
  ofs << "property float x\nproperty float y\nproperty float z\nend_header\n";
  for (const auto &pt : cloud)
  {
    ofs << pt.x << " " << pt.y << " " << pt.z << "\n";
  }

  ofs.close();
}

#endif