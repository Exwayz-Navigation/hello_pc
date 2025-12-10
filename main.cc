#include <iostream>

#include <Eigen/Dense>

#include <happly.h>

#include <nanoflann.hpp>


void readPointCloud(const std::string& src_fpath, std::vector<Eigen::Vector3f>& dst_points) {
  happly::PLYData ply_in(src_fpath);

  size_t num_points = ply_in.getElement("vertex").count;
  std::vector<float> x = ply_in.getElement("vertex").getProperty<float>("x");
  std::vector<float> y = ply_in.getElement("vertex").getProperty<float>("y");
  std::vector<float> z = ply_in.getElement("vertex").getProperty<float>("z");

  dst_points.resize(num_points);

  for (int i(0); i<num_points; ++i) {
    dst_points[i].x() = x[i];
    dst_points[i].y() = y[i];
    dst_points[i].z() = z[i];
  }
}

void writePointCloud(const std::string& dst_fpath, const std::vector<Eigen::Vector3f>& src_points) {
  happly::PLYData ply_out;

  size_t num_points = src_points.size();

  std::vector<float> x(num_points);
  std::vector<float> y(num_points);
  std::vector<float> z(num_points);

  for (int i(0); i<num_points; ++i) {
    x[i] = src_points[i].x();
    y[i] = src_points[i].y();
    z[i] = src_points[i].z();
  }

  ply_out.addElement("vertex", num_points);
  ply_out.getElement("vertex").addProperty<float>("x", x);
  ply_out.getElement("vertex").addProperty<float>("y", y);
  ply_out.getElement("vertex").addProperty<float>("z", z);

  ply_out.write(dst_fpath, happly::DataFormat::Binary);
}

int main(int argc, char* argv[]) {
  std::vector<Eigen::Vector3f> points;

  readPointCloud("/data/paris_13th.ply", points);

  // Print first 10 points
  for (int i(0); i<10; ++i) {
    std::cout << points[i].transpose() << std::endl;
  }

  writePointCloud("/data/paris_13th_proc.ply", points);

  return 0;
}
