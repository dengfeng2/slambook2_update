#include <Eigen/Core>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <unistd.h>

using namespace std;
using TrajectoryType =
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using PointCloudType = vector<Vector6d, Eigen::aligned_allocator<Vector6d>>;

void showPointCloud(const PointCloudType &pointcloud);

int main(int argc, char **argv) {
  vector<cv::Mat> colorImgs, depthImgs; // 彩色图和深度图
  TrajectoryType poses;                 // 相机位姿

  ifstream fin("./pose.txt");
  if (!fin) {
    cerr << "请在有pose.txt的目录下运行此程序" << endl;
    return 1;
  }

  for (int i = 0; i < 5; i++) {
    colorImgs.push_back(cv::imread(fmt::format("color/{}.png", i + 1)));
    depthImgs.push_back(cv::imread(fmt::format("depth/{}.pgm", i + 1),
                                   -1)); // 使用-1读取原始图像

    double x, y, z, qx, qy, qz, qw;
    fin >> x >> y >> z >> qx >> qy >> qz >> qw;
    Sophus::SE3d pose(Eigen::Quaterniond(qw, qx, qy, qz),
                      Eigen::Vector3d(x, y, z));
    poses.push_back(pose);
  }

      // 计算点云并拼接
      // 相机内参
      double cx = 325.5;
      double cy = 253.5;
      double fx = 518.0;
      double fy = 519.0;
      double depthScale = 1000.0;
      PointCloudType pointcloud;
      pointcloud.reserve(1000000);

      for (int i = 0; i < 5; i++) {
          cout << "转换图像中: " << i + 1 << endl;
          cv::Mat color = colorImgs[i];
          cv::Mat depth = depthImgs[i];
          Sophus::SE3d T = poses[i];
          for (int v = 0; v < color.rows; v++)
              for (int u = 0; u < color.cols; u++) {
                  unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                  if (d == 0) continue; // 为0表示没有测量到
                  Eigen::Vector3d point;
                  point[2] = double(d) / depthScale;
                  point[0] = (u - cx) * point[2] / fx;
                  point[1] = (v - cy) * point[2] / fy;
                  Eigen::Vector3d pointWorld = T * point;

                  Vector6d p;
                  p.head<3>() = pointWorld;
                  p[5] = color.data[v * color.step + u * color.channels()]; //blue 
                  p[4] = color.data[v * color.step + u * color.channels() + 1]; // green 
                  p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                  pointcloud.push_back(p);
              }
      }

      cout << "点云共有" << pointcloud.size() << "个点." << endl;
      showPointCloud(pointcloud);
      return 0;
}

void showPointCloud(const PointCloudType &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f /
        768.0f) .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
