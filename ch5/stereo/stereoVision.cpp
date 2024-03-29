#include <Eigen/Core>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
// 文件路径
string left_file = "./left.png";
string right_file = "./right.png";
using PointClouldType =
    vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

void showPointCloud(const PointClouldType &pointcloud);

int main(int argc, char *argv[]) {
  // 内参
  double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  // 基线
  double b = 0.573;

  cv::Mat left = cv::imread(left_file, 0);
  cv::Mat right = cv::imread(right_file, 0);
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
      0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

  PointClouldType pointcloud;

  for (int v = 0; v < left.rows; v++) {
    for (int u = 0; u < left.cols; u++) {
      float cur_dispartiy = disparity.at<float>(v, u);
      if (cur_dispartiy <= 10.0 || cur_dispartiy >= 96.0) {
        continue;
      }

      // 根据双目模型计算 point 的位置

      double depth = fx * b / (disparity.at<float>(v, u));
      double x = (u - cx) / fx * depth;
      double y = (v - cy) / fy * depth;
      double color = left.at<uchar>(v, u) / 255.0;

      pointcloud.emplace_back(x, y, depth, color);
    }
  }
  cv::imshow("dispartiy", disparity / 96.0);
  cv::waitKey();
  showPointCloud(pointcloud);
  return 0;
}

void showPointCloud(const PointClouldType &pointcloud) {

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
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p : pointcloud) {
      glColor3f(p[3], p[3], p[3]);
      glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
    pangolin::FinishFrame();
    usleep(5000); // sleep 5 ms
  }
}