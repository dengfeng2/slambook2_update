#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace std;

string groundtruth_file = "./example/groundtruth.txt";
string estimated_file = "./example/estimated.txt";

using TrajectoryType = vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

TrajectoryType ReadTrajectory(const string &path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

int main(int argc, char *argv[]) {
    TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
    TrajectoryType estimated = ReadTrajectory(estimated_file);
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); ++i) {
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / estimated.size();
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    DrawTrajectory(groundtruth, estimated);
    return 0;
}

TrajectoryType ReadTrajectory(const string &path) {
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }
    return trajectory;
}

void DrawTrajectory(const TrajectoryType &traj) {
    for (size_t i = 0; i < traj.size(); ++i) {
        glBegin(GL_LINES);
        // 不考虑姿态，只考虑位置
        auto p1_trans = traj[i].translation();
        auto p2_trans = traj[i+1].translation();
        glVertex3d(p1_trans[0], p1_trans[1], p1_trans[2]);
        glVertex3d(p2_trans[0], p2_trans[1], p2_trans[2]);
        glEnd();
    }
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
        DrawTrajectory(gt);
        glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
        DrawTrajectory(esti);

        pangolin::FinishFrame();
        usleep(5000);
    }
}