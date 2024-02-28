# slambook2 代码更新版本

## 前置准备
```
# 安装依赖
sudo apt install gcc g++ cmake automake libeigen3-dev libopengl-dev libglx-dev libgl1-mesa-glx freeglut3-dev mesa-common-dev libglew-dev libfmt-dev libopencv-dev libgflags-dev libgoogle-glog-dev
libsuitesparse-dev libmetis-dev qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools qtcreator libqglviewer-dev-qt5
# 下载代码
git clone https://github.com/dengfeng2/slambook2_update.git --recursive

# 编译Pangolin
cd 3rdparty/Pangolin
mkdir build
cd build
cmake ..
make -j4
make install

# 编译Sophus
cd 3rdparty/Sophus
mkdir build
cd build
cmake ..
make -j4
make install

# 编译ceres-solver
cd 3rdparty/ceres-solver
mkdir build
cd build
cmake ..
make -j4
make install

# 编译g2o
cd 3rdparty/g2o
mkdir build
cd build
cmake ..
make -j4
make install
```