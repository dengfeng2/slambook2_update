#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char *argv[]) {
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // 旋转向量使用AngleAxis，它底层不直接是Matrix，但运算可以当做矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  // 沿z轴旋转45度
    cout.precision(3);
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;  // 用matrix()转换成矩阵
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout << "rotation matrix = \n" << rotation_matrix << endl;

    // 用AngleAxis可以进行坐标变换
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // 欧拉角：可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // yaw-pitch-roll
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 欧式变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();     // 虽然称为3d，实质上是4x4的矩阵
    T.rotate(rotation_vector);                      // 按照 rotation_vector进行旋转
    T.pretranslate(Eigen::Vector3d(1, 3, 4));  // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed = T * v;
    cout << "v transformed = " << v_transformed << endl;

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把 AngleAxis 赋值给四元数，反之亦然
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;  // 注意coeffs的顺序是(x, y, z, w), w为实部，前三者为虚部
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q * v;  // 注意数学上是qvq^(-1)
    cout << "(1,0,0) after rotation (by quaternion) = " << v_rotated.transpose() << endl;
    // 用常规向量乘法表示，则应该如下计算（注意四元数的构造函数参数顺序是(w, x, y, z)）
    cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    return 0;
}
