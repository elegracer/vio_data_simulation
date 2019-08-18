#include <viosim/imu.h>

#include <viosim/utility.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

namespace viosim {

// euler2Rotation: body frame to inertial frame
matrix<3> euler2Rotation(vector<3> euler_angles) {
    // euler angles transforms a point in inertial frame to body frame.
    //
    // we use the following rotation convention:
    // 1. Rotate around Z-axis counter-clockwise. (yaw)
    // 2. Rotate around Y-axis counter-clockwise. (pitch)
    // 3. Rotate around X-axis counter-clockwise. (roll)
    //
    // xb = Rbi * xi
    //    = RX * RY * RZ * xi
    // but Rib is what we want.
    // Rib = Rbi^T

    double roll = euler_angles.x();  // psi
    double pitch = euler_angles.y(); // theta
    double yaw = euler_angles.z();   // phi

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    matrix<3> Rib;
    Rib << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
        sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
        -sp, cp * sr, cp * cr;
    return Rib;
}

matrix<3> eulerRates2BodyRates(vector<3> euler_angles) {
    // wb =   (dpsi/dt, 0, 0)^T
    //      + RX * (0, dtheta/dt, 0)^T
    //      + RX * RY * (0, 0, dphi/dt)^T
    double roll = euler_angles.x();
    double pitch = euler_angles.y();

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);

    matrix<3> R;
    R << 1, 0, -sp,
        0, cr, sr * cp,
        0, -sr, cr * cp;
    return R;
}

IMUGenerator::IMUGenerator(Param p) :
    param(p) {
    acc_bias = vector<3>::Zero();
    gyro_bias = vector<3>::Zero();
}

void IMUGenerator::addIMUNoise(MotionData& data) {
    // random double generator
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    vector<3> noise_acc(noise(gen), noise(gen), noise(gen));
    vector<3> noise_acc_bias(noise(gen), noise(gen), noise(gen));
    vector<3> noise_gyro(noise(gen), noise(gen), noise(gen));
    vector<3> noise_gyro_bias(noise(gen), noise(gen), noise(gen));

    // add acc white noise and bias to data
    data.imu_acc += param.sigma_acc_noise * noise_acc / std::sqrt(param.imu_timestep) + acc_bias;
    // add gyro white noise and bias to data
    data.imu_gyro += param.sigma_gyro_noise * noise_gyro / std::sqrt(param.imu_timestep) + gyro_bias;

    // store acc bias
    data.imu_acc_bias = acc_bias;
    // store gyro bias
    data.imu_gyro_bias = gyro_bias;

    // update acc bias
    acc_bias += param.sigma_acc_bias * noise_acc_bias * std::sqrt(param.imu_timestep);
    // update gyro bias
    gyro_bias += param.sigma_gyro_bias * noise_gyro_bias * std::sqrt(param.imu_timestep);
}

MotionData IMUGenerator::genMotionData(double t) {
    MotionData data;

    constexpr double PI = 3.141592653589793238463;
    // param
    constexpr double ellipse_x = 10;
    constexpr double ellipse_y = 10;
    constexpr double z = 8;       // sine in z-axis
    constexpr double K1 = 2;      // z-axis movement is K1 times faster x,y
    constexpr double K = PI / 10; // 20 * K = 2pi, every 20s a complete circle in yaw

    // translation
    vector<3> position(
        ellipse_x * cos(K * t) + 5,
        ellipse_y * sin(K * t) + 5,
        z * sin(K1 * K * t) + 20);
    vector<3> dp(
        -K * ellipse_x * sin(K * t),
        K * ellipse_y * cos(K * t),
        z * K1 * K * cos(K1 * K * t)); // 1st order derivative of position in world frame
    double K2 = K * K;
    vector<3> ddp(
        -K2 * ellipse_x * cos(K * t),
        -K2 * ellipse_y * sin(K * t),
        -z * K1 * K1 * K2 * sin(K1 * K * t)); // 2nd order derivative of position in world frame

    // rotation
    // double k_roll = 0.1;
    // double k_pitch = 0.2;
    // vector<3> euler_angles(k_roll * cos(t), k_pitch * sin(t), K * t);   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0, 2pi]
    // vector<3> euler_angles_rates(-k_roll * sin(t), k_pitch * cos(t), K); // derivative of euler angles
    vector<3> euler_angles(0, -1.1, K * t); // yaw ~ [0, 2pi]
    vector<3> euler_angles_rates(0, 0, K);  // derivative of euler angles

    matrix<3> Rwb = euler2Rotation(euler_angles);                                 // body frame to world frame
    vector<3> imu_gyro = eulerRates2BodyRates(euler_angles) * euler_angles_rates; // euler rates trans to body gyro

    vector<3> gn(0, 0, -9.81);                        // gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    vector<3> imu_acc = Rwb.transpose() * (ddp - gn); // Rbw * Rwn * gn = gs

    data.timestamp = t;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.imu_acc = imu_acc;
    data.imu_gyro = imu_gyro;
    return data;
}

// read the generated IMU data, and use IMU kinematics to calculate the trajectory
// then save the trajectory to validate the kinematics model
void IMUGenerator::testIMU(std::string src, std::string dst) {
    std::vector<MotionData> imudata;
    loadPoses(src, imudata);

    std::ofstream save_points(dst);
    save_points.setf(std::ios::fixed, std::ios::floatfield);

    double dt = param.imu_timestep;
    vector<3> pbc = param.tbc;
    quaternion qbc(param.Rbc);
    vector<3> pwc;
    quaternion qwc;
    vector<3> pwb = init_twb;     // position:     from imu measurements
    quaternion qwb(init_Rwb);     // quaternion:   from imu measurements
    vector<3> vw = init_velocity; // velocity:     from imu measurements
    vector<3> gw(0, 0, -9.81);    // ENU frame

    for (int i = 0; i < imudata.size(); ++i) {
        MotionData imu = imudata[i];

        // std::cout << imu.imu_gyro.transpose() << ", "
        //           << imu.imu_gyro_bias.transpose() << ", "
        //           << imu.imu_acc.transpose() << ", "
        //           << imu.imu_acc_bias.transpose() << "\n";

        // delta_q = [1, 1/2 * theta_x, 1/2 * theta_y, 1/2 * theta_z]
        quaternion dq;
        vector<3> dtheta_half = 0.5 * (imu.imu_gyro - imu.imu_gyro_bias) * dt;
        dq.w() = 1;
        dq.vec() = dtheta_half;

        // IMU kinematics, euler integral
        // aw = Rwb * (acc_body - acc_bias) + gw
        vector<3> acc_w = qwb * (imu.imu_acc - imu.imu_acc_bias) + gw;
        pwb = pwb + vw * dt + 0.5 * acc_w * dt * dt;
        vw = vw + acc_w * dt;
        qwb = qwb * dq;

        qwc = qwb * qbc;
        pwc = pwb + qwb * pbc;

        // format: imu postion, imu quaternion, cam postion, cam quaternion
        save_points.precision(9);
        save_points << imu.timestamp << ", "
                    << pwb.x() << ", "
                    << pwb.y() << ", "
                    << pwb.z() << ", "
                    << qwb.x() << ", "
                    << qwb.y() << ", "
                    << qwb.z() << ", "
                    << qwb.w() << ", "
                    << pwc.x() << ", "
                    << pwc.y() << ", "
                    << pwc.z() << ", "
                    << qwc.x() << ", "
                    << qwc.y() << ", "
                    << qwc.z() << ", "
                    << qwc.w() << std::endl;
    }

    std::cout << "saved test result from '" << src << "' to '" << dst << "'" << std::endl;
}

} // namespace viosim
