#pragma once

#include <viosim/common.h>
#include <viosim/param.h>

#include <string>
#include <vector>

namespace viosim {

// euler2Rotation: body frame to inertial frame
matrix<3> euler2Rotation(vector<3> euler_angles);
matrix<3> eulerRates2BodyRates(vector<3> euler_angles);

struct MotionData {
    double timestamp;

    matrix<3> Rwb;
    vector<3> twb;

    vector<3> imu_velocity;

    vector<3> imu_acc;  // acc
    vector<3> imu_gyro; // gyro

    vector<3> imu_acc_bias = vector<3>::Zero();  // acc bias
    vector<3> imu_gyro_bias = vector<3>::Zero(); // gyro bias
};

class IMUGenerator {
  public:
    IMUGenerator(Param p);
    Param param;

    matrix<3> init_Rwb;
    vector<3> init_twb;

    vector<3> init_velocity;

    vector<3> acc_bias;
    vector<3> gyro_bias;

    MotionData genMotionData(double t);
    void addIMUNoise(MotionData& data);
    // integrate the imu data, in order to check the trajectory
    void testIMU(std::string src, std::string dst);
};

} // namespace viosim
