#pragma once

#include <viosim/common.h>

namespace viosim {

class Param {
  public:
    Param();

    // time
    int imu_frequency = 200;
    int cam_frequency = 30;
    double imu_timestep = 1. / imu_frequency;
    double cam_timestep = 1. / cam_frequency;
    double t_start = 0;
    double t_end = 40; // s

    // white noise
    double sigma_acc_noise = 0.019;  // m/(s^2) * 1/sqrt(hz)
    double sigma_gyro_noise = 0.015; // rad/s * 1/sqrt(hz)
    // bias
    double sigma_acc_bias = 1.0e-4;  // m/(s^3) * 1/sqrt(hz)
    double sigma_gyro_bias = 1.0e-5; // rad(s^2) * 1/sqrt(hz)

    double pixel_noise = 1; // 1 pixel noise

    // camemra parameters
    double fx = 460;
    double fy = 460;
    double cx = 320;
    double cy = 240;
    double image_w = 640;
    double image_h = 480;

    // extrinsic parameters
    matrix<3> Rbc; // cam to body
    vector<3> tbc; // cam to body
};

}
