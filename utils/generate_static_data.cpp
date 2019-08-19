#include <viosim/imu.h>
#include <viosim/utility.h>

#include <fstream>
#include <iostream>
#include <utility>

using namespace viosim;

int main(int argc, const char *argv[]) {
    // IMU model
    Param params;
    IMUGenerator imu_generator(params);

    // create imu data
    // imu pose gyro acc
    std::vector<MotionData> imudata_noisy;
    for (float t = 0.0; t < 8.0 * 60.0 * 60.0; t += 1.0 / 200.0) {
        MotionData imu_noisy;
        imu_noisy.timestamp = t;
        imu_noisy.imu_acc = {0.0, 0.0, 9.81};
        imu_noisy.imu_gyro = {0.0, 0.0, 0.0};

        imu_generator.addIMUNoise(imu_noisy);
        imudata_noisy.push_back(imu_noisy);
    }

    if (FILE *csv = fopen("output/acc.csv", "w")) {
        fputs("#acc: t[s:double],x[m/s^2:double],y[m/s^2:double],z[m/s^2:double]\n", csv);
        for (auto &item : imudata_noisy) {
            fprintf(csv, "%.14e,%.9e,%.9e,%.9e\n", item.timestamp, item.imu_acc.x(), item.imu_acc.y(), item.imu_acc.z());
        }
        fclose(csv);
    }

    if (FILE *csv = fopen("output/gyr.csv", "w")) {
        fputs("#gyr: t[s:double],x[rad/s:double],y[rad/s:double],z[rad/s:double]\n", csv);
        for (auto &item : imudata_noisy) {
            fprintf(csv, "%.14e,%.9e,%.9e,%.9e\n", item.timestamp, item.imu_gyro.x(), item.imu_gyro.y(), item.imu_gyro.z());
        }
        fclose(csv);
    }

    return 0;
}
