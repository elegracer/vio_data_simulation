#include <viosim/utility.h>

#include <fstream>
#include <iostream>

namespace viosim {

void savePoints(std::string filename, std::vector<vector<4>> &points) {
    std::ofstream save_points(filename);
    save_points.precision(9);

    for (int i = 0; i < points.size(); ++i) {
        vector<4> p = points[i];

        save_points << p.x() << ", "
                    << p.y() << ", "
                    << p.z() << ", "
                    << p.w() << std::endl;
    }

    std::cout << "saved points to '" << filename << "'" << std::endl;
}

void saveFeatures(std::string filename, std::vector<vector<4>> &points, std::vector<vector<2>> &features) {
    std::ofstream save_points(filename);
    save_points.precision(9);

    for (int i = 0; i < points.size(); ++i) {
        vector<4> p = points[i];
        vector<2> f = features[i];
        save_points << p.x() << ", "
                    << p.y() << ", "
                    << p.z() << ", "
                    << p.w() << ", "
                    << f.x() << ", "
                    << f.y() << std::endl;
    }

    std::cout << "saved features to '" << filename << "'" << std::endl;
}
void saveLines(std::string filename, std::vector<vector<4>> &features) {
    std::ofstream save_points(filename);
    save_points.precision(9);

    for (int i = 0; i < features.size(); ++i) {
        vector<4> f = features[i];
        save_points << f.x() << ", "
                    << f.y() << ", "
                    << f.z() << ", "
                    << f.w() << std::endl;
    }

    std::cout << "saved lines to '" << filename << "'" << std::endl;
}

void loadPoses(std::string filename, std::vector<MotionData> &poses) {
    if (FILE *f = fopen(filename.c_str(), "r")) {
        char buffer[2048];
        double t, px, py, pz, qx, qy, qz, qw, ax, ay, az, gx, gy, gz, bax, bay, baz, bgx, bgy, bgz;
        while (!feof(f)
               && fscanf(f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf[^\r\n]%*[\r\n]",
                         &t, &px, &py, &pz, &qx, &qy, &qz, &qw, &ax, &ay, &az, &gx, &gy, &gz, &bax, &bay, &baz, &bgx, &bgy, &bgz)
                      == 20) {
            MotionData data;
            data.timestamp = t;
            data.twb = {px, py, pz};
            data.Rwb = quaternion(qw, qx, qy, qz);
            data.imu_acc = {ax, ay, az};
            data.imu_gyro = {gx, gy, gz};
            data.imu_acc_bias = {bax, bay, baz};
            data.imu_gyro_bias = {bgx, bgy, bgz};
            poses.push_back(data);
        }
        fclose(f);
    } else {
        std::cerr << "loadPoses: can't open '" << filename << "'" << std::endl;
        return;
    }

    std::cout << "loaded poses from '" << filename << "'" << std::endl;
}

void savePoses(std::string filename, std::vector<MotionData> &poses) {
    std::ofstream save_points(filename);
    save_points.setf(std::ios::fixed, std::ios::floatfield);

    for (int i = 0; i < poses.size(); ++i) {
        MotionData data = poses[i];

        double time = data.timestamp;
        quaternion q(data.Rwb);
        vector<3> p = data.twb;
        vector<3> a = data.imu_acc;
        vector<3> g = data.imu_gyro;
        vector<3> ba = data.imu_acc_bias;
        vector<3> bg = data.imu_gyro_bias;

        save_points.precision(9);
        save_points << time << ", "
                    << p.x() << ", "
                    << p.y() << ", "
                    << p.z() << ", "
                    << q.x() << ", "
                    << q.y() << ", "
                    << q.z() << ", "
                    << q.w() << ", "
                    << a.x() << ", "
                    << a.y() << ", "
                    << a.z() << ", "
                    << g.x() << ", "
                    << g.y() << ", "
                    << g.z() << ", "
                    << ba.x() << ", "
                    << ba.y() << ", "
                    << ba.z() << ", "
                    << bg.x() << ", "
                    << bg.y() << ", "
                    << bg.z() << std::endl;
    }

    std::cout << "saved poses to '" << filename << "'" << std::endl;
}

void savePosesAsTUM(std::string filename, std::vector<MotionData> &poses) {
    std::ofstream save_points(filename);
    save_points.setf(std::ios::fixed, std::ios::floatfield);

    for (int i = 0; i < poses.size(); ++i) {
        MotionData data = poses[i];

        double time = data.timestamp;
        quaternion q(data.Rwb);
        vector<3> p = data.twb;
        vector<3> gyro = data.imu_gyro;
        vector<3> acc = data.imu_acc;

        // TUM: timestamp(s), px, py, pz, qx, qy, qz, qw
        save_points.precision(9);
        save_points << time << ", "
                    << p.x() << ", "
                    << p.y() << ", "
                    << p.z() << ", "
                    << q.x() << ", "
                    << q.y() << ", "
                    << q.z() << ", "
                    << q.w() << std::endl;
    }

    std::cout << "saved points as TUM to '" << filename << "'" << std::endl;
}

} // namespace viosim
