#include <viosim/imu.h>
#include <viosim/utility.h>

#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <utility>

using namespace viosim;

std::vector<std::pair<vector<4>, vector<4>>> createPointsLines(std::vector<vector<4>> &points) {
    std::vector<std::pair<vector<4>, vector<4>>> lines;

    // read 2-point lines from model
    std::ifstream f("data/house.txt");
    double x1, y1, z1, x2, y2, z2;
    while (f >> x1 >> y1 >> z1 >> x2 >> y2 >> z2) {
        // std::cout << x1 << " " << y1 << " " << z1 << " " << x2 << " " << y2 << " " << z2 << std::endl;
        vector<4> pt0(x1, y1, z1, 1);
        vector<4> pt1(x2, y2, z2, 1);

        bool is_history_point = false;
        for (int i = 0; i < points.size(); ++i) {
            vector<4> pt = points[i];
            if (pt == pt0) {
                is_history_point = true;
            }
        }
        if (!is_history_point) {
            points.push_back(pt0);
        }

        is_history_point = false;
        for (int i = 0; i < points.size(); ++i) {
            vector<4> pt = points[i];
            if (pt == pt1) {
                is_history_point = true;
            }
        }
        if (!is_history_point) {
            points.push_back(pt1);
        }

        lines.emplace_back(pt0, pt1); // lines
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        vector<4> p = points[j] + vector<4>(0.5, 0.5, -0.5, 0);
        points.push_back(p);
    }

    // save points
    savePoints("output/all_points.txt", points);

    return lines;
}

std::vector<cv::Mat> generateImages(const cv::Mat &image, const std::vector<MotionData> &camdata) {
    if (image.empty()) {
        std::cerr << "image empty!" << std::endl;
    }

    Param params;
    std::vector<cv::Mat> warped_images;

    matrix<3> K;
    K << params.fx, 0.0, params.cx,
        0.0, params.fy, params.cy,
        0.0, 0.0, 1.0;
    std::vector<vector<4>> points;
    double edge = 12;
    points.emplace_back(-edge, edge * 3 / 4, 0, 1);
    points.emplace_back(edge, edge * 3 / 4, 0, 1);
    points.emplace_back(edge, -edge * 3 / 4, 0, 1);
    points.emplace_back(-edge, -edge * 3 / 4, 0, 1);
    std::vector<cv::Point2d> src_pts;
    src_pts.emplace_back(0, 0);
    src_pts.emplace_back(640, 0);
    src_pts.emplace_back(640, 480);
    src_pts.emplace_back(0, 480);
    // points obs in image
    for (int i = 0; i < camdata.size(); ++i) {
        std::vector<cv::Point2d> dst_pts;
        MotionData data = camdata[i];
        matrix<4> Twc = matrix<4>::Identity();
        Twc.block<3, 3>(0, 0) = data.Rwb; // data.Rwb is actually camera's Rwc
        Twc.block<3, 1>(0, 3) = data.twb; // data.twb is actually camera's twc

        for (int i = 0; i < points.size(); ++i) {
            vector<4> pw = points[i];
            vector<4> pc = Twc.inverse() * pw;
            vector<3> image_pt = {pc.x() / pc.w(), pc.y() / pc.w(), pc.z() / pc.w()};
            image_pt = K * image_pt;
            image_pt /= image_pt.z();
            dst_pts.emplace_back(image_pt.x(), image_pt.y());
        }
        cv::Mat H = cv::findHomography(src_pts, dst_pts);
        cv::Mat warped_image;
        warpPerspective(image, warped_image, H, warped_image.size());
        cv::imshow("affine image", warped_image);
        cv::waitKey(1);
        warped_images.push_back(warped_image);
    }

    return warped_images;
}

int main(int argc, const char *argv[]) {
    // 生成3d points
    std::vector<vector<4>> points;
    std::vector<std::pair<vector<4>, vector<4>>> lines;
    lines = createPointsLines(points);

    // IMU model
    Param params;
    IMUGenerator imu_generator(params);

    // create imu data
    // imu pose gyro acc
    std::vector<MotionData> imudata;
    std::vector<MotionData> imudata_noisy;
    for (float t = params.t_start; t < params.t_end; t += 1.0 / params.imu_frequency) {
        MotionData imu = imu_generator.genMotionData(t);
        imudata.push_back(imu);

        // add imu noise
        MotionData imu_noisy = imu;
        imu_generator.addIMUNoise(imu_noisy);
        imudata_noisy.push_back(imu_noisy);
    }
    imu_generator.init_velocity = imudata[0].imu_velocity;
    imu_generator.init_twb = imudata[0].twb;
    imu_generator.init_Rwb = imudata[0].Rwb;

    savePoses("output/imu_pose.txt", imudata);
    savePoses("output/imu_pose_noisy.txt", imudata_noisy);

    // test the imu data, integrate the imu data to generate the imu trajecotry
    imu_generator.testIMU("output/imu_pose.txt", "output/imu_int_pose.txt");
    imu_generator.testIMU("output/imu_pose_noisy.txt", "output/imu_int_pose_noisy.txt");

    // cam pose
    std::vector<MotionData> camdata;
    for (float t = params.t_start; t < params.t_end; t += 1.0 / params.cam_frequency) {
        MotionData imu = imu_generator.genMotionData(t); // imu body frame to world frame motion
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.Rbc;           // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.tbc; // Tcw = Twb * Tbc, t = Rwb * tbc + twb

        camdata.push_back(cam);
    }

    savePoses("output/cam_pose.txt", camdata);
    savePosesAsTUM("output/cam_pose_tum.txt", camdata);

    std::vector<cv::Mat> image_files = generateImages(cv::imread("data/template.jpg"), camdata);

    // points obs in image
    for (int n = 0; n < camdata.size(); ++n) {
        MotionData data = camdata[n];
        matrix<4> Twc = matrix<4>::Identity();
        Twc.block<3, 3>(0, 0) = data.Rwb;
        Twc.block<3, 1>(0, 3) = data.twb;

        // iterate to check whether in eyesight
        std::vector<vector<4>> points_cam;   // 3d points in camera frame
        std::vector<vector<2>> features_cam; // 2d points in normalized plane of camera frame
        for (int i = 0; i < points.size(); ++i) {
            vector<4> pw = points[i];
            vector<4> pc = Twc.inverse() * pw;

            // point in front of the camera
            if (pc.z() < 0) {
                continue;
            }

            vector<2> obs(pc.x() / pc.z(), pc.y() / pc.z());
            // if ((obs.x() * 460 + 255) < params.image_h && (obs.x() * 460 + 255) > 0 && (obs.y() * 460 + 255) > 0 && (obs.y() * 460 + 255) < params.image_w)
            {
                points_cam.push_back(points[i]);
                features_cam.push_back(obs);
            }
        }

        // save points
        saveFeatures("output/all_points_" + std::to_string(n) + ".txt", points_cam, features_cam);
    }

    // lines obs in image
    for (int n = 0; n < camdata.size(); ++n) {
        MotionData data = camdata[n];
        matrix<4> Twc = matrix<4>::Identity();
        Twc.block<3, 3>(0, 0) = data.Rwb;
        Twc.block<3, 1>(0, 3) = data.twb;

        // iterate to check whether in eyesight
        // std::vector<vector<4>> points_cam;   // 3d points in camera frame
        std::vector<vector<4>> features_cam; // 2d points in normalized plane of camera frame
        for (int i = 0; i < lines.size(); ++i) {
            std::pair<vector<4>, vector<4>> line = lines[i];

            vector<4> pc1 = Twc.inverse() * line.first;
            vector<4> pc2 = Twc.inverse() * line.second;

            // point in front of the camera
            if (pc1.z() < 0 || pc2.z() < 0) {
                continue;
            }

            vector<4> obs(pc1.x() / pc1.z(), pc1.y() / pc1.z(),
                          pc2.x() / pc2.z(), pc2.y() / pc2.z());
            // if (obs.x() < params.image_h && obs.x() > 0 && obs.y() > 0 && obs.y() < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save points
        saveLines("output/all_lines_" + std::to_string(n) + ".txt", features_cam);
    }

    return 0;
}
