#pragma once

#include <viosim/common.h>
#include <viosim/imu.h>

#include <vector>
#include <string>

namespace viosim {

// save 3d points to file
void savePoints(std::string filename, std::vector<vector<4>> &points);
// save 3d points and it's obs in image
void saveFeatures(std::string filename, std::vector<vector<4>> &points, std::vector<vector<2>> &features);
// save line obs
void saveLines(std::string filename, std::vector<vector<4>> &features);
// load imu body data
void loadPoses(std::string filename, std::vector<MotionData> &poses);
// save imu body data
void savePoses(std::string filename, std::vector<MotionData> &poses);
// save pose as TUM style
void savePosesAsTUM(std::string filename, std::vector<MotionData> &poses);

}
