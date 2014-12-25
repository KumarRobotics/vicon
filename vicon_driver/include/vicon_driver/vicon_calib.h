/*
 * Copyright 2012 Kartik Mohta <kartikmohta@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VICONDRIVER_VICON_CALIB_H_
#define VICONDRIVER_VICON_CALIB_H_

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace vicon_driver
{
namespace calib
{
bool loadZeroPoseFromFile(const std::string &filename,
                          Eigen::Affine3d &zero_pose);

bool saveZeroPoseToFile(const Eigen::Affine3d &zero_pose,
                        const std::string &filename);

bool getTransform(const std::vector<Eigen::Vector3d> &reference_points,
                  const std::vector<Eigen::Vector3d> &actual_points,
                  Eigen::Affine3d &transform);

bool loadCalibMarkerPos(const std::string &filename,
                        std::map<std::string, Eigen::Vector3d> &marker_pos_map);
}
}

#endif // VICONDRIVER_VICON_CALIB_H_
