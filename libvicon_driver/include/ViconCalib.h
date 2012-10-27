#ifndef __VICON_CALIB_H__
#define __VICON_CALIB_H__

#include <Eigen/Geometry>
#include <vector>
#include <map>

namespace ViconCalib
{
  bool loadZeroPoseFromFile(const std::string &filename, Eigen::Affine3d &zero_pose);

  bool saveZeroPoseToFile(const Eigen::Affine3d &zero_pose, const std::string &filename);

  bool getTransform(const std::vector<Eigen::Vector3d> &reference_points,
                    const std::vector<Eigen::Vector3d> &actual_points,
                    Eigen::Affine3d &transform);

  bool loadCalibMarkerPos(const std::string &filename,
                          std::map<std::string, Eigen::Vector3d> &marker_pos_map);
}

#endif
