/*
 * Copyright 2012, 2014 Kartik Mohta <kartikmohta@gmail.com>
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

#include "vicon_driver/vicon_calib.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

// Support for new yaml-cpp, taken from
// https://github.com/ros-planning/moveit_setup_assistant/commit/c9238ca#diff-1
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
static void operator>>(const YAML::Node &node, T &i)
{
  i = node.as<T>();
}
#endif

// yaml-cpp 0.5 also changed how you load the YAML document.  This
// function hides the changes.
static void loadYaml(std::istream &in_stream, YAML::Node &doc_out)
{
#ifdef HAVE_NEW_YAMLCPP
  doc_out = YAML::Load(in_stream);
#else
  YAML::Parser parser(in_stream);
  parser.GetNextDocument(doc_out);
#endif
}

namespace vicon_driver
{
namespace calib
{
bool loadZeroPoseFromFile(const std::string &filename,
                          Eigen::Affine3d &zero_pose)
{
  zero_pose = Eigen::Affine3d::Identity();

  std::ifstream fin(filename.c_str());
  if(!fin.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening file: " << filename
              << ", setting zero pose to identity" << std::endl;
#endif
    return false;
  }

  bool ret = false;

  try
  {
    YAML::Node doc;
    loadYaml(fin, doc);

    try
    {
      Eigen::Vector3d v;
      Eigen::Quaterniond q;
      doc["translation"]["x"] >> v(0);
      doc["translation"]["y"] >> v(1);
      doc["translation"]["z"] >> v(2);
      doc["rotation"]["x"] >> q.x();
      doc["rotation"]["y"] >> q.y();
      doc["rotation"]["z"] >> q.z();
      doc["rotation"]["w"] >> q.w();
      zero_pose.translate(v);
      zero_pose.rotate(q);
      ret = true;
    }
    catch(YAML::KeyNotFound &e)
    {
      ret = false;
    }
  }
  catch(YAML::ParserException &e)
  {
    ret = false;
  }
  fin.close();

  if(!ret)
  {
#if DEBUG
    std::cerr << "Error parsing calib file: " << filename
              << ", setting zero_pose to Identity" << std::endl;
#endif
  }
  return ret;
}

bool saveZeroPoseToFile(const Eigen::Affine3d &zero_pose,
                        const std::string &filename)
{
  YAML::Emitter out;

  Eigen::Vector3d v(zero_pose.translation());
  Eigen::Quaterniond q(zero_pose.rotation());

  out << YAML::BeginMap;
  out << YAML::Key << "translation";
  out << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value
      << v.x() << YAML::Key << "y" << YAML::Value << v.y() << YAML::Key << "z"
      << YAML::Value << v.z() << YAML::EndMap;
  out << YAML::Key << "rotation";
  out << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value
      << q.x() << YAML::Key << "y" << YAML::Value << q.y() << YAML::Key << "z"
      << YAML::Value << q.z() << YAML::Key << "w" << YAML::Value << q.w()
      << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream fout(filename.c_str(),
                     std::ios_base::out | std::ios_base::trunc);
  if(!fout.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening file: " << filename << std::endl;
#endif
    return false;
  }

  bool ret = true;

  fout << out.c_str() << std::endl;
  if(!fout.good())
  {
#ifdef DEBUG
    std::cerr << "Error writing to file: " << filename << std::endl;
#endif
    ret = false;
  }

  fout.close();

  return ret;
}

bool getTransform(const std::vector<Eigen::Vector3d> &reference_points,
                  const std::vector<Eigen::Vector3d> &actual_points,
                  Eigen::Affine3d &transform)
{
  transform = Eigen::Affine3d::Identity();

  if(reference_points.size() != actual_points.size())
  {
    return false;
  }

  // Algorithm from http://dx.doi.org/10.1016/0021-9290(94)00116-L
  const size_t num_points = reference_points.size();
  Eigen::Vector3d reference_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d actual_mean = Eigen::Vector3d::Zero();
  for(size_t i = 0; i < num_points; i++)
  {
    reference_mean += reference_points[i];
    actual_mean += actual_points[i];
  }
  reference_mean /= num_points;
  actual_mean /= num_points;

  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for(size_t i = 0; i < num_points; i++)
  {
    C += (actual_points[i] - actual_mean) *
         (reference_points[i] - reference_mean).transpose();
  }
  C /= num_points;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      C, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d V = svd.matrixV();
  if(U.determinant() * V.determinant() < 0)
  {
    S(2, 2) = -1;
  }

  const Eigen::Matrix3d rotation = U * S * V.transpose();

  const Eigen::Vector3d translation = actual_mean - rotation * reference_mean;

  transform.translate(translation);
  transform.rotate(rotation);

  return true;
}

bool loadCalibMarkerPos(const std::string &filename,
                        std::map<std::string, Eigen::Vector3d> &marker_pos_map)
{
  std::ifstream fin(filename.c_str());
  if(!fin.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening calib marker position file: " << filename
              << std::endl;
#endif
    return false;
  }

  bool ret = false;

  try
  {
    YAML::Node doc;
    loadYaml(fin, doc);

    try
    {
      const YAML::Node &markers = doc["markers"];

      for(unsigned int i = 0; i < markers.size(); i++)
      {
        std::string name;
        markers[i]["name"] >> name;
        const YAML::Node &pos = markers[i]["position"];
        Eigen::Vector3d position;
        pos[0] >> position.x();
        pos[1] >> position.y();
        pos[2] >> position.z();
        marker_pos_map.insert(std::make_pair(name, position));
      }
      ret = true;
    }
    catch(YAML::KeyNotFound &e)
    {
      ret = false;
    }
  }
  catch(YAML::ParserException &e)
  {
    ret = false;
  }
  fin.close();

  if(!ret)
  {
#if DEBUG
    std::cerr << "Error parsing calib marker position file: " << filename
              << std::endl;
#endif
    marker_pos_map.clear();
  }
  return ret;
}
}
}
