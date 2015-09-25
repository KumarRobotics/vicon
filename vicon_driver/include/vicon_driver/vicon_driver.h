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

#ifndef VICONDRIVER_VICON_DRIVER_H_
#define VICONDRIVER_VICON_DRIVER_H_

#include <stdint.h>
#include <string>
#include <vector>
#include <pthread.h>

namespace ViconDataStreamSDK
{
namespace CPP
{
class Client;
} // namespace CPP
} // namespace ViconDataStreamSDK

namespace vicon_driver
{
class ViconDriver
{
 public:
  typedef struct _Marker
  {
    std::string name;
    std::string subject_name;
    double translation[3];
    bool occluded;
  } Marker;

  typedef struct _Subject
  {
    int64_t time_usec;
    int32_t frame_number;
    std::string name;
    bool occluded;
    double translation[3];
    double rotation[4]; // Quaternion(x,y,z,w)
    std::vector<ViconDriver::Marker> markers;
  } Subject;

  typedef struct _Markers
  {
    int64_t time_usec;
    int32_t frame_number;
    std::vector<ViconDriver::Marker> markers;
  } Markers;

  typedef void (*SubjectPubCallback_t)(const ViconDriver::Subject &subject);
  typedef void (*MarkersPubCallback_t)(const ViconDriver::Markers &markers);

  ViconDriver();
  ~ViconDriver();

  bool init(std::string host);
  // Starts the vicon thread which grabs and processes frames from the server
  bool start(void);
  bool shutdown(void);

  bool enableUnlabeledMarkerData(bool enable);

  void setSubjectPubCallback(SubjectPubCallback_t callback);
  void setUnlabeledMarkersPubCallback(MarkersPubCallback_t callback);

 private:
  static void *grabThread(void *arg);
  bool processFrame(int64_t frame_time_usec, unsigned int frame_number);
  void processSubjects(int64_t frame_time_usec, unsigned int frame_number);
  void processUnlabeledMarkers(int64_t frame_time_usec,
                               unsigned int frame_number);

  ViconDataStreamSDK::CPP::Client *client_;
  bool connected_, running_;
  SubjectPubCallback_t subject_callback_;
  MarkersPubCallback_t unlabeled_markers_callback_;
  pthread_t grab_thread_;
  bool grab_frames_;
  bool unlabeled_marker_data_enabled_;
};

} // namespace vicon_driver

#endif // VICONDRIVER_VICON_DRIVER_H_
