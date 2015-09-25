/*
 * Copyright 2012-2013 Kartik Mohta <kartikmohta@gmail.com>
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

#include "vicon_driver/vicon_driver.h"
#include <time.h>
#include <iostream>
#include "Client.h"

namespace ViconSDK = ViconDataStreamSDK::CPP;

namespace vicon_driver
{
ViconDriver::ViconDriver()
    : connected_(false),
      running_(false),
      subject_callback_(NULL),
      unlabeled_markers_callback_(NULL),
      unlabeled_marker_data_enabled_(false)
{
  client_ = new ViconSDK::Client();
}

ViconDriver::~ViconDriver()
{
  delete client_;
}

bool ViconDriver::init(std::string host)
{
  struct timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;

  std::cout << "Connecting to Vicon Datastream server at " << host << std::endl;
  int retries = 10;
  while(retries > 0)
  {
    client_->Connect(host);
    if(client_->IsConnected().Connected)
    {
      break;
    }
    else
    {
      nanosleep(&ts_sleep, NULL);
      retries--;
    }
  }
  connected_ = client_->IsConnected().Connected;
  if(!connected_)
    return false;

  std::cout << "Connected" << std::endl;

  client_->SetStreamMode(ViconSDK::StreamMode::ClientPull);
  client_->SetAxisMapping(ViconSDK::Direction::Forward,
                          ViconSDK::Direction::Left, ViconSDK::Direction::Up);
  return true;
}

bool ViconDriver::start(void)
{
  if(!connected_)
  {
    std::cout << "Called start but not connected to the Vicon Datastream "
                 "server, call init first" << std::endl;
    return false;
  }

  if(!running_)
  {
    client_->EnableSegmentData();
    if(!client_->IsSegmentDataEnabled().Enabled)
      return false;

    client_->EnableMarkerData();
    if(!client_->IsMarkerDataEnabled().Enabled)
      return false;

    // Need to wait for some time after enabling data else you get junk frames
    struct timespec ts_sleep;
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 100000000;
    nanosleep(&ts_sleep, NULL);

    grab_frames_ = true;
    int ret =
        pthread_create(&grab_thread_, NULL, ViconDriver::grabThread, this);
    if(ret != 0)
      return false;

    running_ = true;
  }
  return true;
}

bool ViconDriver::shutdown(void)
{
  if(connected_)
  {
    if(running_)
    {
      grab_frames_ = false; // Tell the grab thread to stop
      int ret = pthread_join(grab_thread_, NULL); // Wait for thread to end
      if(ret != 0)
        return false;

      running_ = false;
    }
    client_->Disconnect();
    connected_ = client_->IsConnected().Connected;
  }
  return !connected_; // If still connected return false
}

bool ViconDriver::enableUnlabeledMarkerData(bool enable)
{
  if(enable)
    client_->EnableUnlabeledMarkerData();
  else
    client_->DisableUnlabeledMarkerData();

  unlabeled_marker_data_enabled_ =
      client_->IsUnlabeledMarkerDataEnabled().Enabled;
  return (unlabeled_marker_data_enabled_ == enable);
}

void ViconDriver::setSubjectPubCallback(SubjectPubCallback_t callback)
{
  subject_callback_ = callback;
}

void ViconDriver::setUnlabeledMarkersPubCallback(MarkersPubCallback_t callback)
{
  unlabeled_markers_callback_ = callback;
}

void *ViconDriver::grabThread(void *arg)
{
  ViconDriver *vd = reinterpret_cast<ViconDriver*>(arg);

  ViconSDK::Result::Enum result;
  struct timespec ts_now;
  int64_t last_frame_time = 0;
  int32_t last_frame_number = 0;

  double frame_rate;
  result = vd->client_->GetFrame().Result; // Need a frame for GetFrameRate
  if(result == ViconSDK::Result::Success)
    frame_rate = vd->client_->GetFrameRate().FrameRateHz;
  else
    frame_rate = 100; // Fallback

  int32_t dt = 1000000 / frame_rate; // usec
  const double alpha = 0.9; // For low pass filter

  while(vd->grab_frames_)
  {
    result = vd->client_->GetFrame().Result;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    if(result != ViconSDK::Result::Success)
      continue;

    const double latency = vd->client_->GetLatencyTotal().Total;

    const int64_t expected_frame_time =
        (ts_now.tv_sec * 1000000 + (ts_now.tv_nsec + 500) / 1000) -
        latency * 1e6;
    if(last_frame_time == 0)
      last_frame_time = expected_frame_time - dt;

    const int32_t frame_number = vd->client_->GetFrameNumber().FrameNumber;
    if(last_frame_number == 0)
      last_frame_number = frame_number - 1;

    const int32_t time_diff = expected_frame_time - last_frame_time;
    const int32_t frame_diff = frame_number - last_frame_number;

    // Low pass filter to remove jitter noise in dt
    dt = alpha * dt + (1 - alpha) * (time_diff / frame_diff);

    const int64_t frame_time = last_frame_time + frame_diff * dt;
    last_frame_time = frame_time;
    last_frame_number = frame_number;

    if(!vd->processFrame(frame_time, frame_number))
    {
      std::cout << "Error processing frame" << std::endl;
    }
  }

  return NULL;
}

bool ViconDriver::processFrame(int64_t frame_time_usec,
                               unsigned int frame_number)
{
  static unsigned int last_frame_number = 0;
  static unsigned int frame_count = 0, dropped_frame_count = 0;

  int frame_diff = 1;

  if(last_frame_number != 0)
  {
    frame_diff = frame_number - last_frame_number;
    frame_count += frame_diff;
    if(frame_diff > 1)
    {
      dropped_frame_count += (frame_diff - 1);
      double dropped_frame_pct =
          (double)dropped_frame_count / frame_count * 100;
      std::cout << "Dropped " << frame_diff - 1 << " more (total "
                << dropped_frame_count << "/" << frame_count << ", "
                << dropped_frame_pct
                << "%) frame(s) dropped. Consider adjusting rates."
                << std::endl;
    }
  }
  last_frame_number = frame_number;

  if(frame_diff == 0)
    return false;

  processSubjects(frame_time_usec, frame_number);
  processUnlabeledMarkers(frame_time_usec, frame_number);

  return true;
}

void ViconDriver::processSubjects(int64_t frame_time_usec,
                                  unsigned int frame_number)
{
  std::string subject_name, segment_name;
  static unsigned int count = 0;
  count++;

  unsigned int n_subjects = client_->GetSubjectCount().SubjectCount;
  for(unsigned int i = 0; i < n_subjects; i++)
  {
    subject_name = client_->GetSubjectName(i).SubjectName;
    unsigned int n_segments =
        client_->GetSegmentCount(subject_name).SegmentCount;
    if(n_segments == 0)
    {
      if(count % 100 == 0)
        std::cout << "No segments for subject " << subject_name << std::endl;
    }
    else
    {
      if(n_segments > 1)
      {
        if(count % 100 == 0)
          std::cout << "Multiple segments for subject " << subject_name
                    << ", only publishing pose for first segment" << std::endl;
      }

      ViconDriver::Subject subject;

      segment_name = client_->GetSegmentName(subject_name, 0).SegmentName;

      ViconSDK::Output_GetSegmentGlobalTranslation trans =
          client_->GetSegmentGlobalTranslation(subject_name, segment_name);
      ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
          client_->GetSegmentGlobalRotationQuaternion(subject_name,
                                                      segment_name);

      subject.time_usec = frame_time_usec;
      subject.frame_number = frame_number;
      subject.name = subject_name;

      if(trans.Result == ViconSDK::Result::Success &&
         quat.Result == ViconSDK::Result::Success && !trans.Occluded &&
         !quat.Occluded)
      {
        subject.occluded = false;
        subject.translation[0] = trans.Translation[0] / 1000.0;
        subject.translation[1] = trans.Translation[1] / 1000.0;
        subject.translation[2] = trans.Translation[2] / 1000.0;
        subject.rotation[0] = quat.Rotation[0];
        subject.rotation[1] = quat.Rotation[1];
        subject.rotation[2] = quat.Rotation[2];
        subject.rotation[3] = quat.Rotation[3];
      }
      else
      {
        subject.occluded = true;
        subject.translation[0] = 0;
        subject.translation[1] = 0;
        subject.translation[2] = 0;
        subject.rotation[0] = 0;
        subject.rotation[1] = 0;
        subject.rotation[2] = 0;
        subject.rotation[3] = 1;
      }

      unsigned int n_subject_markers =
          client_->GetMarkerCount(subject_name).MarkerCount;
      for(unsigned int i_markers = 0; i_markers < n_subject_markers;
          i_markers++)
      {
        Marker marker;
        marker.name =
            client_->GetMarkerName(subject_name, i_markers).MarkerName;
        marker.subject_name = subject_name;

        ViconSDK::Output_GetMarkerGlobalTranslation translation =
            client_->GetMarkerGlobalTranslation(subject_name, marker.name);
        marker.translation[0] = translation.Translation[0] / 1000.0;
        marker.translation[1] = translation.Translation[1] / 1000.0;
        marker.translation[2] = translation.Translation[2] / 1000.0;
        marker.occluded = translation.Occluded;

        subject.markers.push_back(marker);
      }
      if(subject_callback_ != NULL)
        subject_callback_(subject);
    }
  }
}

void ViconDriver::processUnlabeledMarkers(int64_t frame_time_usec,
                                          unsigned int frame_number)
{
  if(!unlabeled_marker_data_enabled_)
    return;

  Markers unlabeled_markers;
  unlabeled_markers.time_usec = frame_time_usec;
  unlabeled_markers.frame_number = frame_number;

  unsigned int n_unlabeled_markers =
      client_->GetUnlabeledMarkerCount().MarkerCount;
  for(unsigned int i_markers = 0; i_markers < n_unlabeled_markers; i_markers++)
  {
    Marker marker;

    ViconSDK::Output_GetUnlabeledMarkerGlobalTranslation translation =
        client_->GetUnlabeledMarkerGlobalTranslation(i_markers);
    marker.translation[0] = translation.Translation[0] / 1000.0;
    marker.translation[1] = translation.Translation[1] / 1000.0;
    marker.translation[2] = translation.Translation[2] / 1000.0;
    marker.occluded = false; // unlabeled markers cannot be occluded

    unlabeled_markers.markers.push_back(marker);
  }

  if(unlabeled_markers_callback_ != NULL)
    unlabeled_markers_callback_(unlabeled_markers);
}

} // namespace vicon_driver
