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

#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <iostream>
#include <map>
#include <set>
#include "ipc.h"
#include "vicon_driver/vicon_driver.h"
#include "vicon_driver/vicon_calib.h"
#include "msgs/ViconTypes.h"

static std::string calib_files_dir;
static volatile bool running = false;
static pthread_mutex_t ipc_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t calib_set_mutex = PTHREAD_MUTEX_INITIALIZER;
static std::map<std::string, bool> calib_set;
static std::map<std::string, Eigen::Affine3d, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string,
    Eigen::Affine3d> > > calib_pose;

static void sigint_handler(int signo)
{
  running = false;
}

static void *loadCalibThread(void *arg)
{
  std::string *subject_name = reinterpret_cast<std::string*>(arg);
  std::string calib_filename = calib_files_dir + "/" + *subject_name + ".yaml";
  Eigen::Affine3d zero_pose;

  if(!vicon_driver::calib::loadZeroPoseFromFile(calib_filename, zero_pose))
  {
    std::cerr << "Error loading calib for " << *subject_name << " from file " <<
        calib_filename << ", setting calib pose to Identity" << std::endl;
  }

  calib_pose[*subject_name] = zero_pose.inverse();
  pthread_mutex_lock(&calib_set_mutex);
  calib_set[*subject_name] = true;
  pthread_mutex_unlock(&calib_set_mutex);

  delete subject_name;
  return NULL;
}

static bool loadCalib(const std::string subject_name)
{
  if(pthread_mutex_trylock(&calib_set_mutex) != 0)
    return false;

  std::map<std::string, bool>::iterator it = calib_set.find(subject_name);
  if(it == calib_set.end())
  {
    calib_set[subject_name] = false;
    pthread_mutex_unlock(&calib_set_mutex);

    pthread_t thread_id;
    std::string *subject_name_ = new std::string(subject_name);
    pthread_create(&thread_id, NULL, loadCalibThread, subject_name_);
    return false;
  }

  pthread_mutex_unlock(&calib_set_mutex);

  return it->second;
}

static void subject_publish_callback(
    const vicon_driver::ViconDriver::Subject &subject)
{
  static std::set<std::string> defined_msgs;

  std::string msgname = "vicon_" + subject.name;
  if(defined_msgs.find(msgname) == defined_msgs.end())
  {
    if(pthread_mutex_trylock(&ipc_mutex) == 0)
    {
      IPC_defineMsg(msgname.c_str(), IPC_VARIABLE_LENGTH,
                    ViconSubject::getIPCFormat());
      pthread_mutex_unlock(&ipc_mutex);
      defined_msgs.insert(msgname);
    }
    else
      return;
  }

  if(loadCalib(subject.name))
  {
    Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
    current_pose.translate(Eigen::Vector3d(subject.translation));
    current_pose.rotate(Eigen::Quaterniond(subject.rotation));
    current_pose = current_pose*calib_pose[subject.name];
    const Eigen::Vector3d position(current_pose.translation());
    const Eigen::Quaterniond rotation(current_pose.rotation());

    ViconSubject subject_ipc;
    subject_ipc.time_sec = subject.time_usec/1000000; // Integer division
    subject_ipc.time_usec = subject.time_usec - subject_ipc.time_sec*1000000;
    subject_ipc.frame_number = subject.frame_number;
    subject_ipc.name = const_cast<char*>(subject.name.c_str());
    subject_ipc.occluded = subject.occluded;
    subject_ipc.position[0] = position.x();
    subject_ipc.position[1] = position.y();
    subject_ipc.position[2] = position.z();
    subject_ipc.orientation[0] = rotation.x();
    subject_ipc.orientation[1] = rotation.y();
    subject_ipc.orientation[2] = rotation.z();
    subject_ipc.orientation[3] = rotation.w();
    subject_ipc.num_markers = subject.markers.size();
    subject_ipc.markers = new ViconMarker[subject_ipc.num_markers];
    for(int i = 0; i < subject_ipc.num_markers; i++)
    {
      subject_ipc.markers[i].name =
          const_cast<char*>(subject.markers[i].name.c_str());
      subject_ipc.markers[i].subject_name =
          const_cast<char*>(subject.markers[i].subject_name.c_str());
      subject_ipc.markers[i].position[0] = subject.markers[i].translation[0];
      subject_ipc.markers[i].position[1] = subject.markers[i].translation[1];
      subject_ipc.markers[i].position[2] = subject.markers[i].translation[2];
      subject_ipc.markers[i].occluded = subject.markers[i].occluded;
    }
    if(pthread_mutex_trylock(&ipc_mutex) == 0)
    {
      IPC_publishData(msgname.c_str(), &subject_ipc);
      pthread_mutex_unlock(&ipc_mutex);
    }
    delete[] subject_ipc.markers;
  }
}

static void unlabeled_markers_publish_callback(
  const vicon_driver::ViconDriver::Markers &markers)
{
  static bool msg_defined = false;
  std::string msgname = "vicon_unlabeled_markers";

  if(!msg_defined)
  {
    if(pthread_mutex_trylock(&ipc_mutex) == 0)
    {
      IPC_defineMsg(msgname.c_str(), IPC_VARIABLE_LENGTH,
                    ViconMarkers::getIPCFormat());
      pthread_mutex_unlock(&ipc_mutex);
      msg_defined = true;
    }
    else
      return;
  }

  ViconMarkers markers_ipc;
  markers_ipc.time_sec = markers.time_usec/1000000; // Integer division
  markers_ipc.time_usec = markers.time_usec - markers_ipc.time_sec*1000000;
  markers_ipc.frame_number = markers.frame_number;
  markers_ipc.num_markers = markers.markers.size();
  markers_ipc.markers = new ViconMarker[markers_ipc.num_markers];
  for(int i = 0; i < markers_ipc.num_markers; i++)
  {
    markers_ipc.markers[i].name =
      const_cast<char*>(markers.markers[i].name.c_str());
    markers_ipc.markers[i].subject_name =
      const_cast<char*>(std::string("").c_str());
    markers_ipc.markers[i].position[0] = markers.markers[i].translation[0];
    markers_ipc.markers[i].position[1] = markers.markers[i].translation[1];
    markers_ipc.markers[i].position[2] = markers.markers[i].translation[2];
    markers_ipc.markers[i].occluded = markers.markers[i].occluded;
  }
  if(pthread_mutex_trylock(&ipc_mutex) == 0)
  {
    IPC_publishData(msgname.c_str(), &markers_ipc);
    pthread_mutex_unlock(&ipc_mutex);
  }
  delete[] markers_ipc.markers;
}

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    std::cout << "Usage: " << argv[0] << " <vicon_server> <calib_files_dir>" <<
      std::endl;
    return 1;
  }

  const std::string vicon_hostname = std::string(argv[1]);

  calib_files_dir = std::string(argv[2]);

  if(IPC_connect("vicon") != IPC_OK)
  {
    std::cerr << "Error connecting to IPC" << std::endl;
    return 1;
  }

  vicon_driver::ViconDriver vd;
  if(vd.init(vicon_hostname))
  {
    vd.setSubjectPubCallback(subject_publish_callback);
    vd.setUnlabeledMarkersPubCallback(unlabeled_markers_publish_callback);
    vd.enableUnlabeledMarkerData(true);
    running = vd.start();
  }
  else
  {
    std::cerr << "Error connecting to vicon server" << std::endl;
  }

  if(running)
  {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
  }

  while(running)
  {
    std::cout << "Running: " << running << std::endl;
    if(pthread_mutex_trylock(&ipc_mutex) == 0)
    {
      IPC_handleMessage(0);
      pthread_mutex_unlock(&ipc_mutex);
    }
    sleep(1);
  }

  std::cout << "Shutting down ViconDriver" << std::endl;
  vd.shutdown();

  std::cout << "Disconnecting from IPC server" << std::endl;
  IPC_disconnect();

  return 0;
}
