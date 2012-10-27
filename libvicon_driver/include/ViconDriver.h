#ifndef __VICON_DRIVER_H__
#define __VICON_DRIVER_H__

#include <stdint.h>
#include <string>
#include <vector>
#include <pthread.h>

namespace ViconDataStreamSDK
{
namespace CPP
{
class Client;
}
}

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
  void processUnlabeledMarkers(int64_t frame_time_usec, unsigned int frame_number);

  ViconDataStreamSDK::CPP::Client *client_;
  bool connected_, running_;
  SubjectPubCallback_t subject_callback_;
  MarkersPubCallback_t unlabeled_markers_callback_;
  pthread_t grab_thread_;
  bool grab_frames_;
  bool unlabeled_marker_data_enabled_;
};

#endif
