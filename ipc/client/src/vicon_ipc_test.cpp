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

#include <stdio.h>
#include <stdlib.h>
#include <ipc.h>
#include "msgs/ViconTypes.h"

void subject_callback(MSG_INSTANCE msg_instance, void *msg, void *client_data)
{
  ViconSubject *subject = reinterpret_cast<ViconSubject*>(msg);
  printf("Frame time: %ld usec, frame number: %d, subject name: %s, occluded: %d, "
      "position: (%f, %f, %f), orientation: (%f, %f, %f, %f), num_markers: %d\n",
      subject->time_sec*1000000UL + subject->time_usec, subject->frame_number,
      subject->name, subject->occluded, subject->position[0], subject->position[1],
      subject->position[2], subject->orientation[0], subject->orientation[1],
      subject->orientation[2], subject->orientation[3], subject->num_markers);
}

void unlabeled_markers_callback(MSG_INSTANCE msg_instance, void *msg, void *client_data)
{
  ViconMarkers *markers = reinterpret_cast<ViconMarkers*>(msg);
  printf("Unlabeled markers: frame time: %ld usec, frame_number: %d, num_markers: %d\n",
         markers->time_sec*1000000UL + markers->time_usec, markers->frame_number, markers->num_markers);
}

int main(int argc, char **argv)
{
  IPC_connect("vicon_client");

  IPC_subscribeData("vicon_QuadrotorHotel", subject_callback, NULL);
  IPC_subscribeData("vicon_unlabeled_markers", unlabeled_markers_callback, NULL);

  IPC_dispatch();

  return 0;
}
