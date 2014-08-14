[![Build Status](https://travis-ci.org/KumarRobotics/vicon.svg?branch=master)](https://travis-ci.org/KumarRobotics/vicon)

# Vicon Driver

The driver consists of 2 parts:

* libvicon\_driver: This contains a base ViconDriver class which handles all the commmunication with the vicon PC and has hooks for the subject/unlabeled markers publish callbacks (see ViconDriver.h)
* Interface layer: (IPC & ROS for now) Hooks into the callbacks supplied by the ViconDriver class and actually publishes the message

The libvicon\_driver design is intended to provide flexibility in terms of supporting any interface, you just need to provide the ViconDriver class with callback functions which will be called with the vicon data structures as their arguments. This design is not the most efficient in terms of CPU usage since you need to convert the data provided by the ViconDriver class into whatever format the interface layer needs to publish, but I believe that the flexibility it provides outweighs the extra CPU cycles required.

There is also an implementation of loading/storing calib (zero pose) files in YAML format using libyaml-cpp (see ViconCalib.h). Loading calib files automatically is implemented in both the interface layes (IPC & ROS) but the ROS interface layer also provides a service which you can call to set the zero pose and automatically save it in the calib file.

## License

libvicon\_driver and the IPC interface are licensed under the Apache-2.0 license.

The ROS interface is licensed under the BSD-3-Clause license.

## Compiling

### ROS
First, add the ros folder in this repository to your ROS\_PACKAGE\_PATH. Then, run

    rosmake vicon_odom vicon_odom

## Example usage

### ROS
Check the launch files in the vicon and vicon\_odom packages. The output from the vicon node has a lot more information but most likely you'll want to use the vicon\_odom package which generates odometry information from the position and orientation provided by Vicon.

#### Calibrating a Model

* Launch the calibrate.launch file in ./ros/vicon/launch using your ViconModelName

        roslaunch vicon calibrate.launch model:=ViconModelName

* In a new terminal, echo the zero\_pose estimate from vicon. **Note:** you will not see anything yet.

        rostopic echo /vicon_calibrate/zero_pose

* In another terminal, toggle the calibration routine:

        rosservice call /vicon_calibrate/toggle_calibration

* Now, check to make sure the zero\_pose provides reasonable values
* Untoggle the calibration routine

        rosservice call /vicon_calibrate/toggle_calibration

* Close the running launch files and verify that a new calibration file was written to ./ros/vicon/calib
