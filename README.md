[![Build Status](https://travis-ci.org/KumarRobotics/vicon.svg?branch=master)](https://travis-ci.org/KumarRobotics/vicon)

# Vicon Driver

The driver consists of 2 parts:

* `vicon_driver`: This contains a base ViconDriver class which handles all the commmunication with the vicon PC and has hooks for the subject/unlabeled markers publish callbacks (see ViconDriver.h)
* Interface layer: (IPC & ROS for now) Hooks into the callbacks supplied by the ViconDriver class and actually publishes the message

The `vicon_driver` design is intended to provide flexibility in terms of supporting any interface, you just need to provide the ViconDriver class with callback functions which will be called with the vicon data structures as their arguments. This design is not the most efficient in terms of CPU usage since you need to convert the data provided by the ViconDriver class into whatever format the interface layer needs to publish, but I believe that the flexibility it provides outweighs the extra CPU cycles required.

There is also an implementation of loading/storing calib (zero pose) files in YAML format using `yaml-cpp` (see ViconCalib.h). Loading calib files automatically is implemented in both the interface layes (IPC & ROS) but the ROS interface layer also provides a service which you can call to set the zero pose and automatically save it in the calib file.

## License

`vicon_driver` and the IPC interface are licensed under the Apache-2.0 license.

The ROS interface is licensed under the BSD-3-Clause license.

## Compiling

### ROS
First, add the ros folder in this repository to your catkin workspace. Then, run

```
catkin_make
```

## Example usage

### ROS
Check the launch files in the vicon and `vicon_odom` packages. The output from the vicon node has a lot more information but most likely you'll want to use the `vicon_odom` package which generates odometry information from the position and orientation provided by Vicon.

#### Getting Odometry Message from Vicon

* Launch the `vicon.launch` file in `vicon` package, make sure you set the `vicon_server` correctly

  ```
  roslaunch vicon vicon.launch
  ````

* For each of the model you have, launch `vicon_odom.launch` in `vicon_odom` package. Note that every `vicon_odom` you launched will now be run under the namespace `vicon`. For example, if your model name is `obj1`, then the `vicon_odom` node will be named `vicon/obj1`, the published odometry message will be named `vicon/obj1/odom`. Also, you can provide a `child_frame_id` for the published odometry message, if not, it will be defaulted to `model`.

  ```
  roslaunch vicon_odom vicon_odom.launch model:=<model> child_frame_id:=<frame> publish_tf:=<bool>
  ```

#### Calibrating a Model

* Launch the `calibrate.launch` file in `vicon/launch` using your ViconModelName

    ```
    roslaunch vicon calibrate.launch model:=ViconModelName
    ```

* In a new terminal, echo the `zero_pose` estimate from vicon. **Note:** you will not see anything yet.

    ```
    rostopic echo /vicon_calibrate/zero_pose
    ```

* In another terminal, toggle the calibration routine:

    ```
    rosservice call /vicon_calibrate/toggle_calibration
    ```

* Now, check to make sure the `zero_pose` provides reasonable values
* Untoggle the calibration routine

    ```
    rosservice call /vicon_calibrate/toggle_calibration
    ```

* Close the running launch files and verify that a new calibration file was written to `./vicon/calib`
