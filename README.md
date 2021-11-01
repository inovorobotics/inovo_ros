# inovo_ros

This repository contains message definitions and Python client libraries which can be used to control the Inovo robot arm from a remote machine using ROS.

> Warning: This repository is still undergoing heavy development is likely to change. Please bear this in mind when writing software which depends on it.

## Requirements

First, install ROS on a compatible machine. We recommend using Ubuntu 20.04 with ROS Noetic on an x86 PC, thus, the following information is only tested on this setup. Instructions on how to do this can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

While it is possible to use ROS on Windows, this has not been tested, and your mileage may vary.

## Setting Up

Create a new ROS workspace and clone this repository using the following commands (this will create a directory called  `inovo_ws` in your home directory):

```
mkdir -p ~/inovo_ws/src
cd ~/inovo_ws/src
git clone https://github.com/inovorobotics/inovo_ros.git
cd ..
```

Make sure to source ROS:
```
source /opt/ros/noetic/setup.bash
```

Next, install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

Then, build your workspace with catkin:
```
catkin_make
```

## Network configuration
See [Network](docs/NETWORK.md) for instructions on how to configure your machine to be able to communicate with the Inovo robot.

## Running a basic example

Make sure to source the workspace.
```
source ~/inovo_ws/devel/setup.bash
```

Run an following exmaple `move_example.py` moves the robot arm to various pre-configured locations.

> Warning: This will move the Inovo robot. Make sure to hold the estop while moving the robot.
```
rosrun commander_api move_example.py
```

Take a look at `move_example.py` using your favourite text editor to find out how to move the robot.

This repository only contains a minimal set of examples, more are to follow soon!

## Visualization
> Note: This package has now moved to https://github.com/inovorobotics/inovo_ros_viz.

If you wish to visualize the robot, you can do so with the `inovo_viz` package which includes some scripts to show the various different robot sizes.

After building and sourcing your workspace, run one of the following launch files to visualize either the short, medium and long robot "preset" configurations. This should open a `joint_state_publisher_gui` instance which allows you to control the joint angles, and an `rviz` instance to show the robot.

```
roslaunch inovo_viz emulate_short.launch
roslaunch inovo_viz emulate_med.launch
roslaunch inovo_viz emulate_long.launch
```

You can find URDFs for the "preset" configurations in the [inovo_description/urdf/examples](inovo_description/urdf/examples) directory.
