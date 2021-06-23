# inovo_ros

This repository contains messages and client libraries for controlling the Inovo Modular Robot arm.


## Instructions for use

Create a new ROS workspace:
```
mkdir -p ~/inovo_ws/src
cd ~/inovo_ws/src
git clone https://github.com/inovorobotics/inovo_ros.git
cd ..
```

Build the workspace:
```
catkin_make
```

## Running an example

Make sure to source the workspace.
```
source ~/inovo_ws/devel/setup.bash
```

Set the ROS_MASTER_URI environment variable to the hostname of your robot.

```
export ROS_MASTER_URI=http://<robot-hostname>:11311
```

You might need to set your machine's hostname and IP address in the robot's hosts file. (TODO)

Run an example motion:
```
rosrun commander_api move_example.py
```
