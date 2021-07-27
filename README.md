# inovo_ros

This repository contains message definitions and Python client libraries which can be used to control the Inovo Modular Robot arm.

## Instructions for use

Install ROS on a ROS compatible machine. We recommend using Ubuntu 20.04 and ROS Noetic, and the following tutorials are oriented towards this system.
While it is possible to use ROS on Windows, this has not been tested, and your mileage may vary.

### Setting up

Create a new ROS workspace using the following commands (this will create a directory called  `inovo_ws` in your home directory):

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

### Network configuration
See [Network](docs/NETWORK.md) for instructions on how to configure your machine to be able to communicate with the Inovo robot over the network.

### Running a basic example

Make sure to source the workspace.
```
source ~/inovo_ws/devel/setup.bash
```

Run an example which moves the robot arm to various pre-configured locations:
```
rosrun commander_api move_example.py
```
