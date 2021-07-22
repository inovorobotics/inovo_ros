# High level motion control

Once you have configured the [network](NETWORK.md), you should try controlling the robot. Use the UI to configure the robot as normal, making sure to configure the workspace correctly. Power on the unit, reset safe/estop and enable the arm.

Make sure to export the correct `ROS_MASTER_URI` environment variable and run the following to inspect the ROS topics available:

```
$ rostopic list
...snip...
/default_move_group/move/cancel
/default_move_group/move/feedback
/default_move_group/move/goal
/default_move_group/move/result
/default_move_group/move/status
/default_move_group/move_group_info
...snip...
```

You should be able to see the "action server" at `/default_move_group/move`. You can use this action server to control the arm at a high level using "move to here" type commands.

In order move the arm to a pose, use the following command:

!!!WARNING, check the arm can move before running this command, and make sure to always hold the estop or the safe stop while controlling the arm!!!
```
rostopic pub /default_move_group/move/goal virtual_robot/MotionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  motion_sequence:
  - pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
    frame_id: ''
    tcp_id: ''
    relative: false
    use_joint_space_target: true
    use_nearest_joint_space_target: false
    joint_names: ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    joint_angles: [0,0,0,0,0,0]
    unwind_joint_target: false
    max_velocity: {linear: 0.0, angular: 0.0}
    max_joint_velocity: 0.0
    max_joint_acceleration: 0.0
    blend: {linear: 0.0, angular: 0.0}
    cartesian: false
  end_effector: ''
  ignore_scaling: false"
```
