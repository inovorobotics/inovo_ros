# Cartesian target
geometry_msgs/Pose pose
string frame_id

# If this is blank, "tool0" is used instead, which is always there
string tcp_id
bool relative

# If true, use the joint space target below - pose is ignored
bool use_joint_space_target
bool use_nearest_joint_space_target

# If joint names is empty but joint angles contains entries, the joint angles will be applied in chain order
# If both arrays contain entries, they must be the same size as joint names are angles are matched up by index
string[] joint_names
float64[] joint_angles

# Find the shortest path to joint space targets rather than using the absolute angle
bool unwind_joint_target

# Velocity profile settings
LinAng max_velocity
float64 max_joint_velocity
float64 max_joint_acceleration

# Blend settings
LinAng blend

# True if cartesian interpolation is to be used, otherwise joint interpolation is used instead
bool cartesian
