uint32 schema_version
# Robot environment: simulation or real. Empty is legacy simulation only.
string runtime_mode
string command_id
string task_id
string observation_id
string profile_hash
string lease_id
uint64 epoch
string mode
string resource_group
# Schema 2 joint commands: ordered, disjoint profile groups. Empty for schema 1.
string[] resource_groups
builtin_interfaces/Time stamp
float64 valid_for
string[] joint_names
float64[] positions
trajectory_msgs/JointTrajectoryPoint[] points
geometry_msgs/Twist velocity
float64 duration
# Required current revision for joint_reference_segment; zero disables check otherwise.
uint64 expected_revision
