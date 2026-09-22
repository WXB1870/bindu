# Internal external-simulator device port. SI units, system-clock source stamp.
# Commands are addressed to one simulator boot and one executor, never hardware.
builtin_interfaces/Time stamp
string simulator_id
string sender_id
uint64 sequence
string profile_hash
float64 valid_for
string operation
string resource_group
string[] joint_names
float64[] positions
geometry_msgs/Twist velocity
