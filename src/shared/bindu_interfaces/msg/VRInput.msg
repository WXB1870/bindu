# Pose is row-major OpenXR SE(3), metres. Stamp is receiver event time.
# source_id changes on browser reconnect or WebXR tracking reset.
# seq increases only on input events.
builtin_interfaces/Time stamp
string source_id
string runtime_mode
uint64 seq
string side
float64[16] pose
float64 grip
float64 trigger
bool init
bool stop
bool valid
uint64 clutch_seq

# Explicit input mapping; empty retains legacy v3.4 consumers.
string mapping
uint8 buttons
# primary press/release, secondary press/release, stick press/release, trigger press/release.
uint64[8] button_edges
bool virtual_grip

# WebXR thumbstickValue (x,y), each in [-1,1]. Empty means unavailable.
float64[] stick

# Actual WebXR viewer pose, row-major, in the controller reference space.
# Empty means unavailable. Never substitute a desktop camera or controller.
# stamp is first receiver observation; age_s is browser age at that observation.
# source_id follows the same input/projection identity as this controller.
float64[] head_pose
builtin_interfaces/Time head_stamp
float64 head_age_s
string head_source_id
uint64 head_seq
