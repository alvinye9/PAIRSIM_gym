float32 desired_speed       # how fast we want to go
float32 exp_float           # unused
string description          # unused
bool kin_enable             # whether auto steering is enabled
bool long_enable            # whether auto throttle/brake is enabled
bool graceful_stop          # set to true to trigger graceful stop
bool emergency_stop         # set to true to trigger emergency stop
bool enable_overtaking      # are we allowed to overtake
bool take_pit_trajectory    # set to true to pit
bool enable_engine          # set to true to start engine, set to false to stop engine
bool enable_safety_override # set to true to disable checks from the nodes and modules specified in safety_node.cs
string desired_controller   # unused
string desired_planner      # unused
int32 flags                 # expansion field for quick implementation of more flags

# Control gains from basestation:
float32 kp_long
float32 ki_long
float32 kd_long
float32 kp_lat
float32 ki_lat
float32 kd_lat

string set_param_command
bool spawn_ghost_vehicle    # set to true to spawn, set to false to stop publishing
float32 ghost_vehicle_velocity # the desired velocity of the ghost vehicle
float32 ghost_vehicle_spawn_distance # the desired spawn distance of the ghost vehicle