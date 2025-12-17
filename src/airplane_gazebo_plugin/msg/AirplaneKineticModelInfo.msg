# TIme header of the message:
float64 real_time
float64 sim_time
std_msgs/Header header

# Position of the UAV
float64 north
float64 east
float64 up

# Velocity in The Earth Frame:
float64 v_north
float64 v_east
float64 v_up

# Orientation using the MIT axes system:
float64 course
float64 fpa
float64 roll

# Angular velocity for the kinematic model;
float64 roll_speed

# Body axes x velocity
float64 velocity_a