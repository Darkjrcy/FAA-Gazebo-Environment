# This contains the position and orientation of a point in free space
geometry_msgs/Pose pose

# This contains the linear and angular velocity of the Velocity
geometry_msgs/Twist velocity

# This conatins the time variables:
float64 real_time
float64 sim_time

# This variable contains the Confidence level and Detection status:
float64 confidence_level

# This variable contians the %CPU and the MemoryUsageGB
float64 perc_cpu
float64 memory