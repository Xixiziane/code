# Chainwing hinge joint status for slave controller position trim
# Published by chainwing_slave module based on IMU integration
# or direct GZ joint state reading in simulation
# Hinge axis = X (forward) = relative ROLL between wing units

uint64 timestamp          # time since system start (microseconds)

float32 hinge_angle_left  # left hinge relative roll deflection from nominal (rad)
float32 hinge_angle_right # right hinge relative roll deflection from nominal (rad)
float32 hinge_rate_left   # left hinge angular rate (rad/s)
float32 hinge_rate_right  # right hinge angular rate (rad/s)

float32 trim_left         # computed elevon trim for left slave (normalized [-1, 1])
float32 trim_right        # computed elevon trim for right slave (normalized [-1, 1])

bool data_valid           # true when hinge data is available and trustworthy
