#!/bin/sh

# Create a new tmux session named "logging"
tmux new-session -d -s logging

tmux send-keys -t logging 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t logging 'source /home/x1m4/x1m4/m4_ws/install/setup.bash' C-m
tmux send-keys -t logging 'cd /home/x1m4/x1m4/logs/' C-m
tmux send-keys -t logging 'ros2 bag record \
        /rosout \
        /drive_vel \
        /tilt_vel \
        /fmu/out/input_rc \
        /fmu/out/battery_status \
        /fmu/out/vehicle_odometry \
        /fmu/out/sensor_combined \
        /fmu/out/vehicle_status \
	/fmu/in/vehicle_thrust_setpoint \
        -d 300' C-m






 

