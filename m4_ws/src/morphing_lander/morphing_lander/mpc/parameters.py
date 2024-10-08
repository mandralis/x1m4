import numpy as np
from os import getenv

# declare parameter dictionary
params_ = {}

# high level parameters
params_['Ts']                    = 0.007                          # control frequency of MPC
params_['Ts_tilt_controller']    = params_.get('Ts')              # control frequency of TiltController
params_['Ts_drive_controller']   = params_.get('Ts')              # control frequency of DriveController
params_['queue_size']            = 1                              # queue size of ros2 messages

# roboclaw addresses
params_['tilt_roboclaw_address']         = "/dev/ttyACM1"
params_['drive_roboclaw_address']        = "/dev/ttyACM0"
params_['max_duty']                      = 32767

# rc inputs
params_['min']                           = 1094
params_['max']                           = 1934
params_['dead']                          = 1514

# rc switch numbers
params_['tilt_channel']        = 9
params_['encoder_channel']     = 11
params_['arm_channel']         = 6
params_['manual_channel']      = 7
params_['drive_speed_channel'] = 1
params_['turn_speed_channel']  = 0


