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

# rc inputs
params_['min']                           = 1094
params_['max']                           = 1934
params_['dead']                          = 1514

