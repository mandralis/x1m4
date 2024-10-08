import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc

# roboclaw and jetson
from morphing_lander.mpc.DriveControllerBase import DriveControllerBase
from morphing_lander.mpc.roboclaw_3 import Roboclaw
from morphing_lander.mpc.parameters import params_

from IPython import embed

# get parameters
min                    = params_.get('min')
max                    = params_.get('max')
dead                   = params_.get('dead')
Ts                     = params_.get('Ts_drive_controller')
drive_roboclaw_address = params_.get('drive_roboclaw_address')
max_duty               = params_.get('max_duty')
arm_channel            = params_.get('arm_channel')
manual_channel         = params_.get('manual_channel')
drive_speed_channel    = params_.get('drive_speed_channel')
turn_speed_channel     = params_.get('turn_speed_channel')

class DriveControllerHardware(DriveControllerBase):
    def __init__(self): 
        super().__init__()

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        # initialize drive speed and turn speed
        self.drive_speed_in = dead
        self.turn_speed_in  = dead

        # msg receive time
        self.time_received = 0.0

        # Manual vs automatic control
        self.manual = True

        # armed 
        self.armed  = False

        # roboclaw stuff
        self.address = 0x80
        self.rc = Roboclaw(drive_roboclaw_address,115200)
        self.rc.Open()

        # time initialization
        self.init_time = False
        self.t0 = 0.0
        self.t0_pixhawk = 0.0

    def rc_listener_callback(self, msg):
        if not self.init_time:
            self.t0_pixhawk = msg.timestamp
            self.t0         = self.get_clock().now().nanoseconds
            self.init_time = True
        self.time_received   = msg.timestamp
        self.drive_speed_in  = msg.values[drive_speed_channel]
        self.turn_speed_in   = msg.values[turn_speed_channel]

        # set manual or automatic control of tilt angle
        if msg.values[manual_channel] == max:
            self.manual = False
        else:
            self.manual = True

        # check if armed
        if msg.values[arm_channel] == max:
            self.armed = True
        else:
            self.armed = False

    def normalize(self,drive_speed_in):
        return (drive_speed_in-dead)/(max-dead)

    def map_speed(self,speed_normalized):
        return int(max_duty*speed_normalized)

    def on_shutdown(self):
        self.rc._port.close()
        self.get_logger().info("port closed !")

    def move_right_wheel(self, speed):
        if not self.armed:
            print(f"right wheel speed: {speed}")
            if speed > 0:
                if speed >= max_duty:
                    speed = max_duty-1
                self.rc.DutyM1(self.address, -abs(speed))
            else:
                if speed <= -max_duty:
                    speed = -(max_duty-1)
                self.rc.DutyM1(self.address, abs(speed))

    def move_left_wheel(self, speed):
        if not self.armed:
            print(f"left wheel speed: {speed}")
            if speed > 0:
                if speed >= max_duty:
                    speed = max_duty
                self.rc.DutyM2(self.address, abs(speed))
            else:
                if speed <= -max_duty:
                    speed = -(max_duty)
                self.rc.DutyM2(self.address, -abs(speed))

    def update(self):
        # get current time
        current_time = (self.get_clock().now().nanoseconds - self.t0)/1e9

        time_received = (self.time_received - self.t0_pixhawk)/1e6

        # get message age
        msg_age      = current_time - time_received

        # print(f"msg_age: {msg_age}")
        
        # print current time and time received
        # print(f"current_time: {current_time}")
        # print(f"time_received: {time_received}")

        print(f"[out] drive,turn:{self.drive_speed_in},{self.turn_speed_in}")

        # if msg_age > 0.5 seconds then set drive and turn speed to zero
        if (msg_age > 0.5):
            self.drive_speed_in = dead
            self.turn_speed_in  = dead
            # print(f"[in] drive,turn:{self.drive_speed_in},{self.turn_speed_in}")

        motion_gain  = 1.0
        turning_gain = 1.0
        if (self.manual):
            # manual control of driving
            lin_vel = -self.map_speed(motion_gain*self.normalize(self.drive_speed_in))
            ang_vel = -self.map_speed(turning_gain*self.normalize(self.turn_speed_in))

            self.get_logger().info(f"lin_vel, ang_vel: ({lin_vel},{ang_vel})")

            self.move_right_wheel(lin_vel - ang_vel)
            self.move_left_wheel(lin_vel  + ang_vel)

        else:
            # automatic control of driving
            lin_vel = -self.map_speed(self.drive_speed)
            ang_vel = self.map_speed(self.turn_speed)
            self.move_right_wheel(lin_vel - ang_vel)
            self.move_left_wheel(lin_vel + ang_vel)
     
def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveControllerHardware()
    drive_controller.get_logger().info("Starting DriveControllerHardware node...")
    rclpy.spin(drive_controller)
    drive_controller.on_shutdown()  # do any custom cleanup
    drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
