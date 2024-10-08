from roboclaw_3 import Roboclaw

address = 0x80
rc = Roboclaw("/dev/ttyACM0",115200)
rc.Open()

rc.DutyM1(address, 32767)
