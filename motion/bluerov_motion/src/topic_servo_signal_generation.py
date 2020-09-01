#!/usr/bin/env python
import rospy

import time
from bluerov_motion.msg import MotorMovement
from bluerov_motion.msg import InputSignal

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=1)

#servo_min = 150  # Min pulse length out of 4096
#servo_max = 600  # Max pulse length out of 4096
servo_min = 250
servo_max = 500

servo_moving_area = servo_max - servo_min
servo_middle = servo_min + servo_moving_area / 2

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

class ServoInputDirecton:
    def __init__(self):
        self.Signal = InputSignal() #signal for servo motor
        self.Signal.xr_int  = 0
        self.Signal.xl_int  = 0
        self.Signal.y_int   = 0
        self.Signal.zfr_int = 0
        self.Signal.zfl_int = 0
        self.Signal.zb_int  = 0

    def convert2signal(self, MotorMovement):
         self.Signal.xr_int  = int(servo_middle + servo_moving_area * MotorMovement.xr / 2)
         self.Signal.xl_int  = int(servo_middle - servo_moving_area * MotorMovement.xl / 2)
         self.Signal.y_int   = int(servo_middle + servo_moving_area * MotorMovement.y / 2)
         self.Signal.zfr_int = int(servo_middle - servo_moving_area * MotorMovement.zfr / 2)
         self.Signal.zfl_int = int(servo_middle + servo_moving_area * MotorMovement.zfl / 2)
         self.Signal.zb_int  = int(servo_middle + servo_moving_area * MotorMovement.zb / 2)

def callback(msg):
    ServoSignal.convert2signal(msg)

    print('servo signal check {0} {1} {2} {3} {4} {5}'.format(ServoSignal.Signal.xr_int,ServoSignal.Signal.xl_int,ServoSignal.Signal.y_int,ServoSignal.Signal.zfr_int,ServoSignal.Signal.zfl_int,ServoSignal.Signal.zb_int))
    print('')

ServoSignal = ServoInputDirecton()
rospy.init_node('topic_servo_signal_generation', anonymous=True)
rospy.Subscriber('servo_signal',MotorMovement,callback)

#rospy.spin()
while True:
    print('set pwm here')
    pwm.set_pwm(0, 0, ServoSignal.Signal.xr_int)
    pwm.set_pwm(1, 0, ServoSignal.Signal.xl_int)
    pwm.set_pwm(5, 0, ServoSignal.Signal.y_int)
    pwm.set_pwm(2, 0, ServoSignal.Signal.zfr_int)
    pwm.set_pwm(3, 0, ServoSignal.Signal.zfl_int)
    pwm.set_pwm(4, 0, ServoSignal.Signal.zb_int)

    #time.sleep(1)
