#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from bluerov_1.msg import MotorMovement

class BlueROVAxisOutputInfo:
    """docstrAxisOutputInfo"""
    def __init__(self):
        self.direction = MotorMovement()
        self.direction.xr = 0.0
        self.direction.xl = 0.0
        self.direction.y = 0.0
        self.direction.zfr = 0.0
        self.direction.zfl = 0.0
        self.direction.zb = 0.0

    def convert2servo(self, joyInput):
        if joyInput.axes[1] == 0.0 and joyInput.axes[3] == 0.0:
            self.direction.xr = 0
            self.direction.xl = 0
        elif joyInput.axes[1] != 0.0 and joyInput.axes[3] == 0.0:
            self.direction.xr = joyInput.axes[1]
            self.direction.xl = joyInput.axes[1]
        elif joyInput.axes[1] == 0 and joyInput.axes[3] != 0.0:
            self.direction.xr = joyInput.axes[3]
            self.direction.xl = -joyInput.axes[3]
        else:
            self.direction.xr = (joyInput.axes[1] + joyInput.axes[3])/2
            self.direction.xl = (joyInput.axes[1] - joyInput.axes[3])/2

        self.direction.y = -joyInput.axes[0]
        self.direction.zfr = -joyInput.axes[6]/2
        self.direction.zfl = -joyInput.axes[6]/2
        self.direction.zb = -joyInput.axes[6]
        return

def callback(msg):
    ServoOutput.convert2servo(msg)

    print('ServoOutput.direction {0} {1} {2} {3} {4} {5}'.format(ServoOutput.direction.xr,ServoOutput.direction.xl,ServoOutput.direction.y,ServoOutput.direction.zfr,ServoOutput.direction.zfl,ServoOutput.direction.zb))
    print('')

    pub.publish(ServoOutput.direction)

ServoOutput = BlueROVAxisOutputInfo()
rospy.init_node('topic_joy2servo_convert', anonymous=True)
rospy.Subscriber('joy', Joy, callback)
pub = rospy.Publisher('servo_signal', MotorMovement)

rospy.spin()
