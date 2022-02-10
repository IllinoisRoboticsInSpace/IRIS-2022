import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy

import serial

class controllerToSerial(Node):

    def __init__(self):
        self.number = 10
        super().__init__('controllerToSerial')

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10)
        self.publisher_ = self.create_publisher(String, 'controller', 10)
        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1)
        self.working = False
        self.button0_prev = False
        self.power = 0


    def joystick_callback(self, msg: Joy):
        if(msg.buttons[0] and not self.button0_prev):
            if(self.working):
                self.working = False
                returnMsg = String()
                returnMsg.data = 'Not Currently Working'
                self.publisher_.publish(returnMsg)
            else:
                self.working = True
                returnMsg = String()
                returnMsg.data = 'Working'
                self.publisher_.publish(returnMsg)
        self.button0_prev = msg.buttons[0]
        if(self.working and msg.axes[1]!=0):
            self.power = int(msg.axes[1] * 30)
            returnMsg = String()
            returnMsg.data = 'Working: power = ' + str(self.power)
            self.publisher_.publish(returnMsg)
            if(self.power < 0):
                self.arduino.write(bytes(128 + self.power))
            else:
                self.arduino.write(bytes(self.power))
        elif(self.working and msg.axes[1]==0 and self.power!=0):
            self.power = 0
            returnMsg = String()
            returnMsg.data = 'Working: power = ' + str(self.power)
            self.publisher_.publish(returnMsg)
            self.arduino.write(bytes(self.power))



def main(args=None):
    rclpy.init(args=args)

    contro = controllerToSerial()
    try:
        rclpy.spin(contro)
    except KeyboardInterrupt:
        pass
    finally:
        contro.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()