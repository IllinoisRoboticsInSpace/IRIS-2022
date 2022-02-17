import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy

import serial

class controllerToSerial(Node):

    def __init__(self):
        super().__init__('controllerToSerial')

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10) #subscribes to Joy topic

        self.publisher_ = self.create_publisher(String, 'controller', 10) #publishes to 'controller' topic

        self.arduino = serial.Serial(port = '/dev/ttyACM0',baudrate = 9600, timeout = .1) #initializes arduino serial port. make sure baudrate is same for arduino and python code

        self.working = False
        self.button0_prev = False
        self.numMotor = 1
        self.power = 0

    ####################################################################################################

    #IMPORTANT FUNCTIONS:
    def power_to_bytes(self) -> bytes:      #converts self.power into bytes
        x = abs(self.power) + 256 * self.numMotor + 256 * 16
        #x is two bits, power must be between -127 and 127. probably.
        #first byte should be # of motor + b'00010000 <- makes sure non-inputs are not read as power (similar to CRC)
        #second byte is power (most significant bit is negative sign)
        if(self.power < 0):
            x += 128
        return x.to_bytes((x.bit_length() + 7) // 8, 'big')
    def sendPower(self):
        self.arduino.write(self.power_to_bytes()) #writes power to serial0

    ####################################################################################################
    def limitPower(self):
        if(self.power > 127):
            self.power = 127
        elif(self.power < -127):
            self.power = -127
    def display_power(self):
        returnMsg = String()
        returnMsg.data = 'Working: power = ' + str(self.power)
        self.publisher_.publish(returnMsg)

    def joystick_callback(self, msg: Joy):
        if(msg.buttons[0] and not self.button0_prev):   # 'A' switches between functioning and non-functiong
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
        self.button0_prev = msg.buttons[0]              #makes sure above code runs only once

        if(self.working and msg.axes[1]!=0): 

            self.power = int(msg.axes[1] * 60)  #Set the power to left-joystick y-axis * 60 
                                                #(joystick value is in range [-1,1])
            if(msg.buttons[2]):                 
                self.power *= 2                 # if 'X' is pressed, multiply power by 2 (for fun not necessary)
            
            self.limitPower()
            self.display_power()

            self.sendPower()
        elif(msg.axes[1]==0 and self.power!=0): #If left joystick y-axis = 0, and the power isn't already 0, set it to 0.
            self.power = 0
            self.display_power()
            self.sendPower()
        
        if(msg.buttons[1]):     #If 'B' is pressed, publish the current value of the arduino Serial port
            returnMsg = String()
            returnMsg.data = str(self.arduino.read())   
            self.publisher_.publish(returnMsg)

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
