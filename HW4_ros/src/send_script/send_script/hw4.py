#!/usr/bin/env python
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from time import sleep

def cube_locate(img):
    if True:
        return None
    picX = 50
    picY = 50
    rot = 45
    return (picX, picY, rot)

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

    sleep(1)

class HW4(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)

        self.picBaseX = 250
        self.picBaseY = 250
        self.picBaseZ = 400
        # -180, 0 won't change
        self.picBasePsi = 135
        self.picBase = f"{self.picBaseX}, {self.picBaseY}, {self.picBaseZ}, -180, 0, {self.picBasePsi}"

        self.pickUpZ = 100

        self.stackX = 400
        self.stackY = 0
        self.stackZ = 100
        # -180, 0 won't change
        self.stackPsi = 135
        # self.stackBase = f"{self.stackX}, {self.stackY}, {self.stackZ}, -180, 0, {self.stackPsi}"

        self.currentX = self.picBaseX
        self.currentY = self.picBaseY
        # -180, 0 won't change
        self.currentPsi = self.picBasePsi

        self.backBase()

    def moveTo(self, pos):
        script = f"Line(\"CPP\",{pos},100,200,0,false)"
        send_script(script)

    def backBase(self):
        self.currentX = self.picBaseX
        self.currentY = self.picBaseY
        self.currentPsi = self.picBasePsi
        self.moveTo(self.picBase)
        set_io(0.0) # open gripper
        send_script("Vision_DoJob(job1)")

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg = data)
        # cv2.imwrite(f"output_{count}.png", image)
        pos = cube_locate(img)
        if pos is not None:
            [picX, picY, picRot] = pos
            
            self.currentX += picX / 2**0.5
            self.currentY -= picX / 2**0.5
            self.currentX -= picY / 2**0.5
            self.currentY -= picY / 2**0.5
            self.currentPsi += picRot

            skyHigh = f"{self.currentX}, {self.currentY}, {self.picBaseZ}, -180, 0, {self.currentPsi}"
            self.moveTo(skyHigh)
            set_io(0.0) # open gripper

            groundLow = f"{self.currentX}, {self.currentY}, {self.pickUpZ}, -180, 0, {self.currentPsi}"
            self.moveTo(groundLow)
            set_io(1.0) # close gripper

            self.moveTo(skyHigh)
            stackSky = f"{self.stackX}, {self.stackY}, {self.picBaseZ}, -180, 0, {self.stackPsi}"
            self.moveTo(stackSky)
            stackBase = f"{self.stackX}, {self.stackY}, {self.stackZ}, -180, 0, {self.stackPsi}"
            self.moveTo(stackBase)
            set_io(0.0) # open gripper
            self.moveTo(stackSky)
            self.stackZ += 25 # block height

            self.backBase()
            return
        else:
            print("Task done")
            raise self.TaskDoneException

    class TaskDoneException(Exception):
        # code exploit
        # not a good habit
        def __init__(self):
            super().__init__("Robot task is done")


def main(args=None):
    rclpy.init(args=args)
    node = HW4('hw4')
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
