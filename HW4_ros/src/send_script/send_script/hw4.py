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

################################################################

# import cv2
import math
import numpy as np
from skimage import measure as skiMeasure
from skimage import color
import matplotlib.pyplot as plt

# Parameter
filtSize = 5
threshold = 40 # ?
kernelSize = 10
org_x = 661
org_y = 785
cube_threshold = 5000
pxl2mm = 25 / math.sqrt(7743.5)

def img_preprocess(orginalImg):
    img = cv2.cvtColor(orginalImg, cv2.COLOR_RGB2GRAY) # BGR2GRAY

    # filter
    filt = np.ones((filtSize, filtSize), np.float32) / (filtSize**2)
    img = cv2.filter2D(img.astype('float32'), -1, filt, borderType = cv2.BORDER_CONSTANT)

    # to binary
    img = (img > threshold).astype('uint8')

    # clear border, imclearborder
    # image open and close
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize)) # MORPH_ELLIPSE, MORPH_CROSS, MORPH_RECT
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    
    # regionprops
    labels = skiMeasure.label(img, connectivity = 2)
    return labels

def visualize(img):
    labels = img_preprocess(img)
    # visualize
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4))
    ax1.imshow(img, plt.cm.gray, interpolation='nearest')
    ax1.axis('off')
    dst=color.label2rgb(labels)  # label colors to different groups
    ax2.imshow(dst,interpolation='nearest')
    ax2.axis('off')

    regions = skiMeasure.regionprops(labels)
    for region in regions:
        if region.area < cube_threshold:
            continue
        print("Centroid",region.centroid)
        y0, x0 = region.centroid
        orientation = region.orientation
        print("Degree", 180*orientation/math.pi)
        x1 = x0 + math.cos(orientation) * 0.5 * region.minor_axis_length
        y1 = y0 - math.sin(orientation) * 0.5 * region.minor_axis_length
        x2 = x0 - math.sin(orientation) * 0.5 * region.minor_axis_length
        y2 = y0 - math.cos(orientation) * 0.5 * region.minor_axis_length

        ax2.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
        ax2.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
        ax2.plot(x0, y0, '.g', markersize=15)

    fig.tight_layout()
    plt.show()

def cube_locate(img):
    visualize(img)
    region_cnt = 0
    cubes = []
    labels = img_preprocess(img)
    regions = skiMeasure.regionprops(labels)
    for region in regions:
        if region.area < cube_threshold:
            continue
        region_cnt += 1
        y0, x0 = region.centroid
        orientation = region.orientation
        dstx = (x0 - org_x) * pxl2mm
        dsty = (y0 - org_y) * pxl2mm
        dst_rot = 180*orientation/math.pi

        # rotation mapping
        while abs(dst_rot) > 45:
            dst_rot = dst_rot + 90 if dst_rot < 0 else dst_rot - 90
        cubes.append([dstx, dsty, dst_rot])
    print("regions number: ", region_cnt)
    if region_cnt == 0:
        return None
    else:
        # TODO
        # try to find the optimal cube to grab
        print(f"picX: {dstx}, pixY: {dsty}, picRot: {dst_rot}")
        return dstx, dsty, dst_rot

################################################################

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
