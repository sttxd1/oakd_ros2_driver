# first, import all necessary modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

from pathlib import Path

import blobconverter
import cv2
import depthai as dai
import numpy as np

class OakDPublisher(Node):
    def __init__(self):
        super().__init__('oakd_publisher')

        self.bridge = CvBridge()
        self.imu_publisher_  = self.create_publisher(Imu, 'oak/imu', 10)
        self.rgb_publisher_ = self.create_publisher(Image, 'oak/rgb/image_raw', 10)
        self.left_publisher_ = self.create_publisher(Image, 'oak/left/image_raw', 10)
        self.right_publisher_ = self.create_publisher(Image, 'oak/right/image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, 'oak/depth/image_raw', 10)

        self.pipeline = dai.Pipeline()

        ####################
        #### set up imu ####
        ####################
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xout_imu = self.pipeline.create(dai.node.XLinkOut)
        self.xout_imu.setStreamName("imu")

        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)# enable ACCELEROMETER_RAW at 500 hz rate
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)# enable GYROSCOPE_RAW at 400 hz rate
        self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 200)
        self.imu.setBatchReportThreshold(1)
        self.imu.setMaxBatchReports(10)
        # Link plugins IMU -> XLINK
        self.imu.out.link(self.xout_imu.input)        


        ####################
        #### set up rgb ####
        ####################
        # Define sources and outputs
        self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        # Properties
        self.cam_rgb.setPreviewSize(1280, 720)
        self.cam_rgb.setInterleaved(False)
        # self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        self.cam_rgb.preview.link(self.xout_rgb.input)
        
        
        ###############################
        #### set up stereo + depth ####
        ###############################
        # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # Better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = False
        # Better handling for occlusions:
        lr_check = True
        # Define sources and outputs
        self.cam_monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.xout_left = self.pipeline.create(dai.node.XLinkOut)
        self.cam_monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.xout_right = self.pipeline.create(dai.node.XLinkOut)
        self.cam_depth = self.pipeline.create(dai.node.StereoDepth)
        self.xout_depth = self.pipeline.create(dai.node.XLinkOut)

        self.xout_left.setStreamName("left")
        self.xout_right.setStreamName("right")
        self.xout_depth.setStreamName("disparity")
        # Properties
        self.cam_monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.cam_monoLeft.setCamera("left")
        self.cam_monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.cam_monoRight.setCamera("right")
        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        self.cam_depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        self.cam_depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.cam_depth.setLeftRightCheck(lr_check)
        self.cam_depth.setExtendedDisparity(extended_disparity)
        self.cam_depth.setSubpixel(subpixel)

        # Linking
        self.cam_monoLeft.out.link(self.xout_left.input)
        self.cam_monoRight.out.link(self.xout_right.input)
        self.cam_monoLeft.out.link(self.cam_depth.left)
        self.cam_monoRight.out.link(self.cam_depth.right)
        self.cam_depth.disparity.link(self.xout_depth.input)


        #################################
        #### load pipeline to device ####
        #################################
        self.device = dai.Device(self.pipeline)

        self.q_imu = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        self.baseTs = None
        imu_timer_period = 1/200  # 200 Hz
        self.imu_timer = self.create_timer(imu_timer_period, self.imu_publisher)

        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        rgb_timer_period = 1/30  # seconds
        self.rgb_timer = self.create_timer(rgb_timer_period, self.rgb_publisher)

        self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
        stereo_timer_period = 1/30
        self.left_timer = self.create_timer(stereo_timer_period, self.left_publisher)
        self.right_timer = self.create_timer(stereo_timer_period, self.right_publisher)

        self.q_depth = self.device.getOutputQueue(name='disparity', maxSize=4, blocking=False)
        depth_timer_period = 1/30 
        self.depth_timer = self.create_timer(depth_timer_period, self.depth_publisher)


    def timeDeltaToMilliS(self, delta):
        return delta.total_seconds() * 1000
    

    def imu_publisher(self):
        imuData = self.q_imu.tryGet()
        if imuData is not None:
            imuPackets = imuData.packets
            for imuPacket in imuPackets:
                # Extract accelerometer, gyroscope and rotation values
                acceleroValues = imuPacket.acceleroMeter
                gyroValues = imuPacket.gyroscope
                rVvalues = imuPacket.rotationVector

                # Get timestamps for both sensors
                acceleroTs = acceleroValues.getTimestampDevice()
                gyroTs = gyroValues.getTimestampDevice()
                rvTs = rVvalues.getTimestampDevice()

                # Normalize timestamps relative to base timestamp
                if self.baseTs is None:
                    self.baseTs = min(acceleroTs, gyroTs, rvTs)

                acceleroTs = self.timeDeltaToMilliS(acceleroTs - self.baseTs)
                gyroTs = self.timeDeltaToMilliS(gyroTs - self.baseTs)
                rvTs = self.timeDeltaToMilliS(rvTs - self.baseTs)

                # Create ROS2 Imu message
                imu_msg = Imu()

                # Fill accelerometer data
                imu_msg.linear_acceleration = Vector3(
                    x=acceleroValues.x,
                    y=acceleroValues.y,
                    z=acceleroValues.z
                )

                # Fill gyroscope data
                imu_msg.angular_velocity = Vector3(
                    x=gyroValues.x,
                    y=gyroValues.y,
                    z=gyroValues.z
                )

                imu_msg.orientation.x = rVvalues.i
                imu_msg.orientation.y = rVvalues.j
                imu_msg.orientation.z = rVvalues.k
                imu_msg.orientation.w = rVvalues.real


                # Set timestamps (you can use acceleroTs or gyroTs depending on your preference)
                imu_msg.header.stamp = self.get_clock().now().to_msg()

                # Publish IMU data
                self.imu_publisher_.publish(imu_msg)
                # self.get_logger().info(f'Published IMU data with acceleroTs: {acceleroTs} ms and gyroTs: {gyroTs} ms')


        
    def rgb_publisher(self):
        msg = Image()
        in_rgb = self.q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.rgb_publisher_.publish(msg)

    def depth_publisher(self):
        msg = Image()
        in_depth = self.q_depth.tryGet()
        if in_depth is not None:
            frame = in_depth.getFrame()
            
            # print("Disparity Frame Shape:", frame.shape)
            # print("Disparity Frame Type:", frame.dtype)
            frame = (frame * (255.0 / self.cam_depth.initialConfig.getMaxDisparity())).astype(np.uint8)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
            self.depth_publisher_.publish(msg)

    def left_publisher(self):
        msg = Image()
        in_left = self.q_left.tryGet()
        if in_left is not None:
            frame = in_left.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
            self.left_publisher_.publish(msg)

    def right_publisher(self):
        msg = Image()
        in_right = self.q_right.tryGet()
        if in_right is not None:
            frame = in_right.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
            self.right_publisher_.publish(msg)

            
    
def main(args=None):
    rclpy.init(args=args)

    oakd_publisher = OakDPublisher()

    rclpy.spin(oakd_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oakd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()