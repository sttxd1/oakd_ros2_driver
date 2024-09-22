import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class MulticamsPublisher(Node):
    def __init__(self):
        super().__init__('multicams_publisher')

        # Declare parameters for camera name and mxid
        self.declare_parameter('camera_name', 'oakd_camera')
        # self.declare_parameter('mxid', 'default_mxid')

        # Declare parameters for enabling nodes

        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_stereo', True)
        self.declare_parameter('enable_imu', True)

        # Read parameters
        camera_name = str(self.get_parameter('camera_name').get_parameter_value().string_value)
        # mxid = str(self.get_parameter('mxid').get_parameter_value().string_value)
        if camera_name == 'oakd_w':
            mxid = '194430107131962E00'
        if camera_name == 'oakd_pro_w':
            mxid = '18443010D1F83CF500'

        enable_rgb = self.get_parameter('enable_rgb').get_parameter_value().bool_value
        enable_stereo = self.get_parameter('enable_stereo').get_parameter_value().bool_value
        enable_depth = self.get_parameter('enable_depth').get_parameter_value().bool_value
        enable_imu = self.get_parameter('enable_imu').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.pipeline = dai.Pipeline()

        #######################
        #### set up rgb ####
        #######################
        if enable_rgb:
            self.rgb_publisher_ = self.create_publisher(Image, f'{camera_name}/rgb/image_raw', 10)
            self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
            self.xout_rgb = self.pipeline.create(dai.node.XLinkOut)
            self.xout_rgb.setStreamName("rgb")
            self.cam_rgb.setPreviewSize(1280, 720)
            self.cam_rgb.setInterleaved(False)
            self.cam_rgb.preview.link(self.xout_rgb.input)
            

         ##############################
        #### set up stereo (left and right) ####
        ##############################
        if enable_stereo:
            self.left_publisher_ = self.create_publisher(Image, f'{camera_name}/left/image_raw', 10)
            self.right_publisher_ = self.create_publisher(Image, f'{camera_name}/right/image_raw', 10)

            self.cam_monoLeft = self.pipeline.create(dai.node.MonoCamera)
            self.cam_monoRight = self.pipeline.create(dai.node.MonoCamera)

            self.xout_left = self.pipeline.create(dai.node.XLinkOut)
            self.xout_right = self.pipeline.create(dai.node.XLinkOut)

            self.xout_left.setStreamName("left")
            self.xout_right.setStreamName("right")

            # Set properties for stereo cameras
            self.cam_monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            self.cam_monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
            self.cam_monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            self.cam_monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

            # Link mono cameras to output
            self.cam_monoLeft.out.link(self.xout_left.input)
            self.cam_monoRight.out.link(self.xout_right.input)


                    
        ##########################
        #### set up depth ####
        ##########################
        if enable_depth:
            self.depth_publisher_ = self.create_publisher(Image, f'{camera_name}/depth/image_raw', 10)
            
            self.cam_depth = self.pipeline.create(dai.node.StereoDepth)
            self.xout_depth = self.pipeline.create(dai.node.XLinkOut)
            self.xout_depth.setStreamName("disparity")

            # Depth settings from your original code
            extended_disparity = False
            subpixel = False
            lr_check = True

            # Configure the StereoDepth node
            self.cam_depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.cam_depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)  # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
            self.cam_depth.setLeftRightCheck(lr_check)
            self.cam_depth.setExtendedDisparity(extended_disparity)
            self.cam_depth.setSubpixel(subpixel)

            # Link mono cameras to depth node
            self.cam_monoLeft.out.link(self.cam_depth.left)
            self.cam_monoRight.out.link(self.cam_depth.right)

            # Link depth to output
            self.cam_depth.disparity.link(self.xout_depth.input)

            
        #######################
        #### set up imu ####
        #######################
        if enable_imu:
            self.imu_publisher_ = self.create_publisher(Imu, f'{camera_name}/imu', 10)
            self.imu = self.pipeline.create(dai.node.IMU)
            self.xout_imu = self.pipeline.create(dai.node.XLinkOut)
            self.xout_imu.setStreamName("imu")
            self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
            self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
            self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 200)
            self.imu.setBatchReportThreshold(1)
            self.imu.setMaxBatchReports(10)
            self.imu.out.link(self.xout_imu.input)
            

        # Start the device
        self.device = dai.Device(self.pipeline, dai.DeviceInfo(mxid))
        if enable_rgb:
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            rgb_timer_period = 1/30  # 30 FPS
            self.rgb_timer = self.create_timer(rgb_timer_period, self.rgb_publisher)
        if enable_stereo:
            self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
            stereo_timer_period = 1/30  # 30 FPS
            self.left_timer = self.create_timer(stereo_timer_period, self.left_publisher)
            self.right_timer = self.create_timer(stereo_timer_period, self.right_publisher)
        if enable_depth:
            self.q_depth = self.device.getOutputQueue(name='disparity', maxSize=4, blocking=False)
            depth_timer_period = 1/30  # 30 FPS
            self.depth_timer = self.create_timer(depth_timer_period, self.depth_publisher)
        if enable_imu:
            self.baseTs = None
            self.q_imu = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            imu_timer_period = 1/200  # 200 Hz
            self.imu_timer = self.create_timer(imu_timer_period, self.imu_publisher)

    
    # Publisher methods for rgb, depth, stereo, imu
    def rgb_publisher(self):
        in_rgb = self.q_rgb.tryGet()
        if in_rgb:
            frame = in_rgb.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.rgb_publisher_.publish(msg)

    def depth_publisher(self):
        in_depth = self.q_depth.tryGet()
        if in_depth:
            frame = in_depth.getFrame()
            frame = (frame * (255.0 / self.cam_depth.initialConfig.getMaxDisparity())).astype(np.uint8)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
            self.depth_publisher_.publish(msg)

    def left_publisher(self):
        in_left = self.q_left.tryGet()
        if in_left:
            frame = in_left.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
            self.left_publisher_.publish(msg)

    def right_publisher(self):
        in_right = self.q_right.tryGet()
        if in_right:
            frame = in_right.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
            self.right_publisher_.publish(msg)

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



def main(args=None):
    rclpy.init(args=args)
    multicams_publisher = MulticamsPublisher()
    rclpy.spin(multicams_publisher)
    multicams_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
