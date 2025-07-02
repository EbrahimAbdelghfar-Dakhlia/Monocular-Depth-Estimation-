# make a code that access usb camera and publish the image to a topic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
import yaml

class UsbCameraPublisher(Node):
    def _read_calibration(self, calibration_file):
        try:
            with open(calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            if not calib_data:
                self.get_logger().error(f"Calibration file is empty: {calibration_file}")
                return None

            cam_info = CameraInfo()
            cam_info.width = calib_data.get('image_width')
            cam_info.height = calib_data.get('image_height')
            cam_info.k = calib_data.get('camera_matrix', {}).get('data', [])
            cam_info.d = calib_data.get('distortion_coefficients', {}).get('data', [])
            cam_info.r = calib_data.get('rectification_matrix', {}).get('data', [])
            cam_info.p = calib_data.get('projection_matrix', {}).get('data', [])
            cam_info.distortion_model = calib_data.get('distortion_model')
            return cam_info
        except (IOError, yaml.YAMLError) as e:
            self.get_logger().error(f"Failed to read or parse calibration file '{calibration_file}': {e}")
            return None

    def __init__(self):
        super().__init__('usb_camera_publisher')

        self.declare_parameter('camera_calibration_file', 'src/modular_robot_description/launch/ost.yaml')  # Default to 'ost.yaml' if not provided
        calibration_file = self.get_parameter('camera_calibration_file').get_parameter_value().string_value

        self.camera_info_msg = None
        if calibration_file:
            self.camera_info_msg = self._read_calibration(calibration_file)
            if self.camera_info_msg:
                self.get_logger().info(f"Successfully loaded camera calibration from {calibration_file}")
        else:
            self.get_logger().warn("Camera calibration file not provided. 'camera_info' will not be published.")

        # Define QoS profile for the publisher
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        #create camera info publisher
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera_info', qos_profile)

        # Create a publisher for the USB camera images
        self.publisher_ = self.create_publisher(Image, '/image_rect', qos_profile)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open the first camera

        if not self.cap.isOpened():
            self.get_logger().error('Could not open USB camera')

        self.timer = self.create_timer(float(1/30), self.timer_callback)  # Publish at 30 Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now().to_msg()
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = now
            image_msg.header.frame_id = 'camera_frame' # or your desired frame
            self.publisher_.publish(image_msg)

            # Publish camera info if available
            if self.camera_info_msg:
                self.camera_info_msg.header.stamp = now
                self.camera_info_msg.header.frame_id = image_msg.header.frame_id
                self.camera_info_publisher.publish(self.camera_info_msg)

            self.get_logger().info('Published image from USB camera')
        else:
            self.get_logger().error('Failed to capture image from USB camera')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    usb_camera_publisher = UsbCameraPublisher()
    rclpy.spin(usb_camera_publisher)
    usb_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()