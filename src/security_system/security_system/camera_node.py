
import rclpy  
from rclpy.node import Node  
import pyrealsense2 as rs
import cv2  
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')  
        self.publisher_ = self.create_publisher(Image, 'camera/image', 1) 
        self.timer = self.create_timer(0.001, self.timer_callback)  
        self.cap = cv2.VideoCapture(0)  
        #self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.bridge = CvBridge()  

    def timer_callback(self):
        ret, frame = self.cap.read() 
        if ret: 
            frame = cv2.resize(frame,(640,480), interpolation=cv2.INTER_NEAREST)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8") 
            self.publisher_.publish(msg) 
            # cv2.imshow('Webcam', frame)  
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                rclpy.shutdown() 
        else:
            self.get_logger().error('Failed to capture image') 


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('camera_node')  
        self.publisher_img = self.create_publisher(Image, 'camera/image', 1) 
        self.publisher_depth = self.create_publisher(Image, 'camera/depth', 1) 
        self.timer = self.create_timer(0.001, self.timer_callback)  
        self.cap = cv2.VideoCapture(0)  
        #self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self.bridge = CvBridge()  

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)  # 960, 540,
        self.align = rs.align(rs.stream.color)

        # Start streaming
        cfg = self.pipeline.start(self.config)
        profile = cfg.get_stream(rs.stream.depth) 
        
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        self.get_logger().info(f"Width: {intrinsics.width}")
        self.get_logger().info(f"Height: {intrinsics.height}")
        self.get_logger().info(f"Principal Point X (ppx): {intrinsics.ppx}")
        self.get_logger().info(f"Principal Point Y (ppy): {intrinsics.ppy}")
        self.get_logger().info(f"Focal Length X (fx): {intrinsics.fx}")
        self.get_logger().info(f"Focal Length Y (fy): {intrinsics.fy}")
        self.get_logger().info(f"Distortion Model: {intrinsics.model}")
        self.get_logger().info(f"Distortion Coefficients: {intrinsics.coeffs}")
        self.get_logger().info(f"Depth_scale: {cfg.get_device().first_depth_sensor().get_depth_scale()}")


    def timer_callback(self):

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        msg_img = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8") 
        self.publisher_img.publish(msg_img) 

        msg_depth = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1") 
        self.publisher_depth.publish(msg_depth) 


def main(args=None):
    rclpy.init(args=args)  
    # node = CameraNode()  
    node = RealSenseNode()
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass  
    finally:
        node.cap.release() 
        cv2.destroyAllWindows() 
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':
    main()  
