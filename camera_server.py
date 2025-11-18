import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import os

class CameraServiceNode(Node):
    def __init__(self):
        super().__init__('camera_service_node')
        
       
        self.subscription = self.create_subscription(
            Image,
            '/TurtleBot3Burger/front_camera/image_color', 
            self.listener_callback,
            10)
        self.latest_image = None
        self.bridge = CvBridge()

        
        self.srv = self.create_service(Trigger, 'capture_image_service', self.capture_callback)
        self.get_logger().info('Camera Service Node is ready! Waiting for BT request...')

    def listener_callback(self, msg):
        self.latest_image = msg

    def capture_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = "No image received yet! (Check camera topic)"
            self.get_logger().warn(response.message)
            return response

        try:
        
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            
            filename = "mission_success.jpg"
            cv2.imwrite(filename, cv_image)
            
            response.success = True
            response.message = f"Saved image to {os.getcwd()}/{filename}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()