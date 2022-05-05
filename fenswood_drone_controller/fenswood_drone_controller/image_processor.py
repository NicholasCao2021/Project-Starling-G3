"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        self.br = CvBridge()
        self.num = 0
        self.done = True

    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)

    # on receiving image, convert and log information
    def image_callback(self,msg):
        img = CvBridge().imgmsg_to_cv2(msg,"bgr8")
        self.num+=1
        if self.done==True and self.num%20==0:
            self.done = False
            # timestr = "%.6f" % msg.header.stamp.to_sec()
            #img_name = "{:0>5d}.png".format(self.num)
            #cv2.imwrite('/ros_ws/img/'+img_name,img)
            #self.get_logger().info(f'{img_name}, img saved')
            #shp = img.shape 
            #self.get_logger().info('picture of {} x {} saved,first pixel{} at {}'.format(shp[0],shp[1],img[0][0],os.path.abspath('.')))
            self.done = True
        else:
            pass
        # can do OpenCV stuff on img now
        # # just get the size
        # self.get_logger().info('Got an image '.format(shp[0],shp[1]))
                    

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()
