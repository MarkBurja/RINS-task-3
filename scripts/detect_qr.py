#!/usr/bin/env python3

import urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import urllib
import validators

import time

class detect_qr(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('detect_qr')

        self.bridge = CvBridge()
        self.rgb_image_sub = self.create_subscription(Image, "/top_camera/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.mona_lisa_pub = self.create_publisher(Image, "/mona_lisa", qos_profile_sensor_data)
        self.mona_lisa_sub = self.create_subscription(Image, "/mona_lisa", self.mona_lisa_callback, qos_profile_sensor_data)
        self.detector = cv2.QRCodeDetector()

        self.found = False
        self.mona_lisa = None

        self.get_logger().info(f"Detect_qr node has been initialized!")

    def mona_lisa_callback(self, data):
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            cv2.imshow("mona", img)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()

        except CvBridgeError as e:
                print(e)

    def rgb_callback(self, data):

        try:
            
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if not self.found:

                retval, decoded_info, points, straight_qrcode = self.detector.detectAndDecodeMulti(img)

                #print(retval)
                #print(decoded_info)

                if retval:
                    img = cv2.polylines(img, points.astype(int), True, (0, 255, 0), 3)

                    value = decoded_info[0]
                    if validators.url(value):

                        print("QR code found!")

                        req = urllib.request.urlopen(value)
                        arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
                        mona_lisa = cv2.imdecode(arr, -1)

                        img_msg = self.bridge.cv2_to_imgmsg(mona_lisa, encoding="bgr8")
                        self.mona_lisa_pub.publish(img_msg)
                        
                        self.mona_lisa = mona_lisa
                        self.found = True
                    else:

                        print("Not a QR code!")


            cv2.imshow("qr", img)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()
                
        except CvBridgeError as e:
            print(e)
            
def main():
    print('QR detection node starting.')

    rclpy.init(args=None)
    node = detect_qr()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()