#!/usr/bin/env python
from __future__ import division, print_function

import apriltag
import cv2
import rospy
from amrl_vision_common.msg import Perception, Perceptions
from amrl_vision_common.srv import SetEnabled
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image


class Webcam:
    def __init__(self):
        # rospy.Subscriber('image', Image, self._callback)
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(
            '/vision/perceptions',
            Perceptions,
            queue_size=10
        )
        self.detector = apriltag.Detector(
            searchpath=apriltag._get_demo_searchpath()
        )
        rospy.Service(
            '/{}/set_enable'.format(rospy.get_name()),
            SetEnabled,
            self.set_enable_cb
        )

    def set_enable(self, state):
        if state:
            self._image_sub = rospy.Subscriber(
                'image', Image, self._callback)
        elif self._image_sub is not None:
            self._image_sub.unregister()

    def set_enable_cb(self, req):
        self.set_enable(req.enabled)
        return req.enabled

    def point_to_msg(self, corner_to_send, width, height):
        points = []
        for point in corner_to_send:
            p = Point32()
            p.x, p.y = point
            p.x, p.y = p.x / width, p.y / height
            points.append(p)
        return points

    def _callback(self, data):
        if self.publisher.get_num_connections() == 0:
            return

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(gray)

        perceptions_array = Perceptions()
        perceptions_array.perception_name = "apriltag"
        perceptions_array.header.stamp = rospy.Time().now()
        perceptions_array.header.frame_id = data.header.frame_id
        height, width = image.shape[:2]
        for i, detection in enumerate(detections):
            p = Perception()
            p.polygon.points = self.point_to_msg(
                detection.corners, width, height)
            p.name = "{}-{}".format(detection.tag_family, detection.tag_id)
            perceptions_array.perceptions.append(p)

        self.publisher.publish(perceptions_array)

    def shutdown(self):
        rospy.logwarn('shutting down')


if __name__ == "__main__":
    rospy.init_node('apriltag_detection_node')
    webcam = Webcam()

    def shutdown():
        webcam.shutdown()

    rospy.on_shutdown(shutdown)
    rospy.spin()
