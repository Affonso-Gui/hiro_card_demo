#!/usr/bin/env python

import sys
import argparse
import rospy
from std_msgs.msg import Empty
from jsk_recognition_msgs.msg import Rect, RectArray

class CheckIntersection:
    # claimed topic must have 0 or 1 elements
    #
    def __init__(self, topic1, topic2, output_topic, tolerance=-1, size_threshold=2.5):
        self.sub1 = rospy.Subscriber(topic1, RectArray, self.callback_update, queue_size=1)
        self.sub2 = rospy.Subscriber(topic2, RectArray, self.callback_check, queue_size=2)
        self.pub  = rospy.Publisher(output_topic, Empty, queue_size=10)

        self.rect = None
        self.empty_counter = 0
        self.tolerance = tolerance
        self.thre = size_threshold

    def callback_update(self, msg):
        if msg.rects:
            self.empty_counter = 0
            self.rect = msg.rects[0]
            return
        # empty
        self.empty_counter += 1
        if self.tolerance >= 0 and self.empty_counter > self.tolerance:
            self.rect = None

    def callback_check(self, msg):
        if self.rect is None:
            return
        for rect in msg.rects:
            if self.check(rect):
                self.pub.publish(Empty())
                return

    def check_overlap(self, r):
        if self.rect is None:
            return False
        if (r.x + r.width) < self.rect.x:
            return False
        if r.x > (self.rect.x + self.rect.width):
            return False
        if (r.y + r.height) < self.rect.y:
            return False
        if r.y > (self.rect.y + self.rect.height):
            return False
        return True

    def check_dimensions(self, r):
        if self.rect is None:
            return False
        if r.width > self.rect.width * self.thre:
            return False
        if r.height > self.rect.height * self.thre:
            return False
        return True

    def check(self, r):
        return self.check_dimensions(r) and self.check_overlap(r)


def filter_roslaunch_args(arg_list):
    return [x for x in arg_list if not x.startswith('__name') and not x.startswith('__log')]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("topic1", help="Claimed resource rectangle")
    parser.add_argument("topic2", help="Claiming resource rectangle")
    parser.add_argument("output_topic", help="Publishing result")
    parser.add_argument("--tolerance", type=int, default=-1,
                        help="Number of empty messages before reseting")

    args = parser.parse_args(filter_roslaunch_args(sys.argv)[1:])

    rospy.init_node('rectangle_intersection', anonymous=True)
    worker = CheckIntersection(args.topic1, args.topic2, args.output_topic,
                               tolerance=args.tolerance)
    rospy.spin()
