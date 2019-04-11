#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import rosbag


class MyClass:

    bag = None

    bagTopicName = "topic_name_1"

    shift = 0

    def __init__(self):
        self.bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/goalPoseTest.bag', 'w')

        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/BinPose", PoseStamped, self.callback)

        rospy.Subscriber("chatter", String, self.topic_callback)

        rospy.Subscriber("/command_mine", Int32, self.shift_callback)

        rospy.spin()

    def callback(self, data):
        print("the data has been received in the class 3")

        data.header.stamp.secs = data.header.stamp.secs - self.shift
        self.bag.write(self.bagTopicName, data)

    def topic_callback(self , data):
        self.bagTopicName = data.data
        print("topic Name is now: " + self.bagTopicName)

    def shift_callback(self, data):
        self.shift = data.data
        print("shift value is: " + str(self.shift))

    def closebag(self):
        self.bag.close()


if __name__ == '__main__':

    obj = None
    try:
        obj = MyClass()
    finally:
        obj.closebag()
