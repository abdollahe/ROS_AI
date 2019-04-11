#!/usr/bin/env python
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf
import geometry_msgs.msg


if __name__ == '__main__':


    rospy.init_node('ur_robot_tf_listener')

    listener = tf.TransformListener()

    ur_robot_listener = rospy.Publisher('ur_robot/arm_position', geometry_msgs.msg.PoseStamped , queue_size=1 )

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        msg = geometry_msgs.msg.PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/wrist_3_link"

        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]

        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]

        ur_robot_listener.publish(msg)

        rate.sleep()