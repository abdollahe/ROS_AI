#!/usr/bin/env python

import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/abdollah/ros_ws/src/UR_Robot_AI/ur_robot_gazebo/launch/test.launch"])
launch.start()
rospy.loginfo("started")

