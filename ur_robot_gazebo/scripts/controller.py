#!/usr/bin/env python
import roslaunch
import rospy

if __name__ == '__main__':

    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/abdollah/catkin_ws/src/skill_transfer/ur_robot_gazebo/launch/test.launch"])
    launch.start()
    rospy.loginfo("started")


    # rospy.sleep(3)
    # # 3 seconds later
    # launch.shutdown()