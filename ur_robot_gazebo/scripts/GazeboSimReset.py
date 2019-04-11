#!/usr/bin/env python
import sys, rospy, tf
# from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import *
from copy import deepcopy

if __name__ == '__main__':

    # model_paths = "/home/abdollah/catkin_ws/src/robot_skill_transfer/schunk_train_data_gen/models"
    # model_paths = "/home/abdollah/catkin_ws/src/skill_transfer/ur_robot/models/green_brick"
    # model_name ="green_brick.sdf"

    rospy.init_node("gazebo_sim_reset")
    # rospy.wait_for_service("gazebo/delete_model")
    # rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/reset_simulation")

    # delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    s = rospy.ServiceProxy("gazebo/reset_simulation", Empty)
    # delete_model("chessboard")

    # orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    # object_pose = Pose(Point(0.5,0,0), orient)
    #
    # with open(model_paths + "/" + model_name, "r") as f:
    #     model = f.read()
    #
    # print(model)
    # print(s("cube1", model, "", object_pose, "world"))




