#!/usr/bin/env python
import sys, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from ur_robot_gazebo.msg import PoseMessageSimple


class ObjectSpawner:

    add_object = None
    delete_object = None
    model_paths = []
    model_names =[]

    def __init__(self):
        self.model_paths.append("/home/abdollah/catkin_ws/src/skill_transfer/ur_robot/models/green_brick")
        self.model_names.append("green_brick.sdf")

        rospy.init_node("object_spawner")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")

        self.add_object = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        self.delete_object = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        rospy.Subscriber("/ur_robot/target_obj_init_pose", PoseMessageSimple , self.spawn_callback)

    def spawn_callback(self, data):
        orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        object_pose = Pose(Point(data.position[0], data.position[1], data.position[2]), orient)

        with open(self.model_paths[0] + "/" + self.model_names[0], "r") as f:
            model = f.read()

        print(model)
        print(self.add_object("cube", model, "", object_pose, "world"))


if __name__ == '__main__':

    try:
        obj = ObjectSpawner()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass









