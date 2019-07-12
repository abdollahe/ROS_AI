#!/usr/bin/env python
import h5py
import math
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import *
import tf
import random
import string
from gazebo_msgs.srv import *

class VerifyData:

    data = None
    eTimestep_list = None

    current_episode = None

    num_episodes = None

    sampling_percentage = None

    targetName = ""

    add_object = None

    delete_object = None

    def __init__(self):
        print("Class Created")

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('verify_robot_trajectory', anonymous=True)
        # self.robot = moveit_commander.RobotCommander()
        self.robot_group = moveit_commander.MoveGroupCommander("robot")

        self.current_episode = 0
        self.sampling_percentage = 10

    def obtain_training_set(self, f):
        """
        @Method
        @Description: Reads the content of the HDF5 file in memory
        @return: {matrix} - Actual data read from the HDF5 file
        """
        keys = f.keys()

        group_list = list(f.keys())
        n = len(group_list)

        return f[group_list[0]].value , f[group_list[1]].value

    def setup_data(self):
        # with h5py.File('/home/abdollah/Documents/training_data_long2.hdf5', 'a') as f:
        with h5py.File('/home/abdollah/Documents/training_data_short.hdf5', 'a') as f:

            self.eTimestep_list, self.data = self.obtain_training_set(f)

            self.num_episodes = len(self.eTimestep_list)

            print("train data is ready for input")
            return True

    def get_next_dataset(self):

        data_length = self.eTimestep_list[self.current_episode]

        temp_list = self.eTimestep_list[:self.current_episode]

        temp_data = self.data[sum(temp_list):(sum(temp_list) + data_length)]

        return_matrix_angles = np.zeros(shape=(int(math.floor(len(temp_data) / self.sampling_percentage)), 6))
        return_matrix_cube = np.zeros(shape=(int(math.floor(len(temp_data) / self.sampling_percentage)), 6))

        for i in range(0 , int(math.floor(len(temp_data) / self.sampling_percentage))):
            return_matrix_angles[i] = temp_data[i * self.sampling_percentage][19:25]
            return_matrix_cube[i] = temp_data[i * self.sampling_percentage][12:18]

        if self.current_episode != (self.num_episodes - 1):
            self.current_episode += 1
        else:
            self.current_episode = 0

        return return_matrix_angles , return_matrix_cube

    def randomString(self):
        stringLength = 20
        """Generate a random string of fixed length """
        letters = string.ascii_letters
        rand = ''.join(random.choice(letters) for i in range(stringLength))
        return rand

    def spawn_object(self , position):

        modelpath = "/home/abdollah/ros_ws/src/UR_Robot_AI/ur_robot/models/green_brick/green_brick.sdf"

        orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        object_pose = Pose(Point(position[0], position[1], position[2]), orient)

        with open(modelpath, "r") as f:
            model = f.read()
        self.targetName = ""
        # print(model)
        self.targetName = "cube" + self.randomString()
        print("Model name for spawn is: " + self.targetName)

        rospy.wait_for_service("gazebo/spawn_sdf_model")

        self.add_object = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        print(self.add_object(self.targetName, model, "", object_pose, "world"))

    def delete_cube(self):
        rospy.wait_for_service("gazebo/delete_model")
        self.delete_object = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        print("Model name for delete is: " + self.targetName)
        result = self.delete_object(self.targetName)
        print "wait for object delete to complete"

    def plan_joint_goal(self, joint_angles):
        joint_goal = self.robot_group.get_current_joint_values()
        joint_goal[0] = joint_angles[2]
        joint_goal[1] = joint_angles[1]
        joint_goal[2] = joint_angles[0]
        joint_goal[3] = joint_angles[3]
        joint_goal[4] = joint_angles[4]
        joint_goal[5] = joint_angles[5]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.robot_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_group.stop()

        return True


if __name__ == "__main__":

    obj = VerifyData()

    obj.setup_data()

    temp_d, cube_pose = obj.get_next_dataset()
    temp_d, cube_pose = obj.get_next_dataset()
    temp_d, cube_pose = obj.get_next_dataset()
    temp_d, cube_pose = obj.get_next_dataset()
    temp_d, cube_pose = obj.get_next_dataset()

    obj.spawn_object(cube_pose[0])

    for i in range(0, len(temp_d)):


        d12 = temp_d[i]

        obj.plan_joint_goal(d12)

    print("Yep Done!!!!!")

    obj.delete_cube()
