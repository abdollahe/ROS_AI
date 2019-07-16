import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
import std_srvs.srv
from ur_robot_gazebo.msg import JointStateSimple
from std_msgs.msg import Bool
from ur_robot_gazebo.msg import PoseMessageSimple
from std_msgs.msg import Int32
import time

import h5py
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotTajectory:

    topic_name = "/ur_traj_controller/command"

    train_data_path = "/home/abdollah/Documents/New_HDF/Long/training_data_long.hdf5"

    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


    def __init__(self):

        # Initialize the node
        rospy.init_node('ur_robot_joint_control')

        self.vel_pub = rospy.Publisher(self.topic_name, JointTrajectory, queue_size=1)

    def read_training_data(self, data_type):

        with h5py.File(self.train_data_path, 'a') as f:

            keys = f.keys()

            group_list = list(f.keys())
            n = len(group_list)

            train_data = f[group_list[1]].value
            episode_lengths = f[group_list[0]].value

            # holds a random index from the episodes available
            random_index = np.random.randint(0 , len(episode_lengths))

            random_index = 0

            start_point = episode_lengths[0:random_index].sum()

            episode_data = train_data[start_point:(start_point + episode_lengths[random_index])]

            return_value = None

            # extract the joint angles and return [19-24]
            if data_type == 0:
                return_value = episode_data[:, 19:25]
            elif data_type == 1:
                return_value = episode_data[:, 25:]

            return return_value

    def send_data(self, data):

        jt = JointTrajectory()

        # jt.header.stamp = rospy.Time.now()
        # jt.header.frame_id = "ur::joints"

        jt.joint_names = self.joint_names

        p = JointTrajectoryPoint()

        p.positions.append(0)
        p.positions.append(-1.57)
        p.positions.append(0)
        p.positions.append(-1.57)
        p.positions.append(-1.57)
        p.positions.append(0)

        p.time_from_start = rospy.Duration.from_sec(1.0)

        jt.points.append(p)

        # for i in range(0, len(trajectory_data)):
        #
        #     if trajectory_type == 0:
        #         p.positions = trajectory_data[i]
        #         jt.points.append(p)
        #         # jt.points[i].time_from_start = rospy.Duration.from_sec(0.01)
        #     elif trajectory_type == 1:
        #         p.velocities = trajectory_data[1]
        #         jt.points.append(p)
        #         # jt.points[i].time_from_start = rospy.Duration.from_sec(0.01)

        ctrl_c = False
        while not ctrl_c:
            if self.vel_pub.get_num_connections() > 0:
                self.vel_pub.publish(jt)
                self.vel_pub.publish(jt)
                self.vel_pub.publish(jt)
                self.vel_pub.publish(jt)
                self.vel_pub.publish(jt)
                ctrl_c = True


if __name__ == "__main__":

    # 0 for joint angles and 1 for joint velocity
    trajectory_type = 0
    try:
        obj = RobotTajectory()
        data1 = obj.read_training_data(trajectory_type)

        obj.send_data(data1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

