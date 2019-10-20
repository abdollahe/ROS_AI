
import rospy

import h5py
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotTrajectory:

    topic_name = "/ur_traj_controller/command"

    train_data_path = "/home/abdollah/Documents/New_HDF/Long/training_data_long.hdf5"

    # The position of the joints is similar to what comes back from
    # the joint_states topic (position 0 and 2 are swapped)
    joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    def __init__(self):
        # Initialize the node
        rospy.init_node('ur_robot_joint_control')

        self.vel_pub = rospy.Publisher(self.topic_name, JointTrajectory, queue_size=1, latch=True)

    def read_training_data(self, data_type):

        with h5py.File(self.train_data_path, 'a') as f:

            keys = f.keys()

            group_list = list(f.keys())
            n = len(group_list)

            train_data = f[group_list[1]].value
            episode_lengths = f[group_list[0]].value

            # holds a random index from the episodes available
            random_index = np.random.randint(0, len(episode_lengths))

            random_index = 0

            start_point = episode_lengths[0:random_index].sum()

            episode_data = train_data[start_point:(start_point + episode_lengths[random_index])]

            return_value = None

            if data_type == 0:
                return_value = episode_data[:, 19:25]
            elif data_type == 1:
                return_value = episode_data[:, 25:]

            return return_value

    def send_data(self, data):

        jt = JointTrajectory()

        jt.header.stamp = rospy.Time.now()
        jt.header.frame_id = "ur::joints"

        jt.joint_names = self.joint_names

        # p.time_from_start = rospy.Duration.from_sec(1.0)

        # jt.points.append(p)

        dt = 0.1
        for i in range(0, len(data)):

            if trajectory_type == 0:
                temp = data[i]
                p = JointTrajectoryPoint()
                for j in range(0, len(temp)):
                    p.positions.append(temp[j])

                jt.points.append(p)

            elif trajectory_type == 1:
                temp = data[i]
                p = JointTrajectoryPoint()
                for j in range(0, len(temp)):
                    p.velocities.append(temp[j])

                jt.points.append(p)

            jt.points[i].time_from_start = rospy.Duration.from_sec((i + 1) * dt)

        ctrl_c = False
        while not ctrl_c:
            if self.vel_pub.get_num_connections() > 0:

                self.vel_pub.publish(jt)

                ctrl_c = True


if __name__ == "__main__":

    # 0 for joint angles and 1 for joint velocity
    trajectory_type = 1
    try:
        obj = RobotTrajectory()
        data1 = obj.read_training_data(trajectory_type)

        obj.send_data(data1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

