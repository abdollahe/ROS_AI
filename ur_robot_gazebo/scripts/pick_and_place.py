#!/usr/bin/env python
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


class PickAndPlace:
    robot_group = None
    robot1_client = None

    joint_wait = [0.05, -1.52, 1.52, -1.57, -1.57, 0]
    # joint_wait = [0.05, -1.57, 1.57, -1.57, -1.57, 0]

    move_to_joint_done_pub = None

    pick_and_place_done_pub = None ;

    target_pose = geometry_msgs.msg.Pose()


    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('simple_pick_place', anonymous=True)
        self.robot_group = moveit_commander.MoveGroupCommander("robot")
        rospy.Subscriber("/arm_joint_state", JointStateSimple, self.arm_joint_state_callback)
        rospy.Subscriber("/ur_robot/arm_start_grasp", PoseMessageSimple, self.start_grasping_process_callback)

        self.move_to_joint_done_pub = rospy.Publisher('/ur_robot/move_joint_done', Bool, queue_size=1)

        self.pick_and_place_done_pub = rospy.Publisher("/ur_robot/pick_place_finish", Int32, queue_size=1)


        #self.go_to_joint(self.joint_wait)

    def arm_joint_state_callback(self, data):
        target_joint_state = data.joint_angles
        self.go_to_joint(target_joint_state)

    def start_grasping_process_callback(self, data):
        self.target_pose.position.x = data.position[0]
        self.target_pose.position.y = data.position[1]
        self.target_pose.position.z = data.position[2] + 0.05

        # pose_goal = geometry_msgs.msg.Pose()
        self.target_pose.orientation.w = 0.00802366832289
        self.target_pose.orientation.x = 0.883899607609
        self.target_pose.orientation.y = -0.467562344368
        self.target_pose.orientation.z = -0.00652369106187
        # pose_goal.position.x = 0.5
        # pose_goal.position.y = 0.0
        # pose_goal.position.z = 0.1

        print("I am in the grasping callback")

        self.move_to_target()

        self.grasp_object()
        self.move_from_target_to_goal()
        self.release_object()
        self.move_to_wait()

    def simple_pick_place(self):
        # First initialize moveit_commander and rospy.

        # Instantiate a MoveGroupCommander object.  This object is an interface
        # to one group of joints.  In this case the group refers to the joints of
        # Action clients to the ExecuteTrajectory action server.

        self.robot1_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self.robot1_client.wait_for_server()
        rospy.loginfo('Execute Trajectory server is available for robot')
        #
        # # Set a named joint configuration as the goal to plan for a move group.
        # # Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
        # self.robot_group.set_named_target("up")
        #
        # # Plan to the desired joint-space goal using the default planner (RRTConnect).
        # plan = self.robot_group.plan()
        # # Create a goal message object for the action server.
        # robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # # Update the trajectory in the goal message.
        # robot1_goal.trajectory = plan
        #
        # # Send the goal to the action server.
        # self.robot1_client.send_goal(robot1_goal)
        #
        # res = self.robot1_client.wait_for_result()
        #
        # if res == 2:
        #     rospy.loginfo("Goal executed, proceeding to the next goal")
        # elif res == 1:
        #     rospy.loginfo("Goal is being executed, waiting to finish to proceed")

        # self.robot_group.set_named_target("wait")
        #
        # # Plan to the desired joint-space goal using the default planner (RRTConnect).
        # plan = self.robot_group.plan()
        # # Create a goal message object for the action server.
        # robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # # Update the trajectory in the goal message.
        # robot1_goal.trajectory = plan
        #
        # # Send the goal to the action server.
        # self.robot1_client.send_goal(robot1_goal)
        # res = self.robot1_client.wait_for_result()
        #
        # if res == 2:
        #     rospy.loginfo("Goal executed, proceeding to the next goal")
        # elif res == 1:
        #     rospy.loginfo("Goal is being executed, waiting to finish to proceed")

        # self.robot_group.set_named_target("down_state")
        #
        # # Plan to the desired joint-space goal using the default planner (RRTConnect).
        # plan = self.robot_group.plan()
        # # Create a goal message object for the action server.
        # robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # # Update the trajectory in the goal message.
        # robot1_goal.trajectory = plan
        #
        # # Send the goal to the action server.
        # self.robot1_client.send_goal(robot1_goal)
        # res = self.robot1_client.wait_for_result()
        #
        # if res == 2:
        #     rospy.loginfo("Goal executed, proceeding to the next goal")
        # elif res == 1:
        #     rospy.loginfo("Goal is being executed, waiting to finish to proceed")

        # self.go_to_joint(self.joint_wait)

    def move_to_target(self):
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 0.00802366832289
        # pose_goal.orientation.x = 0.883899607609
        # pose_goal.orientation.y = -0.467562344368
        # pose_goal.orientation.z = -0.00652369106187
        # pose_goal.position.x = 0.5
        # pose_goal.position.y = 0.0
        # pose_goal.position.z = 0.1

        # self.robot_group.set_pose_target(self.target_pose)
        # plan = self.robot_group.go(wait=True)
        #

        # Calling `stop()` ensures that there is no residual movement
        # self.robot_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # self.robot_group.clear_pose_targets()

        self.robot_group.set_pose_target(self.target_pose)

        # Plan to the desired joint-space goal using the default planner (RRTConnect).
        plan = self.robot_group.plan()
        # Create a goal message object for the action server.
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message.
        robot1_goal.trajectory = plan

        # Send the goal to the action server.
        self.robot1_client.send_goal(robot1_goal)
        res = False
        for i in range(0,5):
            res = self.robot1_client.wait_for_result()
            if res:
                rospy.loginfo("Goal executed - Move to target , proceeding to the next goal")
                break
            elif res:
                rospy.loginfo("Goal is being executed - Move to target , waiting to finish to proceed")

    def go_to_joint(self, joints):
        print("Executing the got to joints!!!!")
        self.robot_group.go(joints, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_group.stop()

        ctrl_c = False
        while not ctrl_c:
            print("Sending done status back")
            if self.move_to_joint_done_pub.get_num_connections() > 0:
                print("Sending done status back - after checking connection")
                msg = Bool()
                msg.data = True
                self.move_to_joint_done_pub.publish(msg)
                ctrl_c = True

    def move_to_pose(self, goalpose):
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 0.00802366832289
        # pose_goal.orientation.x = 0.883899607609
        # pose_goal.orientation.y = -0.467562344368
        # pose_goal.orientation.z = -0.00652369106187
        # pose_goal.position.x = -0.082724855514
        # pose_goal.position.y = 0.582823716523
        # pose_goal.position.z = 0.582901434879

        self.robot_group.set_pose_target(goalpose)

        # Plan to the desired joint-space goal using the default planner (RRTConnect).
        plan = self.robot_group.plan()
        # Create a goal message object for the action server.
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message.
        robot1_goal.trajectory = plan

        # Send the goal to the action server.
        self.robot1_client.send_goal(robot1_goal)
        res = self.robot1_client.wait_for_result()

        if res == 2:
            rospy.loginfo("Goal executed - Move to pose , proceeding to the next goal")
        elif res == 1:
            rospy.loginfo("Goal is being executed - Move to Pose, waiting to finish to proceed")

    def move_from_target_to_goal(self):
        #  ## Cartesian Paths
        # ## ^^^^^^^^^^^^^^^
        # ## You can plan a cartesian path directly by specifying a list of waypoints
        # ## for the end-effector to go through.
        waypoints = []
        # start with the current pose
        current_pose = self.robot_group.get_current_pose()
        rospy.sleep(0.5)
        current_pose = self.robot_group.get_current_pose()

        # create linear offsets to the current pose
        new_eef_pose = geometry_msgs.msg.Pose()
        new_eef_pose2 = geometry_msgs.msg.Pose()
        new_eef_pose3 = geometry_msgs.msg.Pose()
        new_eef_pose4 = geometry_msgs.msg.Pose()
        # Manual offsets because we don't have a camera to detect objects yet.
        new_eef_pose.position.x = current_pose.pose.position.x
        new_eef_pose.position.y = current_pose.pose.position.y
        new_eef_pose.position.z = current_pose.pose.position.z + 0.30

        # Retain orientation of the current pose.
        new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

        waypoints.append(new_eef_pose)

        new_eef_pose2.position.x = new_eef_pose.position.x - 0.2
        new_eef_pose2.position.y = new_eef_pose.position.y + 0.2
        new_eef_pose2.position.z = new_eef_pose.position.z

        # Retain orientation of the current pose.
        new_eef_pose2.orientation = copy.deepcopy(new_eef_pose.orientation)

        waypoints.append(new_eef_pose2)

        new_eef_pose3.position.x = new_eef_pose2.position.x - 0.2
        new_eef_pose3.position.y = new_eef_pose2.position.y + 0.2
        new_eef_pose3.position.z = new_eef_pose2.position.z + 0.2

        # Retain orientation of the current pose.
        new_eef_pose3.orientation = copy.deepcopy(new_eef_pose2.orientation)

        waypoints.append(new_eef_pose3)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.00802366832289
        pose_goal.orientation.x = 0.883899607609
        pose_goal.orientation.y = -0.467562344368
        pose_goal.orientation.z = -0.00652369106187
        pose_goal.position.x = -0.082724855514
        pose_goal.position.y = 0.582823716523
        pose_goal.position.z = 0.582901434879
        waypoints.append(pose_goal)

        new_eef_pose4.position.x = pose_goal.position.x
        new_eef_pose4.position.y = pose_goal.position.y + 0.2
        new_eef_pose4.position.z = pose_goal.position.z - 0.2

        # Retain orientation of the current pose.
        new_eef_pose4.orientation = copy.deepcopy(pose_goal.orientation)

        waypoints.append(new_eef_pose4)
        # waypoints.append(current_pose.pose)

        # We want the cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in cartesian
        # translation.  We will specify the jump threshold as 0.0, effectively
        # disabling it.
        fraction = 0.0
        for count_cartesian_path in range(0, 10):
            if fraction < 1.0:
                (plan_cartesian, fraction) = self.robot_group.compute_cartesian_path(
                    waypoints,  # waypoints to follow
                    0.01,  # eef_step
                    0.0)  # jump_threshold
            else:
                break
        rospy.loginfo("fraction is: " + str(fraction))
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        robot1_goal.trajectory = plan_cartesian

        self.robot1_client.send_goal(robot1_goal)
        self.robot1_client.wait_for_result()

    def grasp_object(self):
        rospy.wait_for_service("/ur/magnetic_gripper/on")
        try:
            grip_object = rospy.ServiceProxy("/ur/magnetic_gripper/on", std_srvs.srv.Empty)
            resp1 = grip_object()
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def release_object(self):
        rospy.wait_for_service("/ur/magnetic_gripper/off")
        try:
            grip_object = rospy.ServiceProxy("/ur/magnetic_gripper/off", std_srvs.srv.Empty)
            resp1 = grip_object()
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def move_to_wait(self):
        self.robot_group.set_named_target("wait")

        # Plan to the desired joint-space goal using the default planner (RRTConnect).
        plan = self.robot_group.plan()
        # Create a goal message object for the action server.
        robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message.
        robot1_goal.trajectory = plan

        # Send the goal to the action server.
        self.robot1_client.send_goal(robot1_goal)
        res = self.robot1_client.wait_for_result()

        if res:
            ctrl_c = False
            while not ctrl_c:
                if self.pick_and_place_done_pub.get_num_connections() > 0:
                    msg = Int32()
                    msg.data = 10
                    self.pick_and_place_done_pub.publish(msg)
                    ctrl_c = True


if __name__ == '__main__':


    try:
        obj = PickAndPlace()
        obj.simple_pick_place()
        # obj.move_to_target()
        # obj.grasp_object()
        # obj.move_from_target_to_goal()
        # obj.release_object()
        # obj.move_to_wait()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
