#!/usr/bin/env python
import sys, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from ur_robot_gazebo.msg import PoseMessageSimple
from std_msgs.msg import Bool
import random
import string


class ObjectSpawner:
    add_object = None
    delete_object = None
    model_paths = []
    model_names = []
    targetName = ""
    test_pub = None

    objectExists = False


    def __init__(self):
        self.model_paths.append("/home/abdollah/ros_ws/src/UR_Robot_AI/ur_robot/models/green_brick")
        self.model_names.append("green_brick.sdf")

        rospy.init_node("object_spawner")
        # rospy.wait_for_service("gazebo/delete_model")

        rospy.wait_for_service("gazebo/spawn_sdf_model")

        self.add_object = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        # self.delete_object = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

        rospy.Subscriber("/ur_robot/target_obj_init_pose", PoseMessageSimple, self.spawn_callback)
        rospy.Subscriber("/ur_robot/delete_target_object", Bool, self.delete_obj_callback)

        self.test_pub = rospy.Publisher("/ur_robot/delete_cube_done", Bool, queue_size=1)

    def spawn_callback(self, data):

        if not self.objectExists:
            orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
            object_pose = Pose(Point(data.position[0], data.position[1], data.position[2]), orient)

            with open(self.model_paths[0] + "/" + self.model_names[0], "r") as f:
                model = f.read()
            self.targetName = ""
            # print(model)
            self.targetName = "cube" + self.randomString()
            print("Model name for spawn is: " + self.targetName)
            print(self.add_object(self.targetName, model, "", object_pose, "world"))
            self.objectExists = True
        else:
            print("Request came in for spawning a new object while the previous object is still not deleted")

    def delete_obj_callback(self, data):

        if self.objectExists:

            rospy.wait_for_service("gazebo/delete_model")
            self.delete_object = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

            print("Model name for delete is: " + self.targetName)
            result = self.delete_object(self.targetName)
            print "wait for object delete to complete"
            if result.success:
                self.objectExists = False
                print result.status_message
                print "Sending back delete flag!!!"
                self.send_delete_cube_flag(True)
            else:
                print "delete of cube not successful"
                print "Sending back delete flag!!!"
                self.send_delete_cube_flag(False)
        else:
            print("Request came in to delete an object that does not exist")

    def send_delete_cube_flag(self, value):
        ctrl_c = False
        while not ctrl_c:
            # print("Sending done status back")
            if self.test_pub.get_num_connections() > 0:
                print("Sending done status back - after checking connection")
                msg = Bool()
                msg.data = value
                self.test_pub.publish(msg)
                ctrl_c = True

    def randomString(self):
        stringLength = 20
        """Generate a random string of fixed length """
        letters = string.ascii_letters
        rand = ''.join(random.choice(letters) for i in range(stringLength))
        return rand


if __name__ == '__main__':

    try:
        obj = ObjectSpawner()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except:
        e = sys.exc_info()[0]
        print "<p>Error: %s</p>" % e
