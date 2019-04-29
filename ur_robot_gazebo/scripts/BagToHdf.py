#!/usr/bin/env python
import h5py
import rosbag
import numpy as np


arm_pose_topic_name = "arm_position_topic"
goal_pose_topic_name = "goal_object_pose"
target_pose_topic_name = "target_object_pose"
magnetic_gripper_topic_name = "magnetic_gripper_state"
joint_angles_topic_name = "joint_position"
overall_min_length = 0

def get_bag_info():
    bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/armPose.bag')
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])

    datasetLength = len(topics)

    print("Length of type is: " + str(len(types)))

    return datasetLength


def create_training_data(datasetLength):

    finalList = []

    for n in range(0, datasetLength):
        arrayLength = []
        mainData = []

        data1 = []
        bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/armPose.bag')

        for topic, msg, t in bag.read_messages(topics=[arm_pose_topic_name + "_" + str(n)]):
            #print msg
            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)

            temp.append(msg.pose.orientation.x)
            temp.append(msg.pose.orientation.y)
            temp.append(msg.pose.orientation.z)
            temp.append(msg.pose.orientation.w)

            data1.append(temp)

        arrayLength.append(len(data1))
        print("There are " + str(len(data1)) + " objects in this bag")
        mainData.append(data1)
        bag.close()

        # --------------------------------------------------------------------

        data2 = []
        bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/goalPose.bag')

        for topic, msg, t in bag.read_messages(topics=[goal_pose_topic_name + "_" + str(n)]):

            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)

            temp.append(msg.pose.orientation.x)
            temp.append(msg.pose.orientation.y)
            temp.append(msg.pose.orientation.z)
            temp.append(msg.pose.orientation.w)

            data2.append(temp)

        arrayLength.append(len(data2))
        print("There are " + str(len(data2)) + " objects in this bag")
        mainData.append(data2)
        bag.close()

        # --------------------------------------------------------------------

        data5 = []
        bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/targetPose.bag')
        for topic, msg, t in bag.read_messages(topics=[target_pose_topic_name + "_" + str(n)]):
            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)

            temp.append(msg.pose.orientation.x)
            temp.append(msg.pose.orientation.y)
            temp.append(msg.pose.orientation.z)
            temp.append(msg.pose.orientation.w)

            data5.append(temp)

        arrayLength.append(len(data5))
        print("There are " + str(len(data5)) + " objects in this bag")
        mainData.append(data5)
        bag.close()

        # --------------------------------------------------------------------

        data4 = []
        bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/magneticGripperState.bag')
        for topic, msg, t in bag.read_messages(topics=[magnetic_gripper_topic_name + "_" + str(0)]):
            #print msg
            temp = []
            if msg.gripperState:
                temp.append(1)

            else:
                temp.append(0)

            data4.append(temp)

        arrayLength.append(len(data4))
        print("There are " + str(len(data4)) + " objects in this bag")
        mainData.append(data4)
        bag.close()

        # --------------------------------------------------------------------

        data3 = []
        bag = rosbag.Bag('/home/abdollah/Documents/ROSBAGs/JointPosition.bag')
        for topic, msg, t in bag.read_messages(topics=[joint_angles_topic_name + "_" + str(n)]):
            #print msg
            data3.append(msg.position)

        arrayLength.append(len(data3))
        print("There are " + str(len(data3)) + " objects in this bag")
        mainData.append(data3)
        bag.close()

        # --------------------------------------------------------------------

        print(arrayLength)

        for i in range(0, len(mainData)):
            if len(mainData[i]) > min(arrayLength):
                num = len(mainData[i]) - min(arrayLength)
                for j in range(0, num):
                    mainData[i].pop()

            print("The new length is: " + str(len(mainData[i])))

        dataArray = []

        for i in range(0, min(arrayLength)):
            row = []
            for j in range(0, len(mainData)):
                element = mainData[j][i]
                for e in range(0 , len(element)):
                    row.append(element[e])

            dataArray.append(row)

        finalList.append(dataArray)

    return finalList


def trim_data_set(data_set):
    data_set_lens = []

    for i in range(0 , len(data_set)):
        data_set_lens.append(len(data_set[i]))

    min_len = min(data_set_lens)

    for i in range(0 , len(data_set)):
        if len(data_set[i]) > min_len:
            data_set[i].pop()

    return data_set


def convert_to_numpy(finalList):
    d1 = len(finalList)
    d2 = len(finalList[0])
    d3 = len(finalList[0][0])

    a = np.zeros(shape=(d1, d2, d3))

    for i in range(0, d1):
        for j in range(0, d2):
            for e in range(0, d3):
                print(finalList[i][j][e])
                a[i, j, e] = finalList[i][j][e]

    return a


# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------

def save_to_hdf(array, databaseRef):
    f = databaseRef
    print("Opening/Creating database ")

    # g = f.create_group("training_data")
    # create the image space to be stored for the training data
    f.create_dataset("training_data", data=array)

    print("Closing database")
    f.close()


def obtain_training_set(databaseRef):
    f = databaseRef
    group_list = list(f.keys())
    n = len(group_list)

    returnkey = group_list[0]
    # return f.get(returnkey).value
    return f[returnkey].value


if __name__ == '__main__':

    datasetLength = get_bag_info()
    data_list = create_training_data(datasetLength)

    data_list = trim_data_set(data_list)
    data_array = convert_to_numpy(data_list)

    f = h5py.File('/home/abdollah/Documents/training_data.hdf5', 'a')
    # save_to_hdf(data_array , f)

    data = obtain_training_set(f)

    d1 = data[0,0,:]

    print("lets check")
