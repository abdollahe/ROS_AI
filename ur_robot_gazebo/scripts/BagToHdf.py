#!/usr/bin/env python
import h5py
import rosbag
import numpy as np
from tf import transformations
import os.path

arm_pose_topic_name = "arm_position_topic"
goal_pose_topic_name = "goal_object_pose"
target_pose_topic_name = "target_object_pose"
magnetic_gripper_topic_name = "magnetic_gripper_state"
joint_angles_topic_name = "joint_position"
joint_velocity_topic_name = "joint_velocity"
overall_min_length = 0


path = "/home/abdollah/Documents/rosbag_archive/V8"


def get_bag_info():
    bag = rosbag.Bag(path + '/armPose.bag')
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])

    datasetLength = len(topics)

    topics = bag.get_type_and_topic_info()[1].keys()
    str1 = topics[0]

    index = 0
    final_index = 0
    while index < len(str1):
        index = str1.find('_', index)
        if index == -1:
            break
        else:
            final_index = index
        index += 1

    min_val = extract_min_index(topics , final_index)

    print("Length of type is: " + str(len(types)))

    return datasetLength , min_val


def create_training_data(datasetLength, min_val):

    finalList = []

    for n in range(0, datasetLength):
        arrayLength = []
        mainData = []

        data1 = []
        bag = rosbag.Bag( path + '/armPose.bag')

        for topic, msg, t in bag.read_messages(topics=[arm_pose_topic_name + "_" + str(min_val + n)]):
            #print msg
            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)


            quaternion = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)

            euler = transformations.euler_from_quaternion(quaternion)

            temp.append(euler[0])
            temp.append(euler[1])
            temp.append(euler[2])
            # temp.append(msg.pose.orientation.w)

            data1.append(temp)

        arrayLength.append(len(data1))
        print("There are " + str(len(data1)) + " objects in this bag")
        mainData.append(data1)
        bag.close()

        # --------------------------------------------------------------------

        data2 = []
        bag = rosbag.Bag(path + '/goalPose.bag')

        for topic, msg, t in bag.read_messages(topics=[goal_pose_topic_name + "_" + str(min_val + n)]):

            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)

            quaternion = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)

            euler = transformations.euler_from_quaternion(quaternion)

            temp.append(euler[0])
            temp.append(euler[1])
            temp.append(euler[2])
            # temp.append(msg.pose.orientation.w)

            data2.append(temp)

        arrayLength.append(len(data2))
        print("There are " + str(len(data2)) + " objects in this bag")
        mainData.append(data2)
        bag.close()

        # --------------------------------------------------------------------

        data5 = []
        bag = rosbag.Bag( path + '/targetPose.bag')
        for topic, msg, t in bag.read_messages(topics=[target_pose_topic_name + "_" + str(min_val + n)]):
            temp = []
            temp.append(msg.pose.position.x)
            temp.append(msg.pose.position.y)
            temp.append(msg.pose.position.z)

            quaternion = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)

            euler = transformations.euler_from_quaternion(quaternion)

            temp.append(euler[0])
            temp.append(euler[1])
            temp.append(euler[2])
            # temp.append(msg.pose.orientation.w)

            data5.append(temp)

        arrayLength.append(len(data5))
        print("There are " + str(len(data5)) + " objects in this bag")
        mainData.append(data5)
        bag.close()

        # --------------------------------------------------------------------

        data4 = []
        bag = rosbag.Bag( path + '/magneticGripperState.bag')
        for topic, msg, t in bag.read_messages(topics=[magnetic_gripper_topic_name + "_" + str(min_val + n)]):
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
        bag = rosbag.Bag( path + '/JointPosition.bag')
        for topic, msg, t in bag.read_messages(topics=[joint_angles_topic_name + "_" + str(min_val + n)]):
            #print msg
            data3.append(msg.position)

        arrayLength.append(len(data3))
        print("There are " + str(len(data3)) + " objects in this bag")
        mainData.append(data3)
        bag.close()

        # --------------------------------------------------------------------
        # --------------------------------------------------------------------

        data6 = []
        bag = rosbag.Bag( path + '/Jointvelocity.bag')
        for topic, msg, t in bag.read_messages(topics=[joint_velocity_topic_name + "_" + str(min_val + n)]):
            #print msg
            data6.append(msg.velocity)

        arrayLength.append(len(data6))
        print("There are " + str(len(data6)) + " objects in this bag")
        mainData.append(data6)
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
    avg_len = sum(data_set_lens) / len(data_set_lens)
    max_len = max(data_set_lens)

    # plt.hist(data_set_lens, len(data_set_lens))
    # plt.show()

    pop_list_index = []
    for i in range(0, len(data_set_lens)):
        if max_len - data_set_lens[i] > 30:
            pop_list_index.append(i)

        # if data_set_lens[i] < avg_len:
        #     pop_list_index.append(i)

        # if abs(data_set_lens[i] - avg_len) > 5:
        #     pop_list_index.append(i)

    for i in range(0, len(pop_list_index)):
        data_set.pop(pop_list_index[i])
        pop_list_index = [x-1 for x in pop_list_index]

    # for i in range(0 , len(data_set)):
    #     t = len(data_set[i])
    #     if t > min_len:
    #         data_set[i].pop()

    return data_set


def convert_to_numpy(finalList):
    d1 = len(finalList)
    d2 = len(finalList[0])
    d3 = len(finalList[0][0])

    # # final_matrix = np.zeros(shape=(d2, d3))
    # final_matrix = np.array(finalList[0])
    # section_length_vector = []
    # section_length_vector.append(len(finalList[0]))
    #
    # for i in range(1 , d1):
    #     temp = np.array(finalList[i])
    #     final_matrix = np.concatenate((final_matrix, temp), axis=0)
    #     section_length_vector.append(len(finalList[i]))
    #
    #
    # test1 = np.array(section_length_vector)
    # print("Check!!!")


    a = np.zeros(shape=(d1, d2, d3))

    for i in range(0, d1):
        for j in range(0, d2):
            for e in range(0, d3):
                print(finalList[i][j][e])
                a[i, j, e] = finalList[i][j][e]

    return a


def convert_to_numpy2(finalList):
    d1 = len(finalList)

    final_matrix = np.array(finalList[0])
    section_length_vector = [len(finalList[0])]

    for i in range(1 , d1):
        temp = np.array(finalList[i])
        final_matrix = np.concatenate((final_matrix, temp), axis=0)
        section_length_vector.append(len(finalList[i]))

    return final_matrix, np.array(section_length_vector)


def extract_min_index(arr, start_index):

    num_arr = []

    for i in range(len(arr)):
        temp = arr[i][(start_index + 1):]
        num_arr.append(int(temp))

    return min(num_arr)
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


def save_to_hdf2(array, databaseRef , array_length):
    f = databaseRef
    print("Opening/Creating database ")

    group_list = list(f.keys())

    if(len(group_list) > 0):
        # append the data
        n = f["training_data"].shape[0]
        f["training_data"].resize((f["training_data"].shape[0] + array.shape[0]), axis = 0)
        f["training_data"][n:] = array

        m = f["metadata"].shape[0]
        f["metadata"].resize((f["metadata"].shape[0] + array_length.shape[0]), axis = 0)
        f["metadata"][m:] = array_length


    else:
        # g = f.create_group("training_data")
        # create the image space to be stored for the training data
        f.create_dataset("training_data", data=array , chunks=True, maxshape=(None, array.shape[1]))
        f.create_dataset("metadata" , data=array_length , chunks=True, maxshape=(None,))




    print("Closing database")
    f.close()


def obtain_training_set(databaseRef):
    f = databaseRef
    group_list = list(f.keys())
    n = len(group_list)

    returnkey = group_list[0]
    # return f.get(returnkey).value
    return f[returnkey].value


def obtain_training_set2(databaseRef):
    f = databaseRef
    group_list = list(f.keys())
    n = len(group_list)

    return f[group_list[0]].value , f[group_list[1]].value


if __name__ == '__main__':

    datasetLength, min_val = get_bag_info()
    data_list = create_training_data(datasetLength , min_val)

    data_list = trim_data_set(data_list)
    data_array, second = convert_to_numpy2(data_list)

    f = h5py.File('/home/abdollah/Documents/training_data6.hdf5', 'a')
    save_to_hdf2(data_array, f, second)

    # data , data1 = obtain_training_set2(f)
    #
    # # d1 = data[0, 0,:]
    # d2 = data[0]
    print("lets check")
