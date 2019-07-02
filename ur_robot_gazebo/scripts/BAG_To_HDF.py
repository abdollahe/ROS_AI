#!/usr/bin/env python
import h5py
import rosbag
import numpy as np
from tf import transformations
import os


topic_list = ["arm_position_topic",
              "goal_object_pose",
              "joint_position",
              "joint_velocity",
              "magnetic_gripper_state",
              "target_object_pose"]

file_stem_name = ["armPose",
                  "goalPose",
                  "JointPosition",
                  "Jointvelocity",
                  "magneticGripperState",
                  "targetPose"]

# root path to search for bag files.
root_path = "/home/abdollah/Documents/ROSBAGs"

hdf_file_path = '/home/abdollah/Documents/verify_training_set_long.hdf5'

# -------------------------------------------------------------------- #
# -------------------------------------------------------------------- #


class BagToHdf :

    # search path
    root_path = ""

    # list of topic names to extract data
    topic_names = []

    # list containing stem of file names to process
    bag_file_stems = []

    # Lists that will contain the each relating topic bag files
    arm_pose_files = []
    goal_pose_files = []
    target_pose_files = []
    magnetic_griper_files = []
    joint_angles_files = []
    joint_velocity_files = []

    overall_min_length = 0

    f = None
    hdf_file_path = ""

    def __init__(self):
        print("Class created")

    def set_root_path(self , r_path):
        """
        @Method
        @Description: set the path to search for bag files
        :param {string} r_path: Path to search for bag files
        """
        self.root_path = r_path

    def set_topic_names(self , topic_name_list):
        """
        @Method
        @Description : set the name of topics to extract data from the bag files
        :param {list} topic_name_list: A list of strings containing the topic names interested in.
        """
        self.topic_names = topic_name_list

    def set_stem_file_names(self , stem_names):
        """
        @Method
        @Description: set the stem of files names to search for
        :param {list} stem_names: A list of strings containing the stem of the file names to search for
        """
        self.bag_file_stems = stem_names

    def set_hdf_path(self , h_path):
        """
        @Method
        @Description: Set he target HDF5 file path
        :param {String} h_path: String representing the HDF5 file path to store the dataset
        """
        self.hdf_file_path = h_path


    def list_files(self):
        """
        @Method
        @Description: lists the bag files that contain the stem names specified
        :return: {list} A list of the list of bags files found in the path specified
        """
        main_list = []

        for j in range(0 , len(self.bag_file_stems)):
            temp_list = []
            for root, dirs, files in os.walk(self.root_path):
                for file in files:
                    if file.__contains__(self.bag_file_stems[j]):
                        temp_list.append(file)

            temp_list.sort()
            main_list.append(temp_list)

        return main_list

    def get_bag_info(self, file_name):

        """
        @Method
        @Description: Gets information of a set of bag files from one of the bag files
        :param {String} file_name: Name of a bag file with in a series of simulation run
        :return: {Int , Int} - Number of topics available in the series , minimum topic index in the bag file
        """

        print("INFO: Reading bag file and gathering info")

        bag = rosbag.Bag(self.root_path + "/" + file_name )
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []

        for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        num_of_topics_in_series = len(topics)

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

        min_val = self.extract_min_index(topics, final_index)

        print("Length of type is: " + str(len(types)))

        return num_of_topics_in_series , min_val

    def extract_min_index(self, arr, start_index):

        """
        @Method
        @Description: Extracts the minimum index from a list
        :param {List} arr: The list of indices
        :param {Int} start_index:
        :return: minimum index in a a bag file
        """
        num_arr = []

        for i in range(len(arr)):
            temp = arr[i][(start_index + 1):]
            num_arr.append(int(temp))

        return min(num_arr)

    def create_training_data(self, num_topics, min_topic_index, bag_file_list):


        finalList = []

        for n in range(0, num_topics):
            arrayLength = []
            mainData = []

            pose_data = []
            print("INFO: Opening Bag file " + bag_file_list[0] + " to read and extracts topics")
            bag = rosbag.Bag( self.root_path + "/" + bag_file_list[0])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[0] + "_" + str(min_topic_index + n)]):

                temp = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

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

                pose_data.append(temp)

            arrayLength.append(len(pose_data))
            print("INFO: A total number of " + str(len(pose_data)) + " topics where extracted from the Bag file :" + bag_file_list[0])
            mainData.append(pose_data)
            bag.close()

            # --------------------------------------------------------------------

            goal_data = []
            print("INFO: Opening Bag file " + bag_file_list[1] + " to read and extracts topics")
            bag = rosbag.Bag( self.root_path + "/" + bag_file_list[1])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[1] + "_" + str(min_topic_index + n)]):

                temp = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

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

                goal_data.append(temp)

            arrayLength.append(len(goal_data))
            print("INFO: A total number of " + str(len(goal_data)) + " topics where extracted from the Bag file :" + bag_file_list[1])

            mainData.append(goal_data)
            bag.close()

            # --------------------------------------------------------------------

            target_data = []

            print("INFO: Opening Bag file " + bag_file_list[5] + " to read and extracts topics")
            bag = rosbag.Bag(self.root_path + "/" + bag_file_list[5])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[5] + "_" + str(min_topic_index + n)]):

                temp = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

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

                target_data.append(temp)

            arrayLength.append(len(target_data))
            print("INFO: A total number of " + str(len(target_data)) + " topics where extracted from the Bag file :" + bag_file_list[5])
            mainData.append(target_data)
            bag.close()

            # --------------------------------------------------------------------

            gripper_state_data = []

            print("INFO: Opening Bag file " + bag_file_list[4] + " to read and extracts topics")
            bag = rosbag.Bag(self.root_path + "/" + bag_file_list[4])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[4] + "_" + str(min_topic_index + n)]):

                temp = []
                if msg.gripperState:
                    temp.append(1)

                else:
                    temp.append(0)

                gripper_state_data.append(temp)

            arrayLength.append(len(gripper_state_data))
            print("INFO: A total number of " + str(len(gripper_state_data)) + " topics where extracted from the Bag file :" + bag_file_list[4])
            mainData.append(gripper_state_data)
            bag.close()

            # --------------------------------------------------------------------

            joint_angles_data = []

            print("INFO: Opening Bag file " + bag_file_list[2] + " to read and extracts topics")
            bag = rosbag.Bag(self.root_path + "/" + bag_file_list[2])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[2] + "_" + str(min_topic_index + n)]):

                joint_angles_data.append(msg.position)

            arrayLength.append(len(joint_angles_data))
            print("INFO: A total number of " + str(len(joint_angles_data)) + " topics where extracted from the Bag file :" + bag_file_list[2])
            mainData.append(joint_angles_data)
            bag.close()

            # --------------------------------------------------------------------
            # --------------------------------------------------------------------

            joint_velocity_data = []

            print("INFO: Opening Bag file " + bag_file_list[3] + " to read and extracts topics")
            bag = rosbag.Bag(self.root_path + "/" + bag_file_list[3])

            for topic, msg, t in bag.read_messages(topics=[self.topic_names[3] + "_" + str(min_topic_index + n)]):

                joint_velocity_data.append(msg.velocity)

            arrayLength.append(len(joint_velocity_data))
            print("INFO: A total number of " + str(len(joint_angles_data)) + " topics where extracted from the Bag file :" + bag_file_list[3])
            mainData.append(joint_velocity_data)
            bag.close()

            # --------------------------------------------------------------------

            # for i in range(0, len(mainData)):
            #     if len(mainData[i]) > min(arrayLength):
            #         num = len(mainData[i]) - min(arrayLength)
            #         for j in range(0, num):
            #             mainData[i].pop()
            #
            #     print("The new length is: " + str(len(mainData[i])))

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

    def trim_data_set(self, data_set):

        """
        @Method
        @Description: Method to delete the incorrect episodes from the dataset
        :param { List } data_set: The dataset in list form
        :return: { List } A dataset holding only correct episodes (Time wise)
        """

        data_set_lengths = []

        for i in range(0 , len(data_set)):
            data_set_lengths.append(len(data_set[i]))

        max_len = max(data_set_lengths)

        pop_list_index = []
        for i in range(0, len(data_set_lengths)):
            # if max_len - data_set_lengths[i] > 50:
            #     pop_list_index.append(i)
            #

            if data_set_lengths[i] < 80:
                pop_list_index.append(i)

        for i in range(0, len(pop_list_index)):
            data_set.pop(pop_list_index[i])
            pop_list_index = [x-1 for x in pop_list_index]

        return data_set

    def filter_data_set(self, data_set ,min_range, max_range):
        """
        @Method
        @Description: Filter and returns a dataset containing the specified range
        :param { List } data_set: The dataset in list form
        :param { Int } min_range: Lower bound of the range (in milliseconds)
        :param { Int } max_range: Higher bound of the range (in milliseconds)
        :return: { List } A dataset holding only requested episodes (With in specified time range)
        """

        data_set_lengths = []

        for i in range(0 , len(data_set)):
            data_set_lengths.append(len(data_set[i]))

        pop_list_index = []
        for i in range(0, len(data_set_lengths)):

            dd = data_set_lengths[i]

            cond1 = dd < min_range

            cond2 = dd > max_range

            if (data_set_lengths[i] < min_range) or (data_set_lengths[i] > max_range):
                pop_list_index.append(i)

        for i in range(0, len(pop_list_index)):
            data_set.pop(pop_list_index[i])
            pop_list_index = [x-1 for x in pop_list_index]

        return data_set

    def convert_to_numpy(self, data_list):

        """
        @Method
        @Description: Method to convert dataset from list type to Numpy array type
        :param {List} data_list: Dataset as a list datatype
        :return: {Numpy array - Numpy array} Dataset in Numpy array form - Episode length array in Numpy array form.
        """
        final_matrix = np.array(data_list[0])
        section_length_vector = [len(data_list[0])]

        for i in range(1, len(data_list)):
            temp = np.array(data_list[i])
            final_matrix = np.concatenate((final_matrix, temp), axis=0)
            section_length_vector.append(len(data_list[i]))

        return final_matrix, np.array(section_length_vector)

    def save_to_hdf(self, data_matrix, episode_len_array):

        """
        @Method
        @Description: Saves the dataset in Numpy format to HDF5 file
        :param data_matrix: Matrix as Numpy type that holds the dataset
        :param episode_len_array: A Numpy array holding the length of each simulation episode
        """

        with h5py.File(self.hdf_file_path, 'a') as f:

            print("INFO: Attempting to open HDF file to write dataset")

            group_list = list(f.keys())

            if len(group_list) > 0:
                # If dataset exists append the data to it
                n = f["training_data"].shape[0]
                f["training_data"].resize((f["training_data"].shape[0] + data_matrix.shape[0]), axis=0)
                f["training_data"][n:] = data_matrix

                m = f["metadata"].shape[0]
                f["metadata"].resize((f["metadata"].shape[0] + episode_len_array.shape[0]), axis=0)
                f["metadata"][m:] = episode_len_array

            else:
                # Create new datasets and add the data to it
                f.create_dataset("training_data", data=data_matrix , chunks=True, maxshape=(None, data_matrix.shape[1]))
                f.create_dataset("metadata", data=episode_len_array, chunks=True, maxshape=(None,))

            print("INFO: Closing HDF file after successfully writing dataset")


if __name__ == '__main__':

    obj = BagToHdf()

    obj.set_root_path(root_path)
    obj.set_stem_file_names(file_stem_name)
    obj.set_topic_names(topic_list)
    obj.set_hdf_path(hdf_file_path)

    list1 = obj.list_files()

    for j in range(0, len(list1[0])):

        f1 = list1[0][j]

        list2 = [i[j] for i in list1]

        num_of_topics, min_index = obj.get_bag_info(f1)

        data_list = obj.create_training_data(num_of_topics, min_index, list2)

        # data_list = obj.trim_data_set(data_list)

        data_list = obj.filter_data_set(data_list, 100, 250)

        if len(data_list) > 0:
            data_array, second = obj.convert_to_numpy(data_list)
    
            obj.save_to_hdf(data_array, second)

    print("INFO: Data converted from Bag format to HDF5 for a number of " + str(len(list1[0])) + " files")