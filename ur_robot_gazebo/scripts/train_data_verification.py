#!/usr/bin/env python
# This code goes through the training data in the HDF files and verfies that each episode has been
# completed and the cube is within the range of the baskets boundries.

import h5py
import numpy as np


class VerifyData:
    __source_path = ""
    __data = None
    __time_list = None
    __length_to_check = None
    __invalid_data_indices = []
    __valid_data_indices = []
    __goal_fence_width = None
    __goal_fence_length = None
    __acceptable_range = None
    __issues_path = None

    __clean_training_data_path = None


    ttt = None

    def __init__(self, src_path, length_to_check , goal_width , goal_length , issues_path , clean_path):
        self.__source_path = src_path
        self.__length_to_check = length_to_check
        self.__goal_fence_length = goal_length
        self.__goal_fence_width = goal_width
        self.__acceptable_range = np.sqrt((self.__goal_fence_width**2) + (self.__goal_fence_length**2))
        self.__issues_path = issues_path
        self.__clean_training_data_path = clean_path

        self.ttt = []
        self.ttt_time = []

    def read_source(self):
        with h5py.File(self.__source_path, 'a') as f:

            group_list = list(f.keys())

            self.__time_list = f[group_list[0]].value
            self.__data = f[group_list[1]].value

    def validate_data(self):
        for i in range(0, len(self.__time_list)):
            temp = self.__time_list[i]

            index1 = np.sum(self.__time_list[0:i]) + (temp - self.__length_to_check)
            index2 = np.sum(self.__time_list[0:i]) + temp
            goal_pose = self.__data[index1:index2, 6:8]
            target_pose = self.__data[ index1:index2, 12:14]

            in_range , list = self.is_in_range(goal_pose , target_pose)

            if not in_range:
                # print("Invalid data found and added to list")
                self.__invalid_data_indices.append(i)
            else:
                self.__valid_data_indices.append(i)



        self.save_issues_to_disk()
        self.delete_incorrect_data()

        print("Data validation process finished and a total amount of " + str(len(self.__invalid_data_indices)) +
              " issues has been found and a total number of " + str(len(self.__valid_data_indices)) + " valid iterations")

    def is_in_range(self, v, u):
        distance = np.sqrt(np.sum((v - u) ** 2, axis=1))
        result = (np.less_equal(distance , self.__acceptable_range)).astype(int)
        if  np.sum(result) == self.__length_to_check:
            return True , result
        else:
            return False , result

    def delete_incorrect_data(self):

        temp = self.__invalid_data_indices.copy()

        while len(temp) > 0:
            t0 = temp.pop(0)
            t1 = self.__time_list[t0]
            t2 = self.__time_list[0:t0]
            t3 = np.sum(t2)
            t4 = (t3 + t1)
            self.__data = np.delete(self.__data , obj=slice(t3 , t4) , axis=0)
            self.__time_list = np.delete(self.__time_list , obj=t0 , axis=0)
            temp[:] = [x - 1 for x in temp]

        self.save_to_hdf(self.__data , self.__time_list , self.__clean_training_data_path)

    def save_issues_to_disk(self):
        for i in range(0, len(self.__invalid_data_indices)):
            t0 = self.__invalid_data_indices[i]
            t1 = self.__time_list[t0]
            t2 = self.__time_list[0:t0]
            t3 = np.sum(t2)
            t4 = (t3+t1)
            t5 = self.__data[t3: t4, :]

            self.ttt.append(t5)
            self.ttt_time.append(t1)

        tr , tr1 = self.convert_to_numpy(self.ttt)

        self.save_to_hdf(tr , tr1 , self.__issues_path)


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


    def save_to_hdf(self, data_matrix, episode_len_array , path):

        """
        @Method
        @Description: Saves the dataset in Numpy format to HDF5 file
        :param data_matrix: Matrix as Numpy type that holds the dataset
        :param episode_len_array: A Numpy array holding the length of each simulation episode
        """

        with h5py.File(path, 'a') as f:

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

if __name__ == "__main__":

    train_data_path = "/home/abdollah/Documents/New_HDF/Cleaned/clean_long.hdf5"
    # train_data_path = "/home/abdollah/Documents/New_HDF/Short/training_data_short.hdf5"
    # train_data_path = "/home/abdollah/Documents/New_HDF/Med/training_data_medium.hdf5"
    # train_data_path = "/home/abdollah/Documents/New_HDF/Long/training_data_long.hdf5"

    issues_file_path = "/home/abdollah/Documents/New_HDF/Issues/issues_long_2.hdf5"

    clean_path_hdf = "/home/abdollah/Documents/New_HDF/Cleaned/clean_long_2.hdf5"

    length_to_check = 10
    goal_fence_width = 0.2
    goal_fence_length = 0.2

    verify_data = VerifyData(train_data_path,
                             length_to_check=length_to_check,
                             goal_length=goal_fence_length,
                             goal_width=goal_fence_width,
                             issues_path=issues_file_path,
                             clean_path=clean_path_hdf)
    verify_data.read_source()
    verify_data.validate_data()

