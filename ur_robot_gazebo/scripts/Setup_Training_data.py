#!/usr/bin/env python
# coding: utf-8
# This script is responsible for padding the training data and categorizing them in too different length groups.

import numpy as np
import tensorflow as tf
import h5py
import math
from tensorflow.python import debug as tf_debug






class SetUpData:

    source_data_path = None
    grouped_data_path = None

    test_data_path = None

    train_data_path = None

    test_data_index_list = []

    categories_lengths = [100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200]
    category_list = []

    episode_lens = None
    raw_data = None



    # -------------------
    group_list = []
    group_index = None


    #-------------------
    default_batch_size = None

    #--------------------


    def __init__(self):
        print("Class Created!!")

    def setup_stream(self , source_path , destination_path , test_data_path , train_data_path):
        self.source_data_path = source_path
        self.grouped_data_path = destination_path
        self.test_data_path = test_data_path
        self.train_data_path = train_data_path


    def obtain_training_set(self, f):

        """
        @Method
        @Description: Reads the content of the HDF5 file in memory
        @return: {matrix} - Actual data read from the HDF5 file
        """
        keys = f.keys()

        group_list = list(f.keys())
        n = len(group_list)

        item = f[group_list[0]]

        return item.value, f[group_list[1]].value


    def divide_data_corpus(self , factor):

        test_data_lengths = []

        test_data_raw_data = []

        with h5py.File(self.source_data_path, 'a') as f:
            self.episode_lens, self.raw_data = self.obtain_training_set(f)

            total_num_of_episodes = len(self.episode_lens)

            num_of_test_data = math.floor(total_num_of_episodes * factor)

            num_of_test_data = 300

            index = 2

            for i in range(0,num_of_test_data):

                total_num_of_episodes = len(self.episode_lens)

                # index = np.random.randint(low=0, high=total_num_of_episodes)

                self.test_data_index_list.append(index)

                selected_test_length = self.episode_lens[index]

                test_data_lengths.append(selected_test_length)

                start_index = np.sum(self.episode_lens[:index])
                end_index = start_index + selected_test_length

                selected_test_data = self.raw_data[start_index:end_index,:]

                test_data_raw_data.append(selected_test_data)

                index +=2

        print("Look")







        arr1 = test_data_raw_data[0]
        for i in range(1 , len(test_data_raw_data)):
            arr1 = np.concatenate((arr1 , test_data_raw_data[i]) , axis=0)


        self.save_to_hdf(arr1 , test_data_lengths , self.test_data_path , True)

        print("Done Deleting!!!")


    def delete_test_data_from_corpus(self):

        length = len(self.test_data_index_list)


        for i in range(0 , length) :
            index = self.test_data_index_list[i]

            episode_length = self.episode_lens[index]

            start_index =  np.sum(self.episode_lens[:index])
            end_index = start_index + episode_length

            self.raw_data = np.delete(self.raw_data, obj=slice(start_index, end_index), axis=0)

            self.episode_lens = np.delete(self.episode_lens , index)

            self.test_data_index_list = np.subtract(self.test_data_index_list , 1)


        self.save_to_hdf(self.raw_data , self.episode_lens , self.train_data_path , True)
        print("Done Deleting from the corpus")


    def save_to_hdf(self, data_matrix, episode_len_array , hdf_file_path , include_meta_data = True):

        """
        @Method
        @Description: Saves the dataset in Numpy format to HDF5 file
        :param data_matrix: Matrix as Numpy type that holds the dataset
        :param episode_len_array: A Numpy array holding the length of each simulation episode
        """

        with h5py.File(hdf_file_path, 'a') as f:

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



    def read_raw_data(self , path):
        with h5py.File(path, 'a') as f:
            self.episode_lens  , self.raw_data = self.obtain_training_set(f)
            copy = self.episode_lens

            for num in self.categories_lengths:

                temp_array = []

                for i in range(0 , len(self.episode_lens)):
                    episode_len = self.episode_lens[i]
                    floor = num - 5
                    ceiling = num + 5

                    if(episode_len < ceiling and episode_len >= floor):
                        temp_array.append(i)

                self.category_list.append(temp_array)

        print("Done!!!")


    def convert_to_numpy(self, data_list):

        """
        @Method
        @Description: Method to convert dataset from list type to Numpy array type
        :param {List} data_list: Dataset as a list datatype
        :return: {Numpy array - Numpy array} Dataset in Numpy array form - Episode length array in Numpy array form.
        """
        if(len(data_list) > 0):

            final_matrix = np.array(data_list[0])

            for i in range(1, len(data_list)):
                temp = np.array(data_list[i])
                final_matrix = np.concatenate((final_matrix, temp), axis=0)


            return final_matrix


    def group_raw_data(self):
        with h5py.File(self.grouped_data_path, 'a') as f:


            for i in range(0 , len(self.category_list)):

                item = self.categories_lengths[i]
                list = self.category_list[i]

                group_name = "data_length" + "_" + str(item)
                group_ref = f.create_group(group_name)

                group_list = []
                group_length_list = []
                for index in list:
                    episode_length = self.episode_lens[index]

                    group_length_list.append(episode_length)

                    start_index =  np.sum(self.episode_lens[:index])
                    end_index = start_index + episode_length

                    chunck_data = self.raw_data[start_index:end_index , :]


                    diff = chunck_data.shape[0] - item

                    if diff > 0:
                       for j in range(0 , diff):
                          chunck_data = np.delete(chunck_data , chunck_data.shape[0] - 1 , axis= 0)
                    elif diff < 0:
                        final_row = chunck_data[chunck_data.shape[0] - 1, :]
                        final_row = np.reshape(final_row, (1, final_row.shape[0]))
                        for j in range(0 , -1 * diff):
                            chunck_data = np.concatenate((chunck_data, final_row), axis=0)

                    chunck_data = np.reshape(chunck_data , (1, chunck_data.shape[0] , chunck_data.shape[1]))


                    group_list.append(chunck_data)

                    # print("Progressing!!!")

                converted_to_numpy = self.convert_to_numpy(group_list)


                if(converted_to_numpy is not None):
                    rotate_converted = np.rot90(converted_to_numpy, axes=(1, 2))
                    rotate_converted = np.rot90(rotate_converted, axes=(0, 1))
                    # print("Convert")

                    group_ref.create_dataset("Data", data= rotate_converted , chunks=True )


    def read_grouped_data(self , group_index):
        with h5py.File(self.grouped_data_path, 'a') as f:

            group_list = list(f.keys())

            item = f[group_list[group_index]]

            dataset = f.get(item.name + "/Data")

            return dataset.value


            # print("INFO: Test data for an episode has been saved to HDF5 file in -> ", file_path)

    def read_next_time_frame(self , grouped_data):

        if(self.default_batch_size > grouped_data.size[1]):
            # read a total of 4 sequences from the data
            return_data = grouped_data[:,:,]




if __name__=="__main__":
    src =  "/home/abdollah/Documents/New_HDF/Cleaned/clean_trimmed.hdf5"


    train_only_data_path = "/home/abdollah/Documents/New_HDF/Cleaned/train_only_data.hdf5"
    grouped_training_data_path = "/home/abdollah/Documents/New_HDF/Cleaned/grouped_data.hdf5"
    test_data_src = "/home/abdollah/Documents/New_HDF/Cleaned/test_data.hdf5"


    object = SetUpData()

    object.setup_stream(src, grouped_training_data_path , test_data_src , train_only_data_path)

    object.divide_data_corpus(0.01)

    object.delete_test_data_from_corpus()

    object.read_raw_data(train_only_data_path)
    object.group_raw_data()

    # group_data_retunred = object.read_grouped_data(0)

    print("End!!!!")