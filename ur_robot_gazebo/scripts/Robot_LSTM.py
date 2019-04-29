import numpy as np
from random import shuffle
import tensorflow as tf
from tensorflow.contrib import rnn
from BagToHdf import *

seq_len = 4
input_len = 28
batch_size = 1
sim_len = 152
num_of_runs = 2

num_of_training_data = 0
num_of_test_data = 0

num_of_training_labels = 0
num_of_test_labels = 0

test_num_ratio = 0.125

# Import the dataset the hdf file
f = h5py.File('/home/abdollah/Documents/training_data.hdf5', 'a')
data = obtain_training_set(f)


# Create the training data and the labels from the corpus.
train_data = []
train_labels = []
test_data = []
test_labels = []

for e in range(0, num_of_runs):
    for i in range(0, sim_len - 4):
        temp = []
        for j in range(0, seq_len):
            temp.append(data[e, i + j, :])
            if j == 3:
                train_labels.append(data[e, i + j + 1, :])
        train_data.append(temp)


num_of_test_data = int(len(train_data) * test_num_ratio)
num_of_training_data = len(train_data) - num_of_test_data

num_of_test_labels = int(len(train_labels) * test_num_ratio)
num_of_training_labels = len(train_labels) - num_of_test_labels

test_data = train_data[num_of_training_data:]
train_data = train_data[:num_of_training_data]

test_labels = train_labels[num_of_training_labels:]
train_labels = train_labels[:num_of_training_labels]


print("Checking!!!!")
print("train data length -> " + str(len(train_data)))
print("test data length -> " + str(len(test_data)))

print("train labels length -> " + str(len(train_labels)))
print("test labels length -> " + str(len(test_labels)))


# ------------------------------------------------
# ------------------------------------------------
# This section is related to the implementation of the LSTM model

# Create the require placeholders.
data = tf.placeholder(tf.float64, [None, seq_len, input_len])
target = tf.placeholder(tf.float64, [None, input_len])

print_target = tf.Print(target, [target] , "This is:")


# Initialize an LSTM cell
num_hidden = 128

cell = rnn.LSTMCell(num_hidden, state_is_tuple=True)


# Create the computation graph

val, state = tf.nn.dynamic_rnn(cell, data, dtype=tf.float64)

val = tf.transpose(val, [1, 0, 2])
last = tf.gather(val, int(val.get_shape()[0]) - 1)


weight = tf.Variable(tf.truncated_normal([num_hidden, int(target.get_shape()[1])] , dtype=tf.float64) , dtype=tf.float64)
bias = tf.Variable(tf.constant(0.1, shape=[target.get_shape()[1]], dtype=tf.float64 ), dtype=tf.float64)

# --- This is the prediction phase -> We have to find out what type we will use or in other words
# how will ours be predicted!!!!
prediction = tf.matmul(last, weight) + bias
prediction_print = tf.Print(prediction , [prediction] , "prediction is: ")

cross_entropy = -tf.reduce_sum(target * tf.log(prediction))
# cross_entropy = tf.nn.sigmoid_cross_entropy_with_logits(logits=prediction, labels=target)

optimizer = tf.train.AdamOptimizer()
minimize = optimizer.minimize(cross_entropy)

mistakes = tf.not_equal(print_target, prediction_print)
# error = tf.reduce_mean(tf.cast(mistakes, tf.float64))
error = tf.Variable((target - prediction)**2, name='loss')

# Setting up Tensorflow for execution

init_op = tf.initialize_all_variables()
sess = tf.Session()
print("Started session")
sess.run(init_op)


no_of_batches = int(len(train_data)/batch_size)
epoch = 250

print("Going in to the for loop")
for i in range(epoch):
    # print("epoch: " + str(epoch))
    ptr = 0
    for j in range(no_of_batches):
        # print("in minimize loop -> " + str(no_of_batches))
        inp = train_data[ptr:ptr + batch_size]
        out = train_labels[ptr:ptr + batch_size]
        ptr += batch_size
        sess.run(minimize,{data: inp, target: out})
    print "Epoch - " + str(i) + " from " + str(epoch)

incorrect = sess.run(error, {data: test_data, target: test_labels})
print('Epoch {:2d} error {:3.1f}%'.format(i + 1, 100 * incorrect))
sess.close()

print("Done!!!")