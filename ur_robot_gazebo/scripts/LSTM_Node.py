import numpy as np
from random import shuffle
import tensorflow as tf
from tensorflow.contrib import rnn

# ---------------------------------------------
# In this section the training data is created
train_input = ['{0:020b}'.format(i) for i in range(2**20)]
shuffle(train_input)
train_input = [map(int,i) for i in train_input]
ti  = []
for i in train_input:
    temp_list = []
    for j in i:
        temp_list.append([j])
    ti.append(np.array(temp_list))
train_input = ti

print("Checking!!!")


# -----------------------------------------------
# In this section the labels are created -> for this example the labels are represented as a one hot encoded vector
# the only one index specifies the number of one in the sequence -> for example if index number 5 is one then there
# are 5 ones in this sequence.

train_output = []

for i in train_input:
    count = 0
    for j in i:
        if j[0] == 1:
            count+=1
    temp_list = ([0]*21)
    temp_list[count]=1
    train_output.append(temp_list)

# ------------------------------------------------
# ------------------------------------------------
NUM_EXAMPLES = 10000
test_input = train_input[NUM_EXAMPLES:]
test_output = train_output[NUM_EXAMPLES:] #everything beyond 10,000

train_input = train_input[:NUM_EXAMPLES]
train_output = train_output[:NUM_EXAMPLES] #till 10,000

# ------------------------------------------------
# ------------------------------------------------
# This section is related to the implementation of the LSTM model

# Create the require placeholders.
data = tf.placeholder(tf.float32, [None, 20, 1])
target = tf.placeholder(tf.float32, [None, 21])


# Initialize an LSTM cell
num_hidden = 24

cell = rnn.LSTMCell(num_hidden , state_is_tuple=True)


# Create the computation graph

val, state = tf.nn.dynamic_rnn(cell, data, dtype=tf.float32)

val = tf.transpose(val, [1, 0, 2])
last = tf.gather(val, int(val.get_shape()[0]) - 1)


weight = tf.Variable(tf.truncated_normal([num_hidden, int(target.get_shape()[1])]))
bias = tf.Variable(tf.constant(0.1, shape=[target.get_shape()[1]]))


# --- This is the prediction phase -> We have to find out what type we will use or in other words
# how will ours be predicted!!!!
prediction = tf.nn.softmax(tf.matmul(last, weight) + bias)

cross_entropy = -tf.reduce_sum(target * tf.log(tf.clip_by_value(prediction, 1e-10, 1.0)))

optimizer = tf.train.AdamOptimizer()
minimize = optimizer.minimize(cross_entropy)


mistakes = tf.not_equal(tf.argmax(target, 1), tf.argmax(prediction, 1))
error = tf.reduce_mean(tf.cast(mistakes, tf.float32))


# Setting up Tensorflow for execution

init_op = tf.initialize_all_variables()
sess = tf.Session()
print("Started session")
sess.run(init_op)


batch_size = 10
no_of_batches = int(len(train_input)/batch_size)
epoch = 500

print("Going in to the for loop")
for i in range(epoch):
    print("epoch: " + str(epoch))
    ptr = 0
    for j in range(no_of_batches):
        print("in minimize loop -> " + str(no_of_batches))
        inp, out = train_input[ptr:ptr+batch_size], train_output[ptr:ptr+batch_size]
        ptr += batch_size
        sess.run(minimize,{data: inp, target: out})
    print "Epoch - ", str(i)
incorrect = sess.run(error, {data: test_input, target: test_output})
print sess.run(prediction,{data: [[[1],[0],[0],[1],[1],[0],[1],[1],[1],[0],[1],[0],[0],[1],[1],[0],[1],[1],[1],[0]]]})
print('Epoch {:2d} error {:3.1f}%'.format(i + 1, 100 * incorrect))
sess.close()



print("Checking the labels")

