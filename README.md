The main file that launches the simulation is the test.launch file in the ur_robot_gazebo package.

The main executables that will perform the simulation and data generation and gathering are the following ones:

- In the ur_robot_gazebo package there is a src folder. In that folder MainController.cpp file and its corresponding header file in the include folder are the heart of the simulation that act as the main controller of the simulation. A state machine is implemented in this file to perfomr the control.

- The rest of the files in the src folder are executables that gather and log the various datas that we require as bag files.

- In the script folder of the same package the script named "pick_and_place.py" is responsible for implementing and performing the pick and place operation of the robotic manipulator.

- The script object spawner is responsible to spawn the target cube and delete it from the scene of Gazebo when required.

The above files are used for performing the simulation and the following files are afterwards used to clean the data generated and convert the data saved as bag files to HDF folrmat for easier usage in training the neural network.

- BAG_TO_HDF.py is the file used to convert bag files to hdf format.

- Setup training data is used to transform the data gathered to a usable struture for the nueral network.
