//
// Created by abdollah on 28/06/19.
//

//
// Created by abdollah on 9/04/19.
//

#include "MainController.h"
#include <thread>
#include <RosBagConfig.h>
#include "JointStateSimple.h"
#include "std_srvs/Empty.h"
#include "../include/PoseMessageSimple.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdlib>




MainController::MainController() {

//    state = 0 ;
    this->iteration = 2 ;




    int argc = 0 ;
    char **argv = nullptr ;
    ros::init(argc , argv , "main_controller_node") ;
    this->rosNode.reset(new ros::NodeHandle("main_controller_node_handle")) ;


    this->timer = this->rosNode->createTimer(ros::Duration(5.0) , &MainController::TimerCallback , this) ;
    //this->endTimer = this->rosNode->createTimer(ros::Duration(3.0) , &MainController::EndTimeCallback , this) ;

    this->armJointPub = this->rosNode->advertise<ur_robot_gazebo::JointStateSimple>("/arm_joint_state" , 1) ;

    this->pickPlaceFinishSub = this->rosNode->subscribe("/ur_robot/pick_place_finish" , 1 ,
                                                        &MainController::EventCallback) ;

    this->armJointConfigDonSub = this->rosNode->subscribe("/ur_robot/move_joint_done" , 1 , &MainController::ArmJointConfigDoneCallback , this) ;

    this->checkTargetDeletedSub = this->rosNode->subscribe("/ur_robot/delete_cube_done" , 1 , &MainController::TargetObjectDeleteCheckCallback , this ) ;

    this->clockSubsciber = this->rosNode->subscribe("/clock" , 1 , &MainController::ClockCallback) ;

    this->targetPosePub = this->rosNode->advertise<ur_robot_gazebo::PoseMessageSimple>("/ur_robot/target_obj_init_pose" , 1) ;

    this->startGraspingPub = this->rosNode->advertise<ur_robot_gazebo::PoseMessageSimple>("/ur_robot/arm_start_grasp" , 1) ;

    this->rosbagNamePub = this->rosNode->advertise<std_msgs::String>("/ur_robot/rosbag_name" , 1) ;

    this->rosConfigPub = this->rosNode->advertise<ur_robot_gazebo::RosBagConfig>("/ur_robot/RosbagConfig" , 1) ;
    this->rosStatePub = this->rosNode->advertise<std_msgs::Bool>("/ur_robot/RosbagState" , 1) ;

    this->deleteTargetPub = this->rosNode->advertise<std_msgs::Bool>("/ur_robot/delete_target_object" , 1) ;

    this->gazeboSimPause = this->rosNode->serviceClient<std_srvs::Empty>("/gazebo/pause_physics") ;
    this->gazeboSimUnPause = this->rosNode->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics") ;

    //this->TakeArmToJointState(this->joint_wait , 6) ;


}

MainController::~MainController() {

}



float MainController::GenerateRandomValue(float lowRange, float highRange) {
    return ((highRange - lowRange) * ((float)rand() / RAND_MAX)) + lowRange;
}

void MainController::TargetObjectDeleteCheckCallback(std_msgs::Bool data) {
    std::cout << "Delete Target Object callback has been called" << std::endl ;
    MainController::state = MainController::State14 ;
}


void MainController::DeleteTargetObject() {
    std_msgs::Bool msg ;
    msg.data = true ;


    ros::Rate loop_rate(10) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(deleteTargetPub.getNumSubscribers() > 0 ) {
            this->deleteTargetPub.publish(msg) ;
            std::cout << "Target Object delete request sent out" << std::endl ;
            ctrl_c = true ;
        }
        else
            loop_rate.sleep() ;
    }


}

void MainController::EventCallback(std_msgs::Int32 data) {
    std::cout << "Event callback has been called" << std::endl ;
    MainController::state = MainController::State11 ;
    //SetState(data.data) ;
}

void MainController::TimerCallback(const ros::TimerEvent &) {
    std::cout << "Timer timed out!!!!" << std::endl;
    MainController::isExecuting = false ;
}

void EndTimeCallback(const boost::system::error_code& /*e*/) {
    std::cout<< "going to the first state of the state machine" << std::endl ;
    MainController::state = MainController::State2 ;
}

void MainController::SetState(int stateNum) {
    state = static_cast<MainController::States>(stateNum);
}

void MainController::TakeArmToJointState(float *joints , int dataLength) {


    ur_robot_gazebo::JointStateSimple jointState ;

    for(int i = 0 ; i < dataLength ; i++)
        jointState.joint_angles.push_back(*(joints + i));


    armJointPub.publish(jointState) ;
    std::cout << "Joint state sent out to robotic arm 2-> " << std::endl ;


}

void MainController::SendBagFileName(std::string name) {
    std_msgs::String msg ;
    msg.data = name ;

    std::cout << "Latest time for sending is: " << latestTime << endl ;

    ros::Rate loop_rate(10) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(rosbagNamePub.getNumSubscribers() > 0 ) {
            this->rosbagNamePub.publish(msg) ;
            ctrl_c = true ;
        }
        else
            loop_rate.sleep() ;
    }

}



void MainController::SendRosBagConfig(std::string str) {
    ur_robot_gazebo::RosBagConfig msg ;
    msg.topic_name = str ;
    msg.time.data = latestTime ;

    std::cout << "Latest time for sending is: " << latestTime << endl ;

    ros::Rate loop_rate(10) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(rosConfigPub.getNumSubscribers() > 0 ) {
            this->rosConfigPub.publish(msg) ;
            ctrl_c = true ;
        }
        else
            loop_rate.sleep() ;
    }


}


void MainController::ChangeRosBagState(bool enable) {
    std_msgs::Bool msg ;
    msg.data = enable ;

    ros::Rate loop_rate(10) ;
    bool ctrl_c = false ;
    while(!ctrl_c) {
        if(rosStatePub.getNumSubscribers() > 0 ) {
            this->rosStatePub.publish(msg) ;
            ctrl_c = true ;
            //std::cout << "Request for object spawn sent out" << std::endl ;
        }
        else
            loop_rate.sleep() ;
    }
}


void MainController::ArmJointConfigDoneCallback(std_msgs::Bool data) {
    std::cout << "Checking the value -> " << MainController::enableArmJointConfigDoneSub << std::endl ;
    if(MainController::enableArmJointConfigDoneSub) {
        std::cout << "Recevied back " << std::endl ;
        MainController::state = MainController::State6 ;
        MainController::enableArmJointConfigDoneSub = false ;
        this->ToggleTimerState(false) ;
        MainController::isExecuting = false ;
    }


}


bool MainController::ChangeSimulationState(bool state) {

    bool st = false ;

    std_srvs::Empty empty ;
    if(state) {
        //std::cout << "Time when pausing is: " << latestTime << std::endl ;
        st = this->gazeboSimPause.call(empty) ;
        if(!st)
            std::cout << "The process of pausing the simulation was unsuccessful" << std::endl ;
    }
    else {
        //std::cout << "Time when un - pausing is: " << latestTime << std::endl ;
        st = this->gazeboSimUnPause.call(empty) ;
        std::cout << "The process of un-pausing the simulation was unsuccessful" << std::endl ;
    }

    return st ;
}



float* MainController::SetUpTargetPosition() {

    // Initial value for the direction params are 1 and for
    // the iteration params are 0
    // Remember the params in this context are save in the below format on disk
    // for config save [x_direction , x_targetIteration , y_direction , y_targetIteration]


//    if(x_targetIteration == 2 ) {
//        x_targetIteration = 0 ;
//        y_targetIteration++ ;
//
//        if(y_targetIteration == 2) {
//
//            y_targetIteration = 0 ;
//
//            if( (x_direction == 1) && (y_direction == 1))
//                y_direction = -1 ;
//            else if ((x_direction == 1) && (y_direction == -1)) {
//                y_direction = 1 ;
//                x_direction = -1 ;
//            }
//            else if ((x_direction == -1) && (y_direction == 1)) {
//                y_direction = -1 ;
//            }
//            else if ((x_direction == -1) && (y_direction == -1)){
//                x_direction = 1 ;
//                y_direction = 1 ;
//                this->reachedEnd = true ;
//            }
//
//        }
//    }


//    float xShift = x_direction * x_targetIteration * targetPosePermutation ;
//    float yShift = y_direction * y_targetIteration * targetPosePermutation ;



//    static float newTargetPos[3] = {0.0 , 0.0 , 0.0};
//
//    newTargetPos[0] = this->targetPose[0] + xShift ;
//    newTargetPos[1] = this->targetPose[1] + yShift ;
//    newTargetPos[2] = this->targetPose[2] ;
//
//    x_targetIteration++ ;

    static float newTargetPos[3] = {0.0 , 0.0 , 0.0};

    float x_value = GenerateRandomValue(-0.75f , 0.75f) ;

    std::cout << "The x value of the cube is: " << x_value << std::endl ;

    float y_value = GenerateRandomValue(-0.25f , 0.25f) ;

    std::cout << "The y value of the cube is: " << y_value << std::endl ;


    newTargetPos[0] = x_value ;
    newTargetPos[1] = y_value ;
    newTargetPos[2] = this->targetPose[2] ;

    return newTargetPos ;

}


void MainController::StartObjectGrasping() {

    ur_robot_gazebo::PoseMessageSimple pose ;

    pose.position.push_back(*(this->target_position)) ;
    pose.position.push_back(*(this->target_position + 1)) ;
    pose.position.push_back(*(this->target_position + 2)) ;

    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;

    ros::Rate loop_rate(1) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(startGraspingPub.getNumSubscribers() > 0 ) {
            this->startGraspingPub.publish(pose) ;
            ctrl_c = true ;
            std::cout << "Command sent to start the grasping" << std::endl ;
        }
        else
            loop_rate.sleep() ;
    }


}

void MainController::SpawnObject(int type, float *position) {

    this->target_position = position ;

    ur_robot_gazebo::PoseMessageSimple pose ;

    pose.position.push_back(*position) ;
    pose.position.push_back(*(position + 1)) ;
    pose.position.push_back(*(position + 2)) ;

    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;
    pose.quaternion.push_back(0) ;


    ros::Rate loop_rate(1) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(targetPosePub.getNumSubscribers() > 0 ) {
            this->targetPosePub.publish(pose) ;
            ctrl_c = true ;
            std::cout << "Request for object spawn sent out" << std::endl ;
        }
        else
            loop_rate.sleep() ;
    }

}


float* MainController::SetUpInitJointState() {

    // The initial values of the direction params are 1 and the
    // initial value of the iteration values are 0 and for saving them
    // in the context file the structure is as going :
    // [direction_j1 , iteration_j1 , direction_j2 , iteration_j2 , direction_j3 , iteration_j3]



    if(iteration_j3 == 2 ) {
        iteration_j3 = 0 ;
        iteration_j2++ ;

        if(iteration_j2 == 2) {
            iteration_j2 = 0 ;
            iteration_j1++ ;
            if(iteration_j1 == 2) {
                iteration_j1 = 0 ;

                if( (direction_j1 == 1) && (direction_j2 == 1) && (direction_j3 == 1) ) {   // 1 1 1
                    direction_j3 = -1 ;
                }
                else if( (direction_j1 == 1) && (direction_j2 == 1) && (direction_j3 == -1) ) { // 1 1 -1
                    direction_j2 = -1 ;
                    direction_j3 = 1 ;
                }
                else if( (direction_j1 == 1) && (direction_j2 == -1) && (direction_j3 == 1) ) { //1 -1 1
                    direction_j3 = -1 ;
                }
                else if( (direction_j1 == 1) && (direction_j2 == -1) && (direction_j3 == -1) ) { // 1 -1 -1
                    direction_j1 = -1 ;
                    direction_j2 = 1  ;
                    direction_j3 = 1  ;
                }
                else if( (direction_j1 == -1) && (direction_j2 == 1) && (direction_j3 == 1) ) { //-1 1 1
                    direction_j3 = -1 ;
                }
                else if( (direction_j1 == -1) && (direction_j2 == 1) && (direction_j3 == -1) ) {//-1 1 -1
                    direction_j2 = -1 ;
                    direction_j3 = 1 ;

                }
                else if( (direction_j1 == -1) && (direction_j2 == -1) && (direction_j3 == 1) ) { //-1 -1 1
                    direction_j3 = -1 ;

                }
                else if( (direction_j1 == -1) && (direction_j2 == -1) && (direction_j3 == -1) ) {//-1 -1 -1
                    direction_j1 = direction_j2 = direction_j3 = 1 ;
                    this->jointReachedEnd = true ;
                }
            }

        }
    }

    float j1Shift = direction_j1 * iteration_j1 * jointPermutation ;
    float j2Shift = direction_j2 * iteration_j2 * jointPermutation ;
    float j3Shift = direction_j3 * iteration_j3 * jointPermutation ;

    static float newJointStatePos[6] = {0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0};

    newJointStatePos[0] = this->joint_wait[0] + j1Shift ;
    newJointStatePos[1] = this->joint_wait[1] + j2Shift ;
    newJointStatePos[2] = this->joint_wait[2] + j3Shift ;
    newJointStatePos[3] = this->joint_wait[3] ;
    newJointStatePos[4] = this->joint_wait[4] ;
    newJointStatePos[5] = this->joint_wait[5] ;


    iteration_j3++ ;

    return newJointStatePos ;
}

void MainController::ClockCallback(rosgraph_msgs::Clock clock) {

    MainController::latestTime =  clock.clock.now() ;

}


void MainController::ToggleTimerState(bool state) {
    if(state)
        this->timer.start() ;
    else
        this->timer.stop() ;
}

void MainController::ToggleEndTimerState(bool state) {
    if(state) {
        this->endTimer.start() ;
    }
    else {
        this->endTimer.stop() ;
    }
}

void MainController::ChangeTargetObjectPose(const int *data) {

    x_direction = *data ;
    x_targetIteration  = *(data + 1) ;
    y_direction  = *(data + 2) ;
    y_targetIteration = *(data + 3) ;

}

void MainController::ChangeArmInitState(const int *data) {

    direction_j1 = *data    ;
    iteration_j1 = *(data + 1) ;
    direction_j2 = *(data + 2) ;
    iteration_j2 = *(data + 3) ;
    direction_j3 = *(data + 4) ;
    iteration_j3 = *(data + 5) ;

}

void MainController::ChangeLastTopicIndex(int index) {

}

void MainController::SaveConfigData(int index) {
    int targetData[4] = {x_direction , x_targetIteration , y_direction , y_targetIteration};
    int armData[6] = {direction_j1 , iteration_j1 , direction_j2 , iteration_j2 , direction_j3 , iteration_j3};
    configClass.writeConfigFile(targetData , armData , index) ;
}


MainController::States MainController::state = MainController::State1 ;
bool MainController::enableArmJointConfigDoneSub = true ;

ros::Time MainController::latestTime  ;

bool MainController::isRunning = true ;

bool MainController::isExecuting = false ;

bool MainController::targetExists = false ;

int main() {

    MainController mainController ;

    ros::Rate loop_rate(2) ;

    mainController.configClass.readConfigFile() ;

    int j = mainController.configClass.getLastTopicIndex() + 1 ;
//   int j = 0 ;

    mainController.SendBagFileName(std::to_string(j)) ;

    for(int i = 0 ; i < mainController.iteration ; i++) {

        std::cout << "Performing Stage: " << i << std::endl ;
        MainController::state = MainController::State1 ;

        while ( ros::ok() && (MainController::state != MainController::endState)) {

            float* targetPose ;
            float* armInitState ;

            switch (MainController::state) {


                case MainController::State0:
                {
                    std::cout << "In State 1" << std::endl ;

                    mainController.ChangeTargetObjectPose(mainController.configClass.getTargetPos()) ;
                    mainController.ChangeArmInitState(mainController.configClass.getJointStatePos()) ;
                    MainController::state = MainController::State1 ;
                }
                    break ;

                    ///Change the target position
                case MainController::State1 :
                {

                    std::cout << "In State 1" << std::endl ;
                    targetPose = mainController.SetUpTargetPosition() ;
                    std::cout << "The target pose to spawn is: " << *targetPose << " - " << *(targetPose + 1) << " - " << *(targetPose + 2) << std::endl ;

                    if(!mainController.reachedEnd)
                        MainController::state = MainController::State2 ;

                    else {
                        MainController::state = MainController::endState;

                        mainController.SaveConfigData(0) ;

                        mainController.reachedEnd = false ;
                    }

                }
                    break;
                    /// Set the joint states of the arm
                case MainController::State2 :
                {
                    std::cout << "In State 2" << std::endl ;

                    mainController.ToggleEndTimerState(false) ;

                    armInitState = mainController.SetUpInitJointState() ;

                    if(!mainController.jointReachedEnd)
                    {
                        MainController::state = MainController::State3 ;
                        MainController::enableArmJointConfigDoneSub = true  ;
                    }
                    else
                    {

                        mainController.jointReachedEnd = false ;
                        MainController::state = MainController::State1 ;
                    }
                }
                    break;

                    /// Spawn the target object in the desired position
                case MainController::State3 :
                {
                    std::cout << "In State 3" << std::endl ;
                    mainController.SpawnObject(0 , targetPose) ;
                    MainController::state = MainController::State4 ;
                }
                    break;

                    /// If the simulation is paused, unpause it.
                case MainController::State4:
                    std::cout << "In State 4" << std::endl ;
                    if(!MainController::isRunning) {
                        std::cout << "In state 4 - first if check" << std::endl ;
                        if(mainController.ChangeSimulationState(false)) {

                            std::cout << "In state 4 - second if check" << std::endl ;
                            MainController::state = MainController::State5 ;
                            MainController::isRunning = true ;
                        }
                    }
                    else {
                        MainController::state = MainController::State5 ;
                    }
                    break ;

                    /// Take the arm to the specified initial joint state
                case MainController::State5 :
                    std::cout << "In state 5" << std::endl ;
                    if(!MainController::isExecuting) {
                        std::cout << "In state 5 - first if check" << std::endl ;
                        mainController.TakeArmToJointState(armInitState , 6) ;
                        MainController::isExecuting = true ;
                        mainController.ToggleTimerState(true) ;
                    }
                    break;

                    ///  Pause the simulation and save the time.
                case MainController::State6 :
                    std::cout << "In state 6 " << std::endl ;
                    if(mainController.ChangeSimulationState(true)) {
                        std::cout << "In state 6 - first if check" << std::endl ;
                        MainController::state = MainController::State7 ;
                        MainController::isRunning = false ;
                    }
                    break;

                    /// Send new topic and time for ROSbags.
                case MainController::State7:
                    std::cout << "In state 7" << std::endl ;
                    mainController.SendRosBagConfig(std::to_string(j)) ;
                    mainController.ChangeRosBagState(true) ;
                    MainController::state = MainController::State8 ;

                    break ;

                    /// Unpause the simulation
                case MainController::State8:
                    std::cout << "In state 8" << std::endl ;
                    if(mainController.ChangeSimulationState(false)) {
                        std::cout << "In state 8 - first if check" << std::endl ;
                        MainController::state = MainController::State9 ;
                        MainController::isRunning = true ;
                    }
                    break ;

                    /// Start the pick and place process.
                case MainController::State9:
                    std::cout << "In state 9" << std::endl ;
                    mainController.StartObjectGrasping() ;
                    MainController::state = MainController::State10 ;
                    break ;

                    /// Wait for the grasping to finish and receive the message on finish
                case MainController::State10:
                    std::cout << "In state 10 - waiting for process to finish" << std::endl ;
                    break ;

                    /// Pause the simulation
                case MainController::State11:
                    std::cout << "In State 11" << std::endl ;
                    if(mainController.ChangeSimulationState(true)) {
                        std::cout << "In state 11 - first if check" << std::endl ;
                        MainController::state = MainController::State12 ;
                        MainController::isRunning = false ;
                    }
                    break ;
                    /// Stop the ROSbags receiving data and storing (shutdown their subscribers)
                case MainController::State12:
                    std::cout << "In state 12" << std::endl ;
                    mainController.ChangeRosBagState(false) ;
                    MainController::state = MainController::State13 ;
                    break ;

                    /// Delete the target object
                case MainController::State13: {
                    std::cout << "In state 13 - Waiting on target object to be deleted" << std::endl;
                    mainController.DeleteTargetObject();
                    MainController::state = MainController::WaitState ;

                }
                    break;

                case MainController::State14:
                {
                    std::cout << "In state14 -> Saving the system config" << std::endl ;
                    mainController.SaveConfigData(j) ;
                    MainController::state = MainController::State15 ;
                }
                    break ;
                case MainController::State15: {

                    std::cout << "One sequence done: -> Going to state 2" << std::endl;
                    j++;
                    //MainController::state = MainController::State2 ;
                    boost::asio::io_service io;
                    boost::asio::deadline_timer t(io, boost::posix_time::seconds(5));
                    t.async_wait(&EndTimeCallback);
                    io.run();
                }
                    break ;
                default :
//                   std::cout << "Error in the state machine" << std::endl ;
                    break;
            }

            ros::spinOnce();

            if(MainController::isRunning)
                loop_rate.sleep();

        }

        std::cout << "---------------------------------------"<< std::endl ;
        std::cout << "---------------------------------------"<< std::endl ;
        std::cout << "---------------------------------------"<< std::endl ;
    }



//    ros::spin() ;

    return 0 ;
}