//
// Created by abdollah on 9/04/19.
//

#include "MainController.h"
#include <thread>
#include <RosBagConfig.h>
#include "JointStateSimple.h"
#include "std_srvs/Empty.h"
#include "../include/PoseMessageSimple.h"


MainController::MainController() {

//    state = 0 ;
    this->iteration = 2 ;


    int argc = 0 ;
    char **argv = nullptr ;
    ros::init(argc , argv , "main_controller_node") ;
    this->rosNode.reset(new ros::NodeHandle("main_controller_node_handle")) ;

    this->armJointPub = this->rosNode->advertise<ur_robot_gazebo::JointStateSimple>("/arm_joint_state" , 1) ;

    this->pickPlaceFinishSub = this->rosNode->subscribe("/ur_robot/pick_place_finish" , 1 ,
                                                        &MainController::EventCallback) ;

    this->armJointConfigDonSub = this->rosNode->subscribe("/ur_robot/move_joint_done" , 1 , &MainController::ArmJointConfigDoneCallback) ;


    this->clockSubsciber = this->rosNode->subscribe("/clock" , 1 , &MainController::ClockCallback) ;

    this->targetPosePub = this->rosNode->advertise<ur_robot_gazebo::PoseMessageSimple>("/ur_robot/target_obj_init_pose" , 1) ;


    this->rosConfigPub = this->rosNode->advertise<ur_robot_gazebo::RosBagConfig>("/ur_robot/RosbagConfig" , 1) ;
    this->rosStatePub = this->rosNode->advertise<std_msgs::Bool>("/ur_robot/RosbagState" , 1) ;


    this->gazeboSimPause = this->rosNode->serviceClient<std_srvs::Empty>("/gazebo/pause_physics") ;
    this->gazeboSimUnPause = this->rosNode->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics") ;

    //this->TakeArmToJointState(this->joint_wait , 6) ;



}

MainController::~MainController() {

}

void MainController::EventCallback(std_msgs::Int32 data) {

    SetState(data.data) ;
}

void MainController::SetState(int stateNum) {
    state = static_cast<MainController::States>(stateNum);
}

void MainController::TakeArmToJointState(float *joints , int dataLength) {

    //std::cout << "I am in the function" << std::endl ;

    ur_robot_gazebo::JointStateSimple jointState ;

    for(int i = 0 ; i < dataLength ; i++)
        jointState.joint_angles.push_back(*(joints + i));

    ros::Rate loop_rate(1) ;
    bool ctrl_c = false ;

    while(!ctrl_c) {
        if(armJointPub.getNumSubscribers() > 0 ) {
            armJointPub.publish(jointState) ;
            ctrl_c = true ;
            std::cout << "Joint state sent out to robotic arm" << std::endl ;
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
            //std::cout << "Request for object spawn sent out" << std::endl ;
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


    if(MainController::enableArmJointConfigDoneSub) {
        std::cout << "Recevied back " << std::endl ;
        MainController::state = MainController::State5 ;
        MainController::enableArmJointConfigDoneSub = false ;
    }


}


bool MainController::ChangeSimulationState(bool state) {

    bool st = false ;

    std_srvs::Empty empty ;
    if(state) {
        std::cout << "Time when pausing is: " << latestTime << std::endl ;
        st = this->gazeboSimPause.call(empty) ;
    }
    else {
        std::cout << "Time when un - pausing is: " << latestTime << std::endl ;
        st = this->gazeboSimUnPause.call(empty) ;
    }

    return st ;
}



float* MainController::SetUpTargetPosition() {

    if(x_targetIteration == 2 ) {
        x_targetIteration = 0 ;
        y_targetIteration++ ;

        if(y_targetIteration == 2) {

            y_targetIteration = 0 ;

            if( (x_direction == 1) && (y_direction == 1))
                y_direction = -1 ;
            else if ((x_direction == 1) && (y_direction == -1)) {
                y_direction = 1 ;
                x_direction = -1 ;
            }
            else if ((x_direction == -1) && (y_direction == 1)) {
                y_direction = -1 ;
            }
            else if ((x_direction == -1) && (y_direction == -1)){
                x_direction = 1 ;
                y_direction = 1 ;
                this->reachedEnd = true ;
            }

        }
    }


    float xShift = x_direction * x_targetIteration * targetPosePermutation ;
    float yShift = y_direction * y_targetIteration * targetPosePermutation ;

    static float newTargetPos[3] = {0.0 , 0.0 , 0.0};

    newTargetPos[0] = this->targetPose[0] + xShift ;
    newTargetPos[1] = this->targetPose[1] + yShift ;
    newTargetPos[2] = this->targetPose[2] ;

    x_targetIteration++ ;

    return newTargetPos ;

}


void MainController::SpawnObject(int type, float *position) {
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


MainController::States MainController::state = MainController::State1 ;
bool MainController::enableArmJointConfigDoneSub = true ;

ros::Time MainController::latestTime  ;

bool MainController::isRunning = true ;

int main() {


    MainController mainController ;


    //mainController.ChangeSimulationState(true) ;

   ros::Rate loop_rate(10) ;


   for(int i = 0 ; i < mainController.iteration ; i++) {

       std::cout << "Performing Stage: " << i << std::endl ;
       MainController::state = MainController::State1 ;

       while ( ros::ok() && (MainController::state != MainController::endState)) {

           float* targetPose ;
           float* armInitState ;

           switch (MainController::state) {
               case MainController::State1 :   ///Change the target position
                   {
                       targetPose = mainController.SetUpTargetPosition() ;
                       std::cout << "The target pose to spawn is: " << *targetPose << " - " << *(targetPose + 1) << " - " << *(targetPose + 2) << std::endl ;

                       if(!mainController.reachedEnd)
                            MainController::state = MainController::State2 ;

                       else {
                           MainController::state = MainController::State5;
                           mainController.reachedEnd = false ;
                       }

                   }
                   break;
               case MainController::State2 : /// Set the joint states of the arm
                   {
                     armInitState = mainController.SetUpInitJointState() ;
//                     std::cout << "Initial state for robot is: " << *armInitState << " - " << *(armInitState + 1) << " - " << *(armInitState + 2) << " -" << *(armInitState + 3) << " - " << *(armInitState + 4) << " - " << *(armInitState + 5) << std::endl ;
                     if(!mainController.jointReachedEnd)
                     {
                         //std::cout << "Initial state for robot is: " << *armInitState << " - " << *(armInitState + 1) << " - " << *(armInitState + 2) << " -" << *(armInitState + 3) << " - " << *(armInitState + 4) << " - " << *(armInitState + 5) << std::endl ;
                        // std::cout << "Progressing to next state ..." << std::endl ;
                         MainController::state = MainController::State3 ;
                     }
                     else
                         {
                         //std::cout << "Initial state for robot is: " << *armInitState << " - " << *(armInitState + 1) << " - " << *(armInitState + 2) << " -" << *(armInitState + 3) << " - " << *(armInitState + 4) << " - " << *(armInitState + 5) << std::endl ;
                         //std::cout << "Going to state 1 ..." << std::endl ;
                         mainController.jointReachedEnd = false ;
                         MainController::state = MainController::State1 ;
                         }
                   }
                   break;
               case MainController::State3 :
                   {
                       //std::cout << "In state 3 -- going to state 2" << std::endl;
                       mainController.SpawnObject(0 , targetPose) ;
                       MainController::state = MainController::State4 ;
                   }
                   break;
               case MainController::State4 :
                   //std::cout << "In state 4" << std::endl;

                   mainController.TakeArmToJointState(armInitState , 6) ;

                   break;
               case MainController::State5 :
                  // std::cout << "In state 5" << std::endl;
                   if(mainController.ChangeSimulationState(true)) {
                       MainController::state = MainController::State6 ;
                       MainController::isRunning = false ;
                   }

                   break;
               case MainController::State6:
                   //std::cout << "In State 6" << std::endl ;
                   mainController.SendRosBagConfig(std::to_string(i)) ;
                   mainController.ChangeRosBagState(true) ;
                   MainController::state = MainController::State7 ;
//                   if(mainController.ChangeSimulationState(false)) {
//
//                       MainController::isRunning = true ;
//                   }

                   break ;
               case MainController::State7:
                  // std::cout << "In state 7" << std::endl ;
                   if(mainController.ChangeSimulationState(false)) {
                       MainController::state = MainController::State8 ;
                       MainController::isRunning = true ;
                   }
                   break ;
               case MainController::State8:
                   //std::cout << "In state 8" << std::endl ;
                   break ;
               default :
                   //std::cout << "In state None" << std::endl;
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