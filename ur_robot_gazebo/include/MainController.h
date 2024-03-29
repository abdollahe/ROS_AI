//
// Created by abdollah on 9/04/19.
//

#ifndef SRC_MAINCONTROLLER_H
#define SRC_MAINCONTROLLER_H

#include "ros/ros.h"
#include "iostream"
#include <thread>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "rosgraph_msgs/Clock.h"
#include "../include/ConfigFileSetup.h"
#include <cstdlib>


using namespace std ;

class MainController {

public:

    //-----------------------------------------------------------------
    //----------------------- Public methods --------------------------

    /// Constructor
    MainController() ;
    /// Desctructor
    ~MainController() ;

    void TakeArmToJointState(float joints[] , int dataLength) ;

    /// @param state -> if true then it pauses the simulation env, if
    /// false then it un pauses the simulation env.
    bool ChangeSimulationState(bool state) ;

    static void EventCallback(std_msgs::Int32 data) ;

    static void SetState(int stateNum) ;

    void ArmJointConfigDoneCallback(std_msgs::Bool data) ;

    static void ClockCallback(rosgraph_msgs::Clock clock ) ;

    void TargetObjectDeleteCheckCallback(std_msgs::Bool data) ;

    float* SetUpTargetPosition() ;

    float* SetUpInitJointState() ;

    void SpawnObject(int type , float position[]) ;

    void SendRosBagConfig(std::string str) ;

    void ChangeRosBagState(bool enable) ;

    void StartObjectGrasping() ;

    void ToggleTimerState(bool state) ;

    void ToggleEndTimerState(bool state) ;

    void DeleteTargetObject() ;

    void ChangeTargetObjectPose(const int * data) ;

    void ChangeArmInitState(const int * data) ;

    void SaveConfigData(int index) ;

    void ChangeLastTopicIndex(int index) ;

    void SendBagFileName(std::string name) ;

    float GenerateRandomValue(float lowRange , float highRange) ;

    //-----------------------------------------------------------------
    //----------------------- Public Properties -----------------------

    /// Possible states of the system
    enum States {State0 , State1 , State2 , State3 , State4 , State5 , State6 , State7 ,
        State8, State9 , State10, State11, State12, State13, State14 , WaitState , State15 , endState};

    /// Variable holding the current state of the system
    static States state ;

    /// Number holding the simulation iteration number
    static uint sim_iteration ;

    /// Maximum number of simulation iterations to run for one set of parameter changes
    static const uint max_sim_iteration = 1000 ;

    /// Unique pointer to the ros node handle
    std::unique_ptr<ros::NodeHandle> rosNode ;

    /// ConfigClass to read and write config file
    ConfigClass configClass ;

    uint32_t iteration = 0 ;

    bool jointReachedEnd = false ;

    bool reachedEnd = false ;
    static bool isRunning ;
    static bool isExecuting ;
    static bool targetExists ;
    static bool enableArmJointConfigDoneSub  ;


private:

    //-----------------------------------------------------------------
    //----------------------- Private Properties ----------------------

    ros::Timer timer ;

    ros::Timer endTimer ;

    /// Sends joint states to robot for moving to a joint state
    ros::Publisher armJointPub ;

    /// A ROS publisher to send the position of the target object
    ros::Publisher targetPosePub ;

    /// A ROS publisher to send the ROS bag configurations
    ros::Publisher rosConfigPub ;

    /// A ROS publisher to enable or disable the ROS bags (Start logging or stop logging)
    ros::Publisher rosStatePub ;

    /// A ROS publisher to enable the magnetic gripper or to disable it
    ros::Publisher startGraspingPub ;

    /// A ROS publisher to request Gazebo to delete the target object
    ros::Publisher deleteTargetPub ;

    /// A ROS publisher to send the new file name to ROSBAGs
    ros::Publisher rosbagNamePub ;

    /// A ROS service to pause the Gazebo simulation
    ros::ServiceClient gazeboSimPause ;

    /// A ROS service to unpause the Gazebo simulation
    ros::ServiceClient gazeboSimUnPause ;

    ///
    float *target_position ;

    /// This value holds the initial positions of the target cube to be spawned
    float targetPose[3] = {0.5, 0 ,0.05};

    float targetPosePermutation = 0.001 ;

    int x_targetIteration = 0  ;
    int y_targetIteration = 0  ;

    int x_direction = 1 ;
    int y_direction = 1 ;


    //----------------------------------
    /// This value holds the wait config of the robotic arm
    float joint_wait [6] = {-0.5, -1.57, 1.57, -1.57, -1.57, 0} ;

    /// Step size for changing joint angles
    float jointPermutation = 0.05 ;

    int direction_j1 = 1 ;
    int direction_j2 = 1 ;
    int direction_j3 = 1 ;

    int iteration_j1 = 0 ;
    int iteration_j2 = 0 ;
    int iteration_j3 = 0 ;


    ros::Subscriber pickPlaceFinishSub ;

    ros::Subscriber armJointConfigDonSub ;

    ///Subscriber for checking if the target object has been deleted or not
    ros::Subscriber checkTargetDeletedSub ;

    ros::Subscriber clockSubsciber ;

    static ros::Time latestTime ;


    void TimerCallback(const ros::TimerEvent&);

    void EndTimeCallback(const ros::TimerEvent&) ;

};


#endif //SRC_MAINCONTROLLER_H
