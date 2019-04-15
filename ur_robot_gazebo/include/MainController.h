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


using namespace std ;

class MainController {

public:
    MainController() ;
    ~MainController() ;

    void TakeArmToJointState(float joints[] , int dataLength) ;


    /// @param state -> if true then it pauses the simulation env, if
    /// false then it un pauses the simulation env.
    bool ChangeSimulationState(bool state) ;

    static void EventCallback(std_msgs::Int32 data) ;

    static void SetState(int stateNum) ;

    void ArmJointConfigDoneCallback(std_msgs::Bool data) ;

    static void ClockCallback(rosgraph_msgs::Clock clock ) ;


    float* SetUpTargetPosition() ;

    float* SetUpInitJointState() ;


    void SpawnObject(int type , float position[]) ;


    void SendRosBagConfig(std::string str) ;

    void ChangeRosBagState(bool enable) ;

    void StartObjectGrasping() ;

    void ToggleTimerState(bool state) ;

    void ToggleEndTimerState(bool state) ;

    void DeleteTargetObject() ;


    uint32_t iteration = 0 ;

    enum States { State1 , State2 , State3 , State4 , State5 , State6 , State7 , State8, State9 , State10, State11, State12, State13, endState };

    static States state ;

    std::unique_ptr<ros::NodeHandle> rosNode ;

    bool jointReachedEnd = false ;

    bool reachedEnd = false ;
    static bool isRunning ;
    static bool isExecuting ;
    static bool targetExists ;
    static bool enableArmJointConfigDoneSub  ;

private:


    ros::Timer timer ;

    ros::Timer endTimer ;

    /// Sends joint states to robot for moving to a joint state
    ros::Publisher armJointPub ;

    ros::Publisher targetPosePub ;


    ros::Publisher rosConfigPub ;
    ros::Publisher rosStatePub ;


    ros::Publisher startGraspingPub ;

    ros::Publisher deleteTargetPub ;

    float *target_position ;





    /// This value holds the positions of the target cube to be spawned
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


    ros::ServiceClient gazeboSimPause ;

    ros::ServiceClient gazeboSimUnPause ;


    ros::Subscriber pickPlaceFinishSub ;

    ros::Subscriber armJointConfigDonSub ;




    ros::Subscriber clockSubsciber ;

    static ros::Time latestTime ;


    void TimerCallback(const ros::TimerEvent&);

    void EndTimeCallback(const ros::TimerEvent&) ;


};


#endif //SRC_MAINCONTROLLER_H
