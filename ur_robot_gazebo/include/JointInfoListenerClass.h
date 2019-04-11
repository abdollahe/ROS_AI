//
// Created by abdollah on 7/04/19.
//


#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "iostream"
#include "ur_robot_gazebo/StampedJointPosition.h"
#include "ur_robot_gazebo/StampedJointVelocity.h"

#ifndef SRC_JOINTINFOLISTENERCLASS_H
#define SRC_JOINTINFOLISTENERCLASS_H


class JointInfoListenerClass {
private:
    std::string jointPositionTopic = "/ur/joint_position" ;
    std::string jointVelocityTopic = "/ur/joint_velocity" ;
    std::string jointStateTopic = "/joint_states" ;
    ros::Subscriber sub ;
    ros::Publisher posPub ;
    ros::Publisher velPub ;
    ur_robot_gazebo::StampedJointPosition jointPosition ;
    ur_robot_gazebo::StampedJointVelocity jointVelocity ;

public:
    JointInfoListenerClass() ;
    ~JointInfoListenerClass() ;

    void JointInfoCallback(const sensor_msgs::JointStatePtr& msg_);
    void PublishJointInfo() ;

    void SetSubscriber(ros::NodeHandle nh_) ;
    void SetPublishers(ros::NodeHandle nh_) ;

};


#endif //SRC_JOINTINFOLISTENERCLASS_H

