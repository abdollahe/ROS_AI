//
// Created by abdollah on 7/04/19.
//

#include "../include/JointInfoListenerClass.h"


JointInfoListenerClass::JointInfoListenerClass() {

}

JointInfoListenerClass::~JointInfoListenerClass() {


}


void JointInfoListenerClass::JointInfoCallback(const sensor_msgs::JointStatePtr& msg_) {

    this->jointPosition.header = msg_->header ;
    this->jointPosition.position = msg_->position ;
    this->jointPosition.jointName = msg_->name ;

    this->jointVelocity.header = msg_->header ;
    this->jointVelocity.velocity = msg_->velocity ;
    this->jointVelocity.jointName = msg_->name ;

    this->PublishJointInfo() ;


}


void JointInfoListenerClass::SetPublishers(ros::NodeHandle nh_) {

     this->posPub = nh_.advertise<ur_robot_gazebo::StampedJointPosition>(this->jointPositionTopic, 1);

     this->velPub = nh_.advertise<ur_robot_gazebo::StampedJointVelocity>(this->jointVelocityTopic, 1) ;

}

void JointInfoListenerClass::SetSubscriber(ros::NodeHandle nh_) {

    this->sub = nh_.subscribe(this->jointStateTopic, 1, &JointInfoListenerClass::JointInfoCallback , this);
}

void JointInfoListenerClass::PublishJointInfo() {

    this->posPub.publish(this->jointPosition) ;
    this->velPub.publish(this->jointVelocity) ;
}

