//
// Created by abdollah on 6/04/19.
//

#include "ros/ros.h"

#include "ur_robot_gazebo/StampedJointPosition.h"
#include "ur_robot_gazebo/StampedJointVelocity.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include "../include/JointInfoListenerClass.h"




int main(int argc , char** argv ) {

    JointInfoListenerClass jointInfo ;

   ros::init(argc, argv, "ur_robot_joint_state_listener");

   ros::NodeHandle n;

   jointInfo.SetSubscriber(n) ;
   jointInfo.SetPublishers(n) ;


   ros::Rate loop_rate(5);

    while (ros::ok()) {

        jointInfo.PublishJointInfo() ;

        ros::spinOnce();

        loop_rate.sleep();
    }

   //ros::spin();

   return 0 ;
}







