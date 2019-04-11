//
// Created by abdollah on 8/04/19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "rosbag/bag.h"
#include "../include/StampedJointPosition.h"
#include "std_msgs/Bool.h"
#include "../include/RosBagConfig.h"



class testClass {
private :
    rosbag::Bag bag;
    ros::Subscriber changeStateSub ;
    ros::Subscriber rosBagConfigSub ;
    ros::Subscriber sub ;

    std::string topic_name = "joint_position" ;
    ros::Time shiftTime ;

public:
    std::unique_ptr<ros::NodeHandle> rosNode ;


    testClass() {
        int argc = 0 ;
        char **argv = nullptr ;
        ros::init(argc , argv , "joint_position_ros_bag") ;
        this->rosNode.reset(new ros::NodeHandle("joint_position_ros_bag")) ;

        changeStateSub = rosNode->subscribe("/ur_robot/RosbagState" , 1 , &testClass::RosBagStateCallback , this) ;
        rosBagConfigSub = rosNode->subscribe("/ur_robot/RosbagConfig" , 1 , &testClass::RosBagConfigCallback , this) ;

        bag.open("/home/abdollah/Documents/ROSBAGs/JointPosition.bag", rosbag::bagmode::Write);
    }

    ~testClass() {

        std::cout << "Closing the bag!!!" << std::endl ;
        bag.close() ;
    }

    void RosBagStateCallback( const std_msgs::BoolPtr &msg_ ) {
        if(msg_->data) {
            sub = sub = rosNode->subscribe("/ur/joint_position" , 100 , &testClass::JointPositionPoseCallback, this) ;
        }
        else {
            sub.shutdown() ;
        }
    }

    void JointPositionPoseCallback(const ur_robot_gazebo::StampedJointPositionPtr &msg_) {

        ros::Time rosTime1 = ros::Time::now() ;
        rosTime1.sec = rosTime1.sec - shiftTime.sec ;
        rosTime1.nsec = rosTime1.nsec - shiftTime.nsec ;

        bag.write(topic_name , rosTime1 , msg_) ;

        std::cout << "Data saved in bag for bin pose" << std::endl ;


    }

    void RosBagConfigCallback(const ur_robot_gazebo::RosBagConfigPtr &msg_) {
        shiftTime = msg_->time.data ;
        topic_name = topic_name + "_" + msg_->topic_name ;
    }


};


//void JointPositionPoseCallback(const ur_robot_gazebo::StampedJointPositionPtr& msg_) ;


//rosbag::Bag bag;
int main(int argv , char ** argc) {
//    ros::init(argv , argc , "joint_position_ros_bag");
//    ros::NodeHandle node ;

    //bag.open("/home/abdollah/Documents/ROSBAGs/JointPosition.bag", rosbag::bagmode::Write);
    //ros::Subscriber sub = node.subscribe("/ur/joint_position" , 100 , JointPositionPoseCallback) ;

    testClass myClass ;

    ros::spin() ;


    //bag.close() ;
    return 0 ;

}



//void JointPositionPoseCallback(const ur_robot_gazebo::StampedJointPositionPtr &msg_) {
//
//    std::cout << "data coming in for bin pose" << std::endl ;
//
//    bag.write("joint_position" , ros::Time::now() , msg_) ;
//    //bag.write("joint_names" , ros::Time::now() , msg_->jointName) ;
//
//
//    std::cout << "Data saved in bag for bin pose" << std::endl ;
//
//
//}



