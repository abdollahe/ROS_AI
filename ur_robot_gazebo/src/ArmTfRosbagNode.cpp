//
// Created by abdollah on 8/04/19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "rosbag/bag.h"
#include "../include/StampedJointVelocity.h"
#include "std_msgs/Bool.h"
#include "../include/RosBagConfig.h"





class testClass {
private :
    rosbag::Bag bag;
    ros::Subscriber changeStateSub ;
    ros::Subscriber rosBagConfigSub ;
    ros::Subscriber sub ;

    std::string topic_name = "arm_position_topic" ;
    ros::Time shiftTime ;

public:
    std::unique_ptr<ros::NodeHandle> rosNode ;


    testClass() {
        int argc = 0 ;
        char **argv = nullptr ;
        ros::init(argc , argv , "arm_tf_rosbag_node") ;
        this->rosNode.reset(new ros::NodeHandle("arm_tf_rosbag_node")) ;

        changeStateSub = rosNode->subscribe("/ur_robot/RosbagState" , 1 , &testClass::RosBagStateCallback , this) ;
        rosBagConfigSub = rosNode->subscribe("/ur_robot/RosbagConfig" , 1 , &testClass::RosBagConfigCallback , this) ;

        bag.open("/home/abdollah/Documents/ROSBAGs/armPose.bag", rosbag::bagmode::Write);
    }

    ~testClass() {

        std::cout << "Closing the bag!!!" << std::endl ;
        bag.close() ;
    }

    void RosBagStateCallback( const std_msgs::BoolPtr &msg_ ) {
        if(msg_->data) {
            sub = rosNode->subscribe("/ur_robot/arm_position" , 100 , &testClass::ArmPoseCallback , this) ;
        }
        else {
            sub.shutdown() ;
        }
    }

    void ArmPoseCallback(const geometry_msgs::PoseStampedPtr &msg_) {

        std::cout << "data coming in for bin pose" << std::endl ;

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



//void ArmPoseCallback(const geometry_msgs::PoseStampedPtr &msg_) ;
//void RosBagStateCallback( const std_msgs::BoolPtr &msg_ ) ;


//rosbag::Bag bag;
//ros::Subscriber sub ;
//ros::NodeHandle node ;
int main(int argv , char ** argc) {
//    ros::init(argv , argc , "arm_position_ros_bag");


//    ros::Subscriber changeStateSub = node.subscribe("/ur_robot/RosbagState" , 1 , RosBagStateCallback ) ;
//
//
//    bag.open("/home/abdollah/Documents/ROSBAGs/armPose.bag", rosbag::bagmode::Write);




    testClass myClass ;



    ros::spin() ;


    //bag.close() ;
    return 0 ;

}



//void ArmPoseCallback(const geometry_msgs::PoseStampedPtr &msg_) {
//
//    std::cout << "data coming in for bin pose" << std::endl ;
//
//    bag.write(msg_->header.frame_id.data() , ros::Time::now() , msg_) ;
//    //bag.write("joint_names" , ros::Time::now() , msg_->jointName) ;
//
//    std::cout << "Data saved in bag for bin pose" << std::endl ;
//
//
//}
//
//
//void RosBagStateCallback(const std_msgs::BoolPtr &msg_) {
//    if(msg_->data) {
//        sub = node.subscribe("/ur_robot/arm_position" , 100 , ArmPoseCallback) ;
//    }
//    else {
//        sub.shutdown() ;
//    }
//
//}



