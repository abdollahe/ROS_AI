//
// Created by abdollah on 8/04/19.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "rosbag/bag.h"
#include "std_msgs/Bool.h"
#include "../include/RosBagConfig.h"




class testClass {
private :
    rosbag::Bag bag;
    ros::Subscriber changeStateSub ;
    ros::Subscriber rosBagConfigSub ;
    ros::Subscriber rosBagNameSub ;
    ros::Subscriber sub ;

    std::string topic_name = "target_object_pose" ;
    std::string topic_name_save = "" ;
    ros::Time shiftTime ;

    std::string file_name = "/home/abdollah/Documents/ROSBAGs/targetPose" ;
    std::string extension = ".bag" ;

public:
    std::unique_ptr<ros::NodeHandle> rosNode ;


    testClass() {
        int argc = 0 ;
        char **argv = nullptr ;
        ros::init(argc , argv , "target_pose_ros_bag") ;
        this->rosNode.reset(new ros::NodeHandle("target_pose_ros_bag")) ;

        changeStateSub = rosNode->subscribe("/ur_robot/RosbagState" , 1 , &testClass::RosBagStateCallback , this) ;
        rosBagConfigSub = rosNode->subscribe("/ur_robot/RosbagConfig" , 1 , &testClass::RosBagConfigCallback , this) ;
        rosBagNameSub = rosNode->subscribe("/ur_robot/rosbag_name" , 1 , &testClass::RosBagNameCallback , this) ;

//        bag.open("/home/abdollah/Documents/ROSBAGs/targetPose.bag", rosbag::bagmode::Write);

    }

    ~testClass() {

        std::cout << "Closing the bag!!!" << std::endl ;
        bag.close() ;
    }

    void RosBagNameCallback(const std_msgs::StringPtr &msg_) {
        bag.open(file_name + "_" + msg_->data + extension, rosbag::bagmode::Write);
    }

    void RosBagStateCallback( const std_msgs::BoolPtr &msg_ ) {
        if(msg_->data) {
            sub = rosNode->subscribe("/objectPosePlugin/ObjectPose" , 100 , &testClass::TargetPoseCallback, this) ;
        }
        else {
            sub.shutdown() ;
        }
    }

    void TargetPoseCallback(const geometry_msgs::PoseStampedPtr& msg_) {

        ros::Duration duration ;

        ros::Time rosTime1 = ros::Time::now() ;


        duration = rosTime1 - shiftTime ;

        ros::Time timeToSave ;
        timeToSave.nsec = duration.nsec ;
        timeToSave.sec = duration.sec ;


        std::cout << "shift time is: " << shiftTime <<std::endl ;
        std::cout << "time to save is : " << timeToSave << std::endl ;

        msg_->header.stamp.sec = timeToSave.sec ;
        msg_->header.stamp.nsec = timeToSave.nsec ;
        bag.write(topic_name_save , timeToSave , msg_) ;


    }

    void RosBagConfigCallback(const ur_robot_gazebo::RosBagConfigPtr &msg_) {
        shiftTime = msg_->time.data ;
        std::cout << "In Config" << std::endl ;
        topic_name_save = topic_name + "_" + msg_->topic_name ;
    }

};


//void TargetPoseCallback(const geometry_msgs::PoseStampedPtr& msg_) ;


//rosbag::Bag bag;
int main(int argv , char ** argc) {
  //ros::init(argv , argc , "target_pose_ros_bag");
  //ros::NodeHandle node ;

//    bag.open("/home/abdollah/Documents/ROSBAGs/targetPose.bag", rosbag::bagmode::Write);

  //ros::Subscriber sub = node.subscribe("/objectPosePlugin/ObjectPose" , 100 , TargetPoseCallback) ;
  testClass myClass ;
  ros::spin() ;



    //bag.close() ;
  return 0 ;

}



//void TargetPoseCallback(const geometry_msgs::PoseStampedPtr& msg_) {
//
//    std::cout << "data coming in- target pose" << std::endl ;
//
//
//
//    bag.write("target_object_pose" , ros::Time::now() , msg_) ;
//
//    std::cout << "Data saved in targetPose bag" << std::endl ;
//
//
//}



