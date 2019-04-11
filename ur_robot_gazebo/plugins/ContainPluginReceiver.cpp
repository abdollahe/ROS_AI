//
// Created by abdollah on 7/04/19.
//

#include "../include/ContainPluginReceiver.h"
//--------------------------
// Gazebo related header files
#include <mutex>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/msgs/color.pb.h>

//-------------------------
// ROS related header files
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

namespace gazebo {


    ContainPluginReceiver::ContainPluginReceiver() {

    }

    ContainPluginReceiver::~ContainPluginReceiver() {

    }

    void ContainPluginReceiver::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {


        gzmsg << "Loading Example plugin\n";
        // Transport initialization
        this->gzNode = transport::NodePtr(new transport::Node());
        this->gzNode->Init();

        // Subscribe to ContainPlugin output
        std::string topic("/gazebo/myworld/contain_example/contain");
//        std::string topic("~/contain_example/contain");
        this->containSub = this->gzNode->Subscribe(
                topic, &ContainPluginReceiver::onContainMsg, this);
        if (!containSub)
        {
            gzerr << "Failed to subscribe to [" << topic << "]\n";
        }
        else {
            gzmsg << "Subscribed successfully" << std::endl ;
        }



        if (!ros::isInitialized()) {
            int argc = 0 ;
            char **argv = nullptr ;

            ros::init(argc , argv , "gazebo_Contain_client" , ros::init_options::NoSigintHandler) ;

        }

        // Create the ROS node. This is similar to the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_contain_plugin_node")) ;

        // Create a named topic and subscribe to it for changing color of the scenery
//        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>("/" + this->modelName +  "/" + this->topicName + "/ObjectColorChange" ,
//                                                                                              1 , boost::bind(&ContainPluginReceiver::onContainMsg , this, _1) , ros::VoidPtr() , &this->rosQueue ) ;
//        this->rosSub = this->rosNode->subscribe(so) ;


        this-> pub_ = this->rosNode->advertise<std_msgs::Bool>("/ur_robot_world/contain", 1);
        //spin up the queue helper thread
        this->rosQueueThread = std::thread(std::bind(&ContainPluginReceiver::QueueThread,  this)) ;
    }

    void ContainPluginReceiver::onContainMsg(ConstIntPtr &_msg) {

        std::cout << "Contain update coming in!!" << std::endl ;

        if(_msg->data() == 1) {
             gzmsg << "Contained!!!!!!" << std::endl ;
             std_msgs::Bool msg_ ;
             msg_.data = true ;
             this->pub_.publish(msg_) ;
         }
        else {
            std_msgs::Bool msg_ ;
            msg_.data = false ;
            this->pub_.publish(msg_) ;
        }

    }




    void ContainPluginReceiver::QueueThread() {
        static const double timeout = 0.01 ;
        while(this->rosNode->ok()) {
            this->rosQueue.callAvailable(ros::WallDuration(timeout)) ;
        }
    }



    GZ_REGISTER_WORLD_PLUGIN(ContainPluginReceiver);

}