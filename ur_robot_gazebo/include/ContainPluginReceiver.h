//
// Created by abdollah on 7/04/19.
//

#ifndef SRC_CONTAINPLUGINRECEIVER_H
#define SRC_CONTAINPLUGINRECEIVER_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


namespace gazebo {
    class GAZEBO_VISIBLE ContainPluginReceiver : public WorldPlugin {
    private:
        /// \brief A node used for transport (ROS)
     std::unique_ptr<ros::NodeHandle> rosNode ;

        /// \brief A subscriber(ROS).
     ros::Subscriber rosSub ;

        /// \brief A callback queue that helps process messages (ROS)
     ros::CallbackQueue rosQueue ;

        /// \brief A thread that keeps running the rosQueue
     std::thread rosQueueThread ;

        /// ROS related methods
     void QueueThread() ;

    ros::Publisher pub_;

     transport::NodePtr gzNode;
     transport::SubscriberPtr containSub;

    public:
        ContainPluginReceiver() ;
        ~ContainPluginReceiver();

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override ;
    void onContainMsg(ConstIntPtr &_msg) ;

    };
}



#endif //SRC_CONTAINPLUGINRECEIVER_H
