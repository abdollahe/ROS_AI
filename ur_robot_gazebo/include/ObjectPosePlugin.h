//
// Created by abdollah on 5/04/19.
//

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

#ifndef GAZEBO_PLUGIN_TUTORIAL_OBJECTPOSEPLUGIN_H
#define GAZEBO_PLUGIN_TUTORIAL_OBJECTPOSEPLUGIN_H

namespace gazebo {

    class GAZEBO_VISIBLE ObjectPosePlugin : public ModelPlugin {
    private :
        /// \brief pointer to ros node
        ros::NodeHandle* rosnode_;
        ros::Publisher pub_;
        PubQueue<geometry_msgs::PoseStamped>::Ptr pub_Queue;

        /// \brief ros message
        geometry_msgs::PoseStamped pose_msg_;

        /// \brief topic name
        std::string topic_name_;

        /// \brief mutex to lock access to fields used in message callbacks
        boost::mutex lock;

        /// \brief save last_time
        common::Time last_time_;

        // rate control
        double update_rate_;

        /// \brief for setting ROS name space
        std::string robot_namespace_;

        ros::CallbackQueue p3d_queue_;
        boost::thread callback_queue_thread_;

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;

        unsigned int seed;

         physics::WorldPtr world_;
         physics::ModelPtr model_;

        // ros publish multi queue, prevents publish() blocking
        PubMultiQueue pmq;

        std::string frame_name_;
        std::string tf_frame_name_;

    public:
        ObjectPosePlugin() ;
        ~ObjectPosePlugin() override ;

        /// Gazebo related methods
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override ;
        void OnUpdate() ;

    private :
        /// Gazebo related methods
        void P3DQueueThread();

    };
}



#endif //GAZEBO_PLUGIN_TUTORIAL_OBJECTPOSEPLUGIN_H
