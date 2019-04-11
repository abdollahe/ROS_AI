//
// Created by abdollah on 5/04/19.
//

#include "ObjectPosePlugin.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


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
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>

namespace gazebo
{

    using namespace gazebo  ;

    ObjectPosePlugin::ObjectPosePlugin() : seed(0)  {
    }

    ObjectPosePlugin::~ObjectPosePlugin() {
        this->update_connection_.reset();
        // Finalize the controller
        this->rosnode_->shutdown();
        this->p3d_queue_.clear();
        this->p3d_queue_.disable();
        this->callback_queue_thread_.join();
        delete this->rosnode_;
    }


    void ObjectPosePlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        gzmsg << "Loading the object pose plugin" << std::endl ;

        this->world_ = _parent->GetWorld();
        this->model_ = _parent;

        // load parameters
        this->robot_namespace_ = "objectPosePlugin";
        if (_sdf->HasElement("robotNamespace")) {



            this->robot_namespace_ =
                    _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

        }

        std::cout << "getting a namespace: " << this->robot_namespace_ << std::endl ;

        if (!_sdf->HasElement("topicName"))
        {
            ROS_FATAL_NAMED("Object Pose Plugin", "Object Pose plugin missing <topicName>, cannot proceed");
            return;
        }
        else
            this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


        if (!_sdf->HasElement("frameName"))
        {
            ROS_DEBUG_NAMED("p3d", "p3d plugin missing <frameName>, defaults to world");
            this->frame_name_ = "world";
        }
        else
            this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();



        if (!_sdf->HasElement("updateRate"))
        {
            ROS_DEBUG_NAMED("Object Pose Plugin", "Object Pose plugin missing <updateRate>, defaults to 0.0"
                                   " (as fast as possible)");
            this->update_rate_ = 0;
        }
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>() ;

//
        //Initialize ROS if it has not been already initialized
        if (!ros::isInitialized()) {
            int argc = 0 ;
            char **argv = nullptr ;

            ros::init(argc , argv , "gazebo_object_pose_node" , ros::init_options::NoSigintHandler) ;

        }
//
        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        // publish multi queue
        this->pmq.startServiceThread();


//        try {
//            // resolve tf prefix
//            std::string prefix;
//            this->rosnode_->getParam(std::string("tf_prefix"), prefix);
//            std::cout << "The values are :" << this->frame_name_ << " and " << prefix << std::endl ;
//            this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);
//
//        }
//        catch (...) {
//            std::cout << "Exception occured!!!" << std::endl ;
//        }
//
//
//
//
        if (this->topic_name_ != "")
        {
            //this->pub_Queue = this->pmq.addPub<geometry_msgs::PoseStamped>();
            this->pub_ =
                    this->rosnode_->advertise<geometry_msgs::PoseStamped>(this->topic_name_, 1);
        }

        #if GAZEBO_MAJOR_VERSION >= 8
            this->last_time_ = this->world_->SimTime();
        #else
            this->last_time_ = this->world_->GetSimTime();
        #endif


        // start custom queue for p3d
        this->callback_queue_thread_ = boost::thread(
                boost::bind(&ObjectPosePlugin::P3DQueueThread, this));

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ObjectPosePlugin::OnUpdate, this));

    }

    void ObjectPosePlugin::OnUpdate() {
//        gzmsg << "OnUpdate called!!!!" << std::endl ;

        #if GAZEBO_MAJOR_VERSION >= 8
            common::Time cur_time = this->world_->SimTime();
        #else
            common::Time cur_time = this->world_->GetSimTime();
        #endif

        if (cur_time < this->last_time_) {
            ROS_WARN_NAMED("p3d", "Negative update time difference detected.");
            this->last_time_ = cur_time;
        }

        // rate control
        if (this->update_rate_ > 0 && (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_)) {
            //gzmsg << "In rate control" << std::endl ;
            return;
        }



        if (this->topic_name_ != "") {
            //gzmsg << "In data processing" << std::endl ;
            // copy data into pose message
            this->pose_msg_.header.frame_id = "ObjectNew";
            this->pose_msg_.header.stamp.sec = static_cast<uint32_t>(cur_time.sec);
            this->pose_msg_.header.stamp.nsec = static_cast<uint32_t>(cur_time.nsec);

            // Fill out messages
            this->pose_msg_.pose.position.x = this->model_->GetWorldPose().pos.x;
            this->pose_msg_.pose.position.y = this->model_->GetWorldPose().pos.y;
            this->pose_msg_.pose.position.z = this->model_->GetWorldPose().pos.z;

            this->pose_msg_.pose.orientation.x = this->model_->GetWorldPose().rot.x;
            this->pose_msg_.pose.orientation.y = this->model_->GetWorldPose().rot.y;
            this->pose_msg_.pose.orientation.z = this->model_->GetWorldPose().rot.z;
            this->pose_msg_.pose.orientation.w = this->model_->GetWorldPose().rot.w;


            // publish to ros
            //this->pub_Queue->push(this->pose_msg_, this->pub_);

            pub_.publish(pose_msg_) ;




            //gzmsg << "Sent" << std::endl ;
            this->lock.unlock();

            // save last time stamp
            this->last_time_ = cur_time;

        }
    }

    void ObjectPosePlugin::P3DQueueThread()
    {
        static const double timeout = 0.01;

        while (this->rosnode_->ok())
        {
            this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(ObjectPosePlugin)
}