/*----------*/
/* Includes */
/*----------*/
#include <algorithm>
#include <assert.h>
#include <std_msgs/Bool.h>
#include <schunk_gripper_plugin.h>
#include "ur_robot_gazebo/StampedGripperState.h"


/*-------------*/
/* Definitions */
/*-------------*/
#define GRIPPING_DIST_THRESH 0.2


/*-------------------------------------------------*/
/* Gazebo plugin for simulating a magnetic gripper */
/*-------------------------------------------------*/
namespace gazebo
{
  /*---------------------------------------*/
  /* Register plugin with Gazebo simulator */
  /*---------------------------------------*/
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosMagneticGripper)

  /*-------------*/
  /* Constructor */
  /*-------------*/
  GazeboRosMagneticGripper::GazeboRosMagneticGripper() :
    status_(false)
  {
  }

  /*------------*/
  /* Destructor */
  /*------------*/
  GazeboRosMagneticGripper::~GazeboRosMagneticGripper()
  {
  }

  /*-----------------------------------------------*/
  /* Load Gazebo environment and advertise service */
  /*-----------------------------------------------*/
  void GazeboRosMagneticGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO_NAMED( "magnetic_gripper", "Loading schunk_magnetic_gripper");

    /*--------------------------------------------*/
    /* Get Gazebo world, model and sdf properties */
    /*--------------------------------------------*/
    this->world_ = _model->GetWorld();
    this->parent_ = _model;
    this->robot_name_ = _model->GetName();
    this->robot_namespace_ = "";

    /*--------------------------------*/
    /* Validate sdf robot description */
    /*--------------------------------*/
    if (_sdf->HasElement("robotNamespace"))
      this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    if (!_sdf->HasElement("bodyName"))
    {
      ROS_FATAL_NAMED("magnetic_gripper", "magnetic_gripper plugin missing <bodyName>, cannot proceed");
      return;
    }
    else
    {
      this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    }
    this->link_ = _model->GetLink(this->link_name_);
    if (!this->link_)
    {
      std::string found;
      physics::Link_V links = _model->GetLinks();
      for (size_t i = 0; i < links.size(); i++) {
        found += std::string(" ") + links[i]->GetName();
      }
      ROS_FATAL_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper plugin error: link named: %s does not exist", link_name_.c_str());
      ROS_FATAL_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper plugin error: You should check it exists and is not connected with fixed joint");
      ROS_FATAL_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper plugin error: Found links are: %s", found.c_str());
      return;
    }
    if (!_sdf->HasElement("topicName"))
    {
      //ROS_FATAL_NAMED("magnetic_gripper", "magnetic_gripper plugin missing <serviceName>, cannot proceed");
      //return;
      this->topic_name_ = "magnetic_gripper_status" ;
    }
    else
    {
      this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    }
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("magnetic_gripper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    /*------------------------*/
    /* Advertise the services */
    /*------------------------*/
    this->nh_ = ros::NodeHandle(this->robot_namespace_);

    this->magnetise_service_ = this->nh_.advertiseService("on", &GazeboRosMagneticGripper::magnetise_callback, this);
    ROS_INFO_STREAM("Magnetise service at: " << this->nh_.resolveName("on"));

    this->demagnetise_service_ = this->nh_.advertiseService("off", &GazeboRosMagneticGripper::demagnetise_callback,this);
    ROS_INFO_STREAM("Demagnetise service at: " << this->nh_.resolveName("off"));

    ROS_INFO_NAMED("magnetic_gripper", "Loaded gazebo_ros_magnetic_gripper");

    /*-----------------*/
    /* Service clients */
    /*-----------------*/

    // Create a service client called gazeboLinkerClient for the service: /link_attacher_node/attach
    this->gazeboLinkerAttachClient_ = this->nh_.serviceClient<ur_robot_gazebo::Attach>("/link_attacher_node/attach");

    // Create a service client called gazeboLinkerClient for the service: /link_attacher_node/detach
    this->gazeboLinkerDetachClient_ = this->nh_.serviceClient<ur_robot_gazebo::Attach>("/link_attacher_node/detach");

    /*-------------------------------------*/
    /* Check required services are running */
    /*-------------------------------------*/

    //Make sure /link_attacher_node/attach service is service_ready
    bool service_ready = false;
    while (!service_ready){
      service_ready = ros::service::exists("/link_attacher_node/attach", true);
      ROS_INFO_NAMED("magnetic_gripper", "waiting for /link_attacher_node/attach service");
    }
    ROS_INFO_NAMED("magnetic_gripper", "/link_attacher_node/attach service is ready");


    // Make sure /link_attacher_node/detach service is service_ready
    service_ready = false;
    while (!service_ready){
      service_ready = ros::service::exists("/link_attacher_node/detach", true);
      ROS_INFO_NAMED("magnetic_gripper", "waiting for /link_attacher_node/detach service");
    }
    ROS_INFO_NAMED("magnetic_gripper", "/link_attacher_node/detach service is ready");


    if (!_sdf->HasElement("updateRate"))
    {
      ROS_DEBUG_NAMED("Magnetic Gripper", " Magnetic Gripper plugin missing <updateRate>, defaults to 10.0hz"
                                            " (as fast as possible)");
      this->update_rate_ = 10;
    }
    else
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>() ;




    // publish multi queue
    this->pmq.startServiceThread();

    gzmsg << "The topic name for the publisher is: " << this->topic_name_ << std::endl ;

    this->pub_ = this->nh_.advertise<std_msgs::Bool>(this->topic_name_, 1);


    #if GAZEBO_MAJOR_VERSION >= 8
        this->last_time_ = this->world_->SimTime();
    #else
        this->last_time_ = this->world_->GetSimTime();
    #endif


    // start custom queue for p3d
    this->callback_queue_thread_ = boost::thread(
            boost::bind(&GazeboRosMagneticGripper::P3DQueueThread, this));

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboRosMagneticGripper::OnUpdate, this));

  }


    void GazeboRosMagneticGripper::OnUpdate() {
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


      //gzmsg << "In data processing" << std::endl ;
      // copy data into pose message
      this->msg_.data = this->status_ ;


      // publish to ros
      //this->pub_Queue->push(this->msg_, this->pub_);

      pub_.publish(this->msg_) ;

      //gzmsg << "Sent" << std::endl ;
      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;

    }

    void GazeboRosMagneticGripper::P3DQueueThread()
    {
      static const double timeout = 0.01;

      while (this->nh_.ok())
      {
        this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }


  /*-----------------------------*/
  /* Magnetise callback function */
  /*-----------------------------*/
  bool GazeboRosMagneticGripper::magnetise_callback(std_srvs::Empty::Request  &req,
                                                    std_srvs::Empty::Response &res)
  {
    if (this->status_){
      ROS_WARN_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper: status: already on");
    } else {
      this->status_ = true;
      GazeboRosMagneticGripper::Magnetise();
      ROS_INFO_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper: status: off -> on");
    }
    return true;
  }

  /*-------------------------------*/
  /* Demagnetise callback function */
  /*-------------------------------*/
  bool GazeboRosMagneticGripper::demagnetise_callback(std_srvs::Empty::Request  &req,
                                                      std_srvs::Empty::Response &res)
  {
    if (status_) {
      status_ = false;
      GazeboRosMagneticGripper::Demagnetise();
      ROS_INFO_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper: status: on -> off");
    } else {
      ROS_WARN_NAMED("magnetic_gripper", "gazebo_ros_magnetic_gripper: status: already off");
    }
    return true;
  }

  /*------------------------*/
  /* Magnetisation function */
  /*------------------------*/
  bool GazeboRosMagneticGripper::Magnetise()
  {
    ignition::math::Pose3d parent_pose = this->link_->GetWorldPose().Ign();
    physics::Model_V models = this->world_->GetModels();

    for (size_t i = 0; i < models.size(); i++) {

      ROS_WARN_STREAM("Checking the num of models: " << models.size()) ;

      if (models[i]->GetName() == this->link_->GetName() ||
          models[i]->GetName() == this->parent_->GetName())
      {
        continue;
      }
      physics::Link_V links = models[i]->GetLinks();



      for (size_t j = 0; j < links.size(); j++) {


        ROS_WARN_STREAM("The link is for " << j << " , " << links[j]->GetName()) ;
        ignition::math::Pose3d link_pose = links[j]->GetWorldPose().Ign();
        ignition::math::Pose3d diff = parent_pose - link_pose;

        double norm = diff.Pos().Length();


        ROS_WARN_STREAM("Norm is: " << norm) ;

        //Debugging ROS console output
        //ROS_INFO_STREAM("Model " << i << ": " << models[i]->GetName() << " has norm distance of:" << norm);
        //ROS_INFO_STREAM("status_ " << "has value of " << this->status_);

        if (norm < GRIPPING_DIST_THRESH)
        {
          //Debugging ROS console output
          // ROS_FATAL_STREAM("Model " << i << ": " << models[i]->GetName() << "has norm distance: " << norm);
          // ROS_FATAL_STREAM("Link found is link no." << j << " with name: " << links[j]->GetName());

          this->attachReq_.model_name_1 = this->robot_name_;
          this->attachReq_.link_name_1  = this->link_name_;
          this->attachReq_.model_name_2 = models[i]->GetName();
          this->attachReq_.link_name_2  = links[j]->GetName();

          bool call_service = this->gazeboLinkerAttachClient_.call(this->attachReq_, this->attachRes_);
          if (call_service) {
            if (!this->attachRes_.ok)
              ROS_WARN_NAMED("magnetic_gripper", "Attach service response: failed!");
          } else {
            ROS_WARN_NAMED("magnetic_gripper", "Call to attach service failed");
          }
        }
      }
    }
    return true;
  }

  /*--------------------------*/
  /* Demagnetisation function */
  /*--------------------------*/
  bool GazeboRosMagneticGripper::Demagnetise()
  {
    this->detachReq_.model_name_1 = this->attachReq_.model_name_1;
    this->detachReq_.link_name_1  = this->attachReq_.link_name_1;
    this->detachReq_.model_name_2 = this->attachReq_.model_name_2;
    this->detachReq_.link_name_2  = this->attachReq_.link_name_2;

    bool call_service = this->gazeboLinkerDetachClient_.call(this->detachReq_, this->detachRes_);
    if (call_service) {
      if (!this->detachRes_.ok)
        ROS_WARN_NAMED("magnetic_gripper", "Detach service response: failed!");
    } else {
      ROS_WARN_NAMED("magnetic_gripper", "Call to detach service failed");
    }
    return true;
  }
}
