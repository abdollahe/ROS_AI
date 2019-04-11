#ifndef SCHUNK_GRIPPER_PLUGIN_HH
#define SCHUNK_GRIPPER_PLUGIN_HH

#include <string>

// Custom Callback Queue
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_plugins/PubQueue.h>

#include <ur_robot_gazebo/Attach.h>
#include "ur_robot_gazebo/AttachRequest.h"
#include "ur_robot_gazebo/AttachResponse.h"

#include "std_msgs/Bool.h"

namespace gazebo
{

  class GazeboRosMagneticGripper : public ModelPlugin
  {
  public:
    /// \brief Constructor
    GazeboRosMagneticGripper();

    /// \brief Destructor
    virtual ~GazeboRosMagneticGripper();

    /// \brief Load the controller
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Magnetise
    bool Magnetise(void);

    /// \brief Demagnetise
    bool Demagnetise(void);

  private:
    bool magnetise_callback(std_srvs::Empty::Request  &req,
                           std_srvs::Empty::Response &res);

    bool demagnetise_callback(std_srvs::Empty::Request  &req,
                            std_srvs::Empty::Response &res);

    bool status_;

    physics::WorldPtr world_;
    physics::ModelPtr parent_;
    physics::LinkPtr link_;

    ros::NodeHandle nh_;

    ros::Publisher pub_;

    ros::ServiceServer magnetise_service_;
    ros::ServiceServer demagnetise_service_;

    ros::ServiceClient gazeboLinkerAttachClient_;
    ur_robot_gazebo::Attach::Request  attachReq_;
    ur_robot_gazebo::Attach::Response attachRes_;


    ros::ServiceClient gazeboLinkerDetachClient_;
    ur_robot_gazebo::Attach::Request  detachReq_;
    ur_robot_gazebo::Attach::Response detachRes_;

    std::string topic_name_;
    std::string service_name_;
    std::string link_name_;


    std::string robot_name_;
    std::string robot_namespace_;

  private:
      // rate control
      double update_rate_;
      // ros publish multi queue, prevents publish() blocking
      PubMultiQueue pmq;
      //ros::Publisher pub_;
      PubQueue<std_msgs::Bool>::Ptr pub_Queue;
      /// \brief save last_time
      common::Time last_time_;

      /// \brief ros message
      std_msgs::Bool msg_;

      /// \brief mutex to lock access to fields used in message callbacks
      boost::mutex lock;

      ros::CallbackQueue p3d_queue_;
      boost::thread callback_queue_thread_;

      /// Gazebo related methods
      void P3DQueueThread();


      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;
  public:
      void OnUpdate() ;

  };
}
#endif
