//
// Created by abdollah on 7/04/19.
//

#ifndef SRC_OBJECTCONTACTPLUGIN_H
#define SRC_OBJECTCONTACTPLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>
#include "ContactPlugin.h"


namespace gazebo {
    class GAZEBO_VISIBLE ObjectContactPlugin : public ContactPlugin {
/// \brief Constructor.
    public: ObjectContactPlugin();

    /// \brief Destructor.
    public: virtual ~ObjectContactPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Act on models that are ontop of the sensor's link
    protected: void ActOnContactingModels();

    /// \brief If true, only delete models if their CoG is within the bounding box of the link
    protected: bool centerOfGravityCheck;

    /// \brief Pose where the object will be teleported.
    protected: math::Pose disposalPose;
    };
}




#endif //SRC_OBJECTCONTACTPLUGIN_H
