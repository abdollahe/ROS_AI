//
// Created by abdollah on 7/04/19.
//

#include <string>

#include <ignition/math/OrientedBox.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/UpdateInfo.hh"

#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/transport/Node.hh"

#include "../include/ContainPlugin.h"


namespace gazebo
{
    /// \brief Private data class for the ContainPlugin class
    class ContainPluginPrivate
    {
        /// \brief Connection to world update.
    public: event::ConnectionPtr updateConnection;

        /// \brief Pointer to the world.
    public: physics::WorldPtr world;

        /// \brief Scoped name of the entity we're checking.
    public: std::string entityName;

        /// \brief Pointer to the entity we're checking.
    public: physics::EntityPtr entity;

        /// \brief Box representing the volume to check.
    public: ignition::math::OrientedBoxd box;

        /// \brief Gazebo transport node for communication.
    public: transport::NodePtr gzNode;

        /// \brief Publisher which publishes contain / doesn't contain messages.
    public: transport::PublisherPtr containGzPub;

        /// \brief Subscriber to enable messages.
    public: transport::SubscriberPtr enableGzSub;

        /// \brief Namespace for the topics:
        /// /<ns>/contain
        /// /<ns>/enable
    public: std::string ns;

        /// \brief 1 if contains, 0 if doesn't contain, -1 if unset
    public: int contain = -1;
    };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ContainPlugin)

/////////////////////////////////////////////////
ContainPlugin::ContainPlugin() : WorldPlugin(),
                                 dataPtr(new ContainPluginPrivate)
{
}

/////////////////////////////////////////////////
void ContainPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    // Entity name
    if (!_sdf->HasElement("entity"))
    {
        gzerr << "Missing required parameter <entity>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    this->dataPtr->entityName = _sdf->Get<std::string>("entity");

    // Namespace
    if (!_sdf->HasElement("namespace"))
    {
        gzerr << "Missing required parameter <namespace>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    this->dataPtr->ns = _sdf->Get<std::string>("namespace");

    // Pose
    if (!_sdf->HasElement("pose"))
    {
        gzerr << "Missing required parameter <pose>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    auto pose = _sdf->Get<ignition::math::Pose3d>("pose");

    // Geometry
    if (!_sdf->HasElement("geometry"))
    {
        gzerr << "Missing required parameter <geometry>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    auto geometryElem = _sdf->GetElement("geometry");

    // Only box for now
    if (!geometryElem->HasElement("box"))
    {
        gzerr << "Missing required parameter <box>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    auto boxElem = geometryElem->GetElement("box");

    if (!boxElem->HasElement("size"))
    {
        gzerr << "Missing required parameter <size>, plugin will not be "
              << "initialized." << std::endl;
        return;
    }
    auto size = boxElem->Get<ignition::math::Vector3d>("size");

    this->dataPtr->box = ignition::math::OrientedBoxd(size, pose);

    this->dataPtr->world = _world;

    // Start/stop
    this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
    this->dataPtr->gzNode->Init();
    this->dataPtr->enableGzSub = this->dataPtr->gzNode->Subscribe("/" +
                                                                  this->dataPtr->ns + "/enable", &ContainPlugin::Enable, this);

    auto enabled = true;
    if (_sdf->HasElement("enabled"))
        enabled = _sdf->Get<bool>("enabled");

    if (enabled)
    {
        boost::shared_ptr<msgs::Int> msg(new msgs::Int());
        msg->set_data(1);
        this->Enable(msg);
    }
}

//////////////////////////////////////////////////
void ContainPlugin::Enable(ConstIntPtr &_msg)
{
    auto enable = _msg->data() == 1;

    // Already started
    if (enable && this->dataPtr->updateConnection)
    {
        gzwarn << "Contain plugin is already enabled." << std::endl;
        return;
    }

    // Already stopped
    if (!enable && !this->dataPtr->updateConnection)
    {
        gzwarn << "Contain plugin is already disabled." << std::endl;
        return;
    }

    // Start
    if (enable)
    {
        // Start update
        this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ContainPlugin::OnUpdate, this, std::placeholders::_1));

        this->dataPtr->containGzPub = this->dataPtr->gzNode->Advertise<msgs::Int>(
                "/" + this->dataPtr->ns + "/contain");

        gzmsg << "Started contain plugin [" << this->dataPtr->ns << "]"
              << std::endl;
        return;
    }

    // Stop
    {
        this->dataPtr->updateConnection.reset();
        this->dataPtr->containGzPub.reset();
        this->dataPtr->contain = -1;

        gzmsg << "Stopped contain plugin [" << this->dataPtr->ns << "]"
              << std::endl;
        return;
    }
}

/////////////////////////////////////////////////
void ContainPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
    // Only get the entity once
    if (!this->dataPtr->entity)
    {
        this->dataPtr->entity = this->dataPtr->world->GetEntity(
                this->dataPtr->entityName);

        // Entity may not have been spawned yet
        if (!this->dataPtr->entity)
            return;
    }

    auto pos = this->dataPtr->entity->GetWorldPose().Ign().Pos();
    auto containNow = this->dataPtr->box.Contains(pos) ? 1 : 0;

    if (containNow != this->dataPtr->contain)
    {
        this->dataPtr->contain = containNow;

        msgs::Int msg;
        msg.set_data(this->dataPtr->contain);
        this->dataPtr->containGzPub->Publish(msg);
    }
}
