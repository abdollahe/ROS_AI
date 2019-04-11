//
// Created by abdollah on 7/04/19.
//

#ifndef SRC_CONTAINPLUGIN_H
#define SRC_CONTAINPLUGIN_H

#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
    class ContainPluginPrivate;

    /// \brief Plugin which emits gazebo transport messages according to whether
    /// an entity's origin is inside or outside a given volume. A message is only
    /// published when the state changes.
    ///
    /// Example usage:
    ///
    ///  <plugin name="containRobotArm" filename="libContainPlugin.so">
    ///
    ///    <!-- True to enable automatically, false so it must be enabled
    ///         via a message - true by default -->
    ///    <enabled>true</enabled>
    ///
    ///    <!-- Scoped name of entity to check -->
    ///    <entity>robot::arm_link</entity>
    ///
    ///    <!-- Namespace for gazebo transport topic and service:
    ///           /<namespace>/box/contains : topic where true / false messages
    ///                                       are published.
    ///           /<namespace>/box/enable : service to turn plugin on and off.
    ///    -->
    ///    <namespace>gazebo/robot</namespace>
    ///
    ///    <!-- Pose of the geometry's center point in world coordinates -->
    ///    <pose>10 10 2 0 0 1.57</pose>
    ///
    ///    <!-- Geometry of the volume to check, uses the SDF geometry spec:
    ///         http://sdformat.org/spec?ver=1.6&elem=geometry
    ///         currently <box> is supported -->
    ///    <geometry>
    ///      <box>
    ///        <size>1 1 4</size>
    ///      </box>
    ///    </geometry>
    ///
    ///  </plugin>
    ///
    class GAZEBO_VISIBLE ContainPlugin : public WorldPlugin
{
    // Documentation inherited
    public: ContainPlugin();

    // Documentation inherited
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \brief Called every world iteration on world update begin.
    /// \param[in] _info Update info.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Callback for enable "service".
    /// \param[in] _msg Message with 0 to disable and 1 to enable the plugin.
    private: void Enable(ConstIntPtr &_msg);

    /// \brief Pointer to private data
    private: std::unique_ptr<ContainPluginPrivate> dataPtr;
};
}



#endif //SRC_CONTAINPLUGIN_H
