#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace nav2_gazebo
{

class FakeLatchPlugin : public ignition::gazebo::System,
                        public ignition::gazebo::ISystemConfigure,
                        public ignition::gazebo::ISystemPreUpdate

{
public:
  FakeLatchPlugin();

  /**
   * Implements the Configure interface for ignition system plugin
   * This is called once when the plugin is loaded
   */
  void Configure(const ignition::gazebo::Entity& entity,
                 [[maybe_unused]] const std::shared_ptr<const sdf::Element>& sdf,
                 ignition::gazebo::EntityComponentManager& ecm,
                 [[maybe_unused]] ignition::gazebo::EventManager& _eventMgr) override;

  /**
   * Implements the PreUpdate interface for ignition system plugin
   * This method is called once every iteration, any modification to system state
   * must be implemented here, eg: creating and deleting joints
   */
  void PreUpdate([[maybe_unused]] const ignition::gazebo::UpdateInfo& _nfo,
                 ignition::gazebo::EntityComponentManager& ecm) override;

  void LatchMsgCallback(const ignition::msgs::StringMsg& latch_msg);

  void UnlatchMsgCallback(const ignition::msgs::StringMsg& unlatch_msg);

private:
  bool latched_;
  bool unlatch_requested_;
  std::string robot_name_;
  std::string cart_name_;
  ignition::gazebo::Entity robot_;
  ignition::gazebo::Entity cart_;
  ignition::gazebo::Entity latch_joint_;
  ignition::transport::Node transport_node_;
};

}  // namespace nav2_gazebo
