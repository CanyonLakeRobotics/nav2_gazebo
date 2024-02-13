#include <nav2_gazebo/fake_latch_plugin.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>

namespace nav2_gazebo
{

FakeLatchPlugin::FakeLatchPlugin()
  : latched_{ false }
  , unlatch_requested_{ false }
  , robot_name_{ "" }
  , cart_name_{ "" }
  , robot_{ ignition::gazebo::kNullEntity }
  , cart_{ ignition::gazebo::kNullEntity }
  , latch_joint_{ ignition::gazebo::kNullEntity }
{
}

void FakeLatchPlugin::Configure(const ignition::gazebo::Entity& entity,
                                [[maybe_unused]] const std::shared_ptr<const sdf::Element>& sdf,
                                ignition::gazebo::EntityComponentManager& ecm,
                                [[maybe_unused]] ignition::gazebo::EventManager& eventMgr)
{
  std::string latch_topic = "fake_latch/latch";
  std::string unlatch_topic = "fake_latch/unlatch";
  auto latch_sub = transport_node_.Subscribe(latch_topic, &FakeLatchPlugin::LatchMsgCallback, this);
  auto unlatch_sub = transport_node_.Subscribe(unlatch_topic, &FakeLatchPlugin::UnlatchMsgCallback, this);
  if (!latch_sub)
  {
    ignerr << "Could not create subscription to: " << latch_topic << std::endl;
  }
  else if (!unlatch_sub)
  {
    ignerr << "Could not create subscription to: " << unlatch_topic << std::endl;
  }
  else
  {
    ignition::gazebo::Model model(entity);
    robot_ = model.LinkByName(ecm, "base_link");

    robot_name_ = ecm.Component<ignition::gazebo::components::Name>(entity)->Data();
    ignwarn << "FakeLatchPlugin has been configured for robot: " << robot_name_ << std::endl;
  }
}

void FakeLatchPlugin::PreUpdate([[maybe_unused]] const ignition::gazebo::UpdateInfo& info,
                                ignition::gazebo::EntityComponentManager& ecm)
{
  // TODO: Exception handling if cart with given name not not found

  if (latched_ && unlatch_requested_)
  {
    // Remove joint entity
    auto cart_name = ecm.Component<ignition::gazebo::components::Name>(cart_)->Data();
    ignwarn << "Unlatching " << cart_name << " from " << robot_name_ << std::endl;
    ecm.RequestRemoveEntity(latch_joint_);
    latch_joint_ = ignition::gazebo::kNullEntity;
    cart_ = ignition::gazebo::kNullEntity;

    unlatch_requested_ = false;
    latched_ = false;
  }
  else if (!cart_name_.empty() && !latched_ && !unlatch_requested_)
  {
    latch_joint_ = ecm.CreateEntity();  // Create a detachable joint entity
    cart_ = ecm.EntityByComponents(ignition::gazebo::components::Name(cart_name_));
    auto cart_name = ecm.Component<ignition::gazebo::components::Name>(cart_)->Data();
    ignwarn << "Latching " << cart_name << " to " << robot_name_ << std::endl;
    ignition::gazebo::Model cart_model(cart_);
    ecm.CreateComponent(latch_joint_, ignition::gazebo::components::DetachableJoint(
                                          { robot_, cart_model.LinkByName(ecm, "base_link"), "fixed" }));
    latched_ = true;
  }
}

void FakeLatchPlugin::LatchMsgCallback(const ignition::msgs::StringMsg& latch_msg)
{
  if (!latched_)
  {
    ignwarn << "Latch requested for: " << latch_msg.data() << std::endl;
    cart_name_ = latch_msg.data();
  }
}

void FakeLatchPlugin::UnlatchMsgCallback(const ignition::msgs::StringMsg& unlatch_msg)
{
  if (latched_ && (cart_name_ == unlatch_msg.data()))
  {
    ignwarn << "Unlatch requested for: " << unlatch_msg.data() << std::endl;
    cart_name_ = "";
    unlatch_requested_ = true;
  }
  else
  {
    ignwarn << "Requested cart: " << unlatch_msg.data() << " is not currently latched to: " << robot_name_ << std::endl;
  }
}
}  // namespace nav2_gazebo

IGNITION_ADD_PLUGIN(nav2_gazebo::FakeLatchPlugin, ignition::gazebo::System,
                    nav2_gazebo::FakeLatchPlugin::ISystemConfigure, nav2_gazebo::FakeLatchPlugin::ISystemPreUpdate)
