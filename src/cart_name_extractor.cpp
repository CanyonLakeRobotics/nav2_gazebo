
#include <nav2_gazebo/cart_name_extractor.hpp>

namespace nav2_gazebo
{

  CartContactDetector::CartContactDetector()
  : Node("cart_contact_detector")
  , in_contact_{false}
  , cart_name_{}
  , contact_topic_name_{"/latch_lever/contacts"}
  {
    contact_subscriber_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        contact_topic_name_,
        10,
        [&](const ros_gz_interfaces::msg::Contacts::SharedPtr msg){ 
            return contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);}
    )
  }


  void CartContactDetector::contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    auto contact = msg.contacts[0]; // ros_gz_interfaces/msgs/Contacts
    auto other_entity = contact.collision2; // ros_gz_interfaces/msgs/Entity
    cart_name_ = other_entity.name; // string
  }

}