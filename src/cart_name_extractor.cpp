
#include <string>
#include <nav2_gazebo/cart_name_extractor.hpp>

namespace nav2_gazebo
{

  CartContactDetector::CartContactDetector()
  : Node("cart_contact_detector")
  , contact_topic_name_{"/latch_lever/contact"}
  {
    contact_subscriber_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        contact_topic_name_,
        10,
        [&](const ros_gz_interfaces::msg::Contacts::SharedPtr msg){ 
            return contactCallback(msg);}
    );


    cart_name_publisher_ = this->create_publisher<std_msgs::msg::String>("/latch_lever/cart_name", 10);

  }

  void CartContactDetector::contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    auto contact = msg->contacts[0]; // ros_gz_interfaces/msgs/Contacts
    auto other_entity = contact.collision2; // ros_gz_interfaces/msgs/Entity
    
    auto object_in_contact = other_entity.name.substr(0, other_entity.name.find("::"));

    // Check if the object is actually a cart
    if( object_in_contact.substr(0, object_in_contact.find("_")) == "cart")
    {
    auto cart_name_msg = std_msgs::msg::String();
    cart_name_msg.data = object_in_contact; // string
    RCLCPP_INFO_STREAM(this->get_logger(), "Cart name: " << cart_name_msg.data);
    cart_name_publisher_->publish(cart_name_msg);
    }

  }

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_gazebo::CartContactDetector>());
  rclcpp::shutdown();
  return 0;
}