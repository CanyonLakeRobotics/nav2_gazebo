/**
 * Listens to the contacts topics and extracts the cart name to be published
 * 
 * **/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

namespace nav2_gazebo
{
  class CartContactDetector : public rclcpp::Node
  {
    public:

        CartContactDetector();

    private:

        void contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
    
        std::string contact_topic_name_;
        rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_subscriber_;
        
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cart_name_publisher_;
  };
}