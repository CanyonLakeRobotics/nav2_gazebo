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

        void PublishCartName();

        void contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);
    
        std::atomic<bool> in_contact_;
        std::atomic<std::string> cart_name_;
        std::string contact_topic_name_;
        rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_subscriber_;
  };
}