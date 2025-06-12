#include "component/component_demo_sub.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ne
{
    ComponentDemoSub::ComponentDemoSub(const rclcpp::NodeOptions & options)
        : Node("component_demo_sub", options), count_(0)
    {
        auto callback_group_sub_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_option = rclcpp::SubscriptionOptions();
        sub_option.callback_group = callback_group_sub_;

        sub_ = this->create_subscription<std_msgs::msg::String>("demo_component/topic", 10,
            [this](std_msgs::msg::String::UniquePtr msg){this->on_sub(msg);}, sub_option);
        
    }
    
    void ComponentDemoSub::on_sub(std_msgs::msg::String::UniquePtr &msg)
    {
        auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());

        // 打印线程ID
        RCLCPP_INFO(this->get_logger(), "SubThID: '%s'", std::to_string(hashed).c_str());
        
        RCLCPP_INFO(this->get_logger(), "Sub send:'%s'", msg->data.c_str());

        // 打印消息ID
        RCLCPP_INFO_STREAM(this->get_logger(), "Sub::Addr is: " << reinterpret_cast<std::uintptr_t>(msg.get()));
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ne::ComponentDemoSub)
