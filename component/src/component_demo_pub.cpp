#include "component/component_demo_pub.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ne
{
    ComponentDemoPub::ComponentDemoPub(const rclcpp::NodeOptions & options)
        : Node("component_demo_pub", options), count_(0)
    {
        
        pub_ = create_publisher<std_msgs::msg::String>("demo_component/topic", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {return this->on_timer();});
        
    }
    
    void ComponentDemoPub::on_timer()
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello World:" + std::to_string(++count_);

        auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());

        // 打印线程ID
        RCLCPP_INFO(this->get_logger(), "PubThID: '%s'", std::to_string(hashed).c_str());

        RCLCPP_INFO(this->get_logger(), "Pub: '%s'", msg->data.c_str());

        // 打印消息ID
        RCLCPP_INFO_STREAM(this->get_logger(), "Pub:Addr is:" << reinterpret_cast<std::uintptr_t>(msg.get()));

        std::flush(std::cout);

        pub_->publish(std::move(msg));
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ne::ComponentDemoPub)
