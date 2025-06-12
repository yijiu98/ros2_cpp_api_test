#ifndef COMPONENT_DEMO__COMPONENT_DEMO_SUB_HPP_
#define COMPONENT_DEMO__COMPONENT_DEMO_SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ne
{
    class ComponentDemoSub : public rclcpp::Node
    {
    private:
        size_t count_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    public:
        
        explicit ComponentDemoSub(const rclcpp::NodeOptions & options);
        ~ComponentDemoSub() = default;

    protected:
        void on_sub(std_msgs::msg::String::UniquePtr &msg);
    };
}

#endif
