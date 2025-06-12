#ifndef COMPONENT_DEMO__COMPONENT_DEMO_PUB_HPP_
#define COMPONENT_DEMO__COMPONENT_DEMO_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ne
{
    class ComponentDemoPub : public rclcpp::Node
    {
    private:
        size_t count_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        
        explicit ComponentDemoPub(const rclcpp::NodeOptions & options);
        ~ComponentDemoPub() = default;

    protected:
        void on_timer();
    };
}

#endif
