#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LidarLifecycleNode(const std::string & node_name)
    : rclcpp_lifecycle::LifecycleNode(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Creating LidarLifecycleNode");
    }

protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring LidarLifecycleNode");
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Activating LidarLifecycleNode");
        publisher_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarLifecycleNode::publish_scan, this));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating LidarLifecycleNode");
        timer_->cancel();
        publisher_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up LidarLifecycleNode");
        publisher_.reset();
        timer_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down LidarLifecycleNode");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void publish_scan()
    {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        // 填充激光雷达数据
        publisher_->publish(*scan_msg);
    }

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLifecycleNode>("lidar_lifecycle_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    // 手动控制生命周期节点的状态转换
    node->configure();
    node->activate();

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
