#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

class TrackerNode : public rclcpp::Node {
public:
    TrackerNode( )
        : Node("tracker") {
        // 创建订阅者
        float_subscriber_ =
            this->create_subscription< std_msgs::msg::Float32MultiArray >(
                "tracker/float_params", 10,
                std::bind(&TrackerNode::float_callback, this, std::placeholders::_1));
        int_subscriber_ =
            this->create_subscription< std_msgs::msg::Int32MultiArray >(
                "tracker/int_params", 10,
                std::bind(&TrackerNode::int_callback, this, std::placeholders::_1));
    }

private:
    void float_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(
            this->get_logger( ), "Received float parameters: %f, %f, %f",
            msg->data[0], msg->data[1], msg->data[2]);
    }

    void int_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(
            this->get_logger( ), "Received int parameters: %d, %d, %d",
            msg->data[0], msg->data[1], msg->data[2]);
    }

    rclcpp::Subscription< std_msgs::msg::Float32MultiArray >::SharedPtr float_subscriber_;
    rclcpp::Subscription< std_msgs::msg::Int32MultiArray >::SharedPtr   int_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared< TrackerNode >( );
    rclcpp::spin(node);
    rclcpp::shutdown( );
    return 0;
}