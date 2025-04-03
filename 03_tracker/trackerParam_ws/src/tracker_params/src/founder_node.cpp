#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class FounderNode : public rclcpp::Node {
public:
    FounderNode( )
        : Node("founder") {
        // 获取包的安装路径
        std::string package_name = "tracker_params";    // 替换为你的包名
        std::string config_file  = this->declare_parameter< std::string >(
            "config_file",
            "params.yaml");    // 默认配置文件名
        // 构造完整的配置文件路径
        std::string config_path = ament_index_cpp::get_package_share_directory(package_name) + "/config/" + config_file;

        RCLCPP_INFO(this->get_logger( ), "Loading YAML file: %s", config_path.c_str( ));
        std::ifstream file_stream(config_path);
        if (!file_stream.is_open( )) {
            RCLCPP_ERROR(this->get_logger( ), "Failed to open YAML file: %s", config_path.c_str( ));
            throw std::runtime_error("Failed to open YAML file");
        }
        YAML::Node config = YAML::Load(file_stream);

        ekf_xyz_ = config["ekf_xyz"].as< float >( );
        ekf_yaw_ = config["ekf_yaw"].as< float >( );
        ekf_r_   = config["ekf_r"].as< float >( );
        vx_      = config["vx"].as< int >( );
        vy_      = config["vy"].as< int >( );
        vz_      = config["vz"].as< int >( );

        // 输出确认读取的参数
        RCLCPP_INFO(this->get_logger( ), "Read parameters:");
        RCLCPP_INFO(this->get_logger( ), "ekf_xyz: %f", ekf_xyz_);
        RCLCPP_INFO(this->get_logger( ), "ekf_yaw: %f", ekf_yaw_);
        RCLCPP_INFO(this->get_logger( ), "ekf_r: %f", ekf_r_);
        RCLCPP_INFO(this->get_logger( ), "vx: %d", vx_);
        RCLCPP_INFO(this->get_logger( ), "vy: %d", vy_);
        RCLCPP_INFO(this->get_logger( ), "vz: %d", vz_);

        // 创建发布者
        float_publisher_ = this->create_publisher< std_msgs::msg::Float32MultiArray >("tracker/float_params", 10);
        int_publisher_   = this->create_publisher< std_msgs::msg::Int32MultiArray >("tracker/int_params", 10);

        // 创建定时器，每秒发布一次参数
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FounderNode::publish_params, this));
    }

private:
    void publish_params( ) {
        // 创建并填充浮点数消息
        std_msgs::msg::Float32MultiArray float_msg;
        float_msg.data.push_back(ekf_xyz_);
        float_msg.data.push_back(ekf_yaw_);
        float_msg.data.push_back(ekf_r_);

        // 创建并填充整数消息
        std_msgs::msg::Int32MultiArray int_msg;
        int_msg.data.push_back(vx_);
        int_msg.data.push_back(vy_);
        int_msg.data.push_back(vz_);

        // 发布消息
        float_publisher_->publish(float_msg);
        int_publisher_->publish(int_msg);

        RCLCPP_INFO(this->get_logger( ), "Published float parameters: %f, %f, %f", ekf_xyz_, ekf_yaw_, ekf_r_);
        RCLCPP_INFO(this->get_logger( ), "Published int parameters: %d, %d, %d", vx_, vy_, vz_);
    }

    float ekf_xyz_;
    float ekf_yaw_;
    float ekf_r_;
    int   vx_;
    int   vy_;
    int   vz_;

    rclcpp::Publisher< std_msgs::msg::Float32MultiArray >::SharedPtr float_publisher_;
    rclcpp::Publisher< std_msgs::msg::Int32MultiArray >::SharedPtr   int_publisher_;
    rclcpp::TimerBase::SharedPtr                                     timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared< FounderNode >( );
    rclcpp::spin(node);
    rclcpp::shutdown( );
    return 0;
}