#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <can_raw/msg/can_frame.hpp>
#include <vector>
#include <cmath>
#include "main_bus.hpp"

const double PI = 3.14159;


// In ROS2, node is an object
class MotorCtrlNode : public rclpp::Node {
public:
    MotorCtrlNode() : Node("motor_ctrl_node") { 
        // parameter declaration
        // the purpose of this is to allow the user to change the parameters as
        // they align to the real robot. All parameters are true to BOWIE
        this->declare_parameter<double>("wheel_diameter", 0.3);
        this->declare_parameter<double>("wheel_base", 1.0);
        this->declare_parameter<double("max_rpm", 100.0);
        this->declare_parameter<int>("frame_id", 0);

        this->get_parameter("wheel_diameter", wheel_diameter);
        this->get_parameter("wheel_base", wheel_base);
        this->get_parameter("max_rpm", max_rpm);
        this->get_parameter("frame_id", frame_id);
        
        // publisher declaration (This is different in ROS2 than in ROS1)
        // this node needs to convert the ros messages into can messages for the microcontrollers
        can_pub = this->create_publisher<can_raw::msg::CanFrame>("/canbus", 10);

        // subscriber declaration
        auto callback = [this](const geometry_msgs::msgs::Twist::SharedPtr msg) {
            this->handle_twist_message(msg);
        };
        subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, callback);
    }

private:
    // member variables, twist subscriber, can publisher
    void handle_twist_message(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // calculate the rpm for the left and right wheels
        int rpm_left, rpm_right;
        double wheel_circumference = PI * wheel_diameter;
        double v_left = msg->linear.m - msg->angular.z;
        double v_right = msg->linear.x + msg->angular.z;

        rpm_left = static_cast<int>(std::min(max_rpm, (v_left * 60) / wheel_circumference));
        rpm_right = static_cast<int>(std::min(max_rpm, (v_right * 60) / wheel_circumference));

        RCLPP_INFO(this->get_logger(), "RPM Left: %d, RPM Right: %d", rpm_left, rpm_right);

        can::MotorCommands cmd = {
            .left = {
                .speed = static_cast<double>(rpm_left)
            },
            .right = {
                .speed = static_cast<double>(rpm_right)
            }
        };

        uint8_t buffer[8];
        can::to_buffer(buffer, can::serialize(cmd));

        can_raw::msg::CanFrame can_frame;
        can_frame.id = static_cast<short int>(can::FrameID::MotorCommands);
        memcpy(can_frame.data.data(), buffer, sizeof(buffer));
    
        can_pub->publish(can_frame);
    }
    
    rclcpp::Publisher<can_raw::msg::CanFrame>::SharedPtr can_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;

    double wheel_diameter, wheel_base, max_rpm;
    int frame_id;
};

int main(int argc, char** argv) {
    rclpp::init(argc, argv);
    auto node = std::make_shared<MotorCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


