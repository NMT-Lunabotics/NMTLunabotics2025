#include <rclcpp/rclcpp.hpp>
#include <can_raw/msg/can_frame.hpp>
#include "can_interface.hpp"

class Node : public rclcpp::Node {
    rclcpp::Subscription<can_raw::msg::CanFrame>::SharedPtr subscriber;
    SocketCAN socket;

public:
    Node() : Node("can_raw_node") {}

    Node(const std::string &node_name) : rclcpp::Node(node_name) {
        subscriber = this->create_subscription<can_raw::msg::CanFrame>(
                "/canbus", 16, std::bind(&Node::handle_message, this, std::placeholders::_1));
        socket = SocketCAN("can0");
    }

    void handle_message(const can_raw::msg::CanFrame::SharedPtr frame) {
        socket.transmit(static_cast<int>(frame->id), frame->data.data());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<Node>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "can_raw error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
