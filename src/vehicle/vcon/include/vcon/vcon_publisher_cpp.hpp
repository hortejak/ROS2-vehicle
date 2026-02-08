#ifndef VCON_PUBLISHER_HPP_
#define VCON_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/vcon.hpp"
// Replace with your actual message header
// #include "my_robot_msgs/msg/vehicle_specs.hpp" 

using VCON_msg = interfaces::msg::VCON;

class VCONPublisher : public rclcpp::Node
{
public:
    VCONPublisher();

private:
    void timer_callback();
    void load_vehicle_config();

    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<VCON_msg>::SharedPtr publisher_;
    VCON_msg vcon_msg_;
};

#endif  // VCON_PUBLISHER_HPP_