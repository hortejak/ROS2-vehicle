#include "vcon/vcon_publisher_cpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

VCONPublisher::VCONPublisher() : Node("VCON_publisher_cpp")
{
    // 1. Load the file immediately on startup
    load_vehicle_config();

    // 2. Initialize timer
    timer_ = this->create_wall_timer(1000ms, std::bind(&VCONPublisher::timer_callback, this));

    publisher_ = this->create_publisher<VCON_msg>("params/VCON", 10);
}

void VCONPublisher::load_vehicle_config()
{
    try {
        // Automatically find the path in the install space
        std::string pkg_share = ament_index_cpp::get_package_share_directory("vcon");
        std::string file_path = pkg_share + "/VCON.yaml";

        YAML::Node config = YAML::LoadFile(file_path);

        vcon_msg_.id = config["vehicle"]["metadata"]["id"].as<int>();
        vcon_msg_.name = config["vehicle"]["metadata"]["name"].as<std::string>();

        vcon_msg_.vehicle_dimensions.length = config["vehicle"]["dimensions"]["length"].as<double>();
        vcon_msg_.vehicle_dimensions.width = config["vehicle"]["dimensions"]["width"].as<double>();
        vcon_msg_.vehicle_dimensions.height = config["vehicle"]["dimensions"]["height"].as<double>();
        vcon_msg_.vehicle_dimensions.wheelbase = config["vehicle"]["dimensions"]["wheelbase"].as<double>();
        vcon_msg_.vehicle_dimensions.track_width = config["vehicle"]["dimensions"]["track_width"].as<double>();

        RCLCPP_INFO(this->get_logger(), "Loaded config for: %s", vcon_msg_.name.c_str());
    } 
    catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
    }
}

void VCONPublisher::timer_callback()
{
    // Now you can use config_data_ whenever you want
    publisher_->publish(vcon_msg_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VCONPublisher>());
    rclcpp::shutdown();
    return 0;
}