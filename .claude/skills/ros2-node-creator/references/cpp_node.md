# C++ Node Templates

## Header — `include/<pkg>/<node>_cpp.hpp`

```cpp
#ifndef <PKG>_<NODE>_CPP_HPP_
#define <PKG>_<NODE>_CPP_HPP_

#include <chrono>
#include <memory>
#include <string>

// Include yaml-cpp only when loading config files
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

// Message headers follow the pattern: <package>/msg/<MsgName_snake>.hpp
#include "interfaces/msg/vcon.hpp"
// #include "std_msgs/msg/float64.hpp"

// Type aliases keep the code readable and easy to update when message types change.
using MyMsg = interfaces::msg::VCON;

class <Node>Publisher : public rclcpp::Node
{
public:
    <Node>Publisher();

private:
    // Separate config loading from the constructor so errors are easier to isolate.
    void load_config();
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<MyMsg>::SharedPtr publisher_;
    // rclcpp::Subscription<SomeMsg>::SharedPtr subscription_;

    MyMsg msg_;
};

#endif  // <PKG>_<NODE>_CPP_HPP_
```

### Header naming rule

The include guard follows `<PKG>_<NODE>_CPP_HPP_` in SCREAMING_SNAKE_CASE. Use the full file path relative to `include/` so collisions across packages are impossible.

---

## Source — `src/<node>_cpp.cpp`

```cpp
#include "<pkg>/<node>_cpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

<Node>Publisher::<Node>Publisher() : Node("<node>_cpp")
{
    // Load configuration before creating any publishers/timers so that
    // the node fails fast and loudly if the config file is missing or malformed.
    load_config();

    publisher_ = this->create_publisher<MyMsg>("<topic/name>", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&<Node>Publisher::timer_callback, this));
}

void <Node>Publisher::load_config()
{
    try {
        // get_package_share_directory resolves to the install space at runtime,
        // so this path works correctly whether the workspace is sourced locally
        // or installed to /opt/ros/<distro>.
        std::string pkg_share = ament_index_cpp::get_package_share_directory("<pkg>");
        std::string file_path = pkg_share + "/config.yaml";

        YAML::Node config = YAML::LoadFile(file_path);

        // Example: read a nested value and store it in a member variable.
        // double my_param = config["section"]["key"].as<double>();

        RCLCPP_INFO(this->get_logger(), "Config loaded from: %s", file_path.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
    }
}

void <Node>Publisher::timer_callback()
{
    publisher_->publish(msg_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<<Node>Publisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Subscriber Variant

If the node subscribes rather than (or in addition to) publishing, add to the header:

```cpp
rclcpp::Subscription<IncomingMsg>::SharedPtr subscription_;
void on_message(const IncomingMsg::SharedPtr msg);
```

And in the source constructor:

```cpp
subscription_ = this->create_subscription<IncomingMsg>(
    "<input_topic>", 10,
    std::bind(&<Node>Publisher::on_message, this, std::placeholders::_1)
);
```

## Logging Conventions

| Situation | Macro |
|-----------|-------|
| Normal info (startup, config loaded) | `RCLCPP_INFO` |
| Something unexpected but recoverable | `RCLCPP_WARN` |
| Failed to load config, bad message | `RCLCPP_ERROR` |
| Unrecoverable, about to crash | `RCLCPP_FATAL` |
| High-frequency debug (disable in release) | `RCLCPP_DEBUG` |

Never use `std::cout` in ROS2 nodes — it bypasses the ROS logging system and won't appear in `ros2 topic echo /rosout`.
