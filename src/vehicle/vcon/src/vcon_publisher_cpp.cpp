#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class VCONPublisher : public rclcpp::Node
{
    public:
        VCONPublisher()
        : Node("VCON_publisher_cpp"), count_(0)
        {
            timer_ = this->create_wall_timer(1000ms, std::bind(&VCONPublisher::timer_callback, this));
        }
    
    private:
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Working");
        }

        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCONPublisher>());
  rclcpp::shutdown();
  return 0;
}