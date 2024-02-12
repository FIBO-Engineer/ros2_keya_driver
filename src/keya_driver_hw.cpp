#include <ros2_keya_driver/keya_driver_hw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>

namespace keya_driver_hardware_interface
{
    KeyaDriverHW::KeyaDriverHW()
    {
        RCLCPP_INFO(rclcpp::get_logger("constructor logger"), "KeyaDriverHW Constructor");
    }

    KeyaDriverHW::~KeyaDriverHW()
    {
        RCLCPP_INFO(rclcpp::get_logger("destructor logger"), "KeyaDriverHW Destructor");
    }

    void KeyaDriverHW::reconnect()
    {
        disconnect();
        connect();
    }
}
