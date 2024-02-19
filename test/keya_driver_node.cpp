#include <ros2_keya_driver/can_driver_hw.hpp>
#include <ros2_keya_driver/keya_codec.hpp>
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.h>
#include <string>

using namespace keya_driver_hardware_interface;

class KeyaNode : public rclcpp::Node
{
    public:
        KeyaNode() : Node("keya_driver")
        // keya_driver_hw_(std::make_shared<keya_driver_hardware_interface::KeyaDriverHW>()),
        {
            this->declare_parameter("device_id","can0");
            this->declare_parameter("can_id_list", std::vector<int64_t>{1});
            // this->declare_parameter<std::string>("device_id", "can0");

            // Get parameters value
            auto device_id = get_parameter("device_id").as_string();
            auto can_id_list = get_parameter("can_id_list").as_integer_array();

            // Print parameters
            // RCLCPP_INFO(get_logger(), "device_id: %s", device_id.c_str());
            // RCLCPP_INFO(get_logger(), "can_id_list: %i", static_cast<int>(can_id_list[0]));

            // Logging error
            // std::string device_id;
            // this->get_parameter("device_id", device_id);
            if(!(this->get_parameter("device_id", device_id)) || !(this-get_parameter("can_id_list", can_id_list)))
            {
                RCLCPP_FATAL(this->get_logger(), "Either `device_id` or `can_id_list` is not specified");
            }
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<KeyaNode>();
    auto node = std::make_shared<rclcpp::Node>("keya_driver");

    std::string device_id = "can0";
    std::vector<int64_t> can_id_list_int = {1};
    std::vector<canid_t> can_id_list;

    node->declare_parameter("device_id", device_id);
    node->declare_parameter("can_id_list_int", can_id_list_int);

    if(!(node->get_parameter("device_id", device_id)) || !(node->get_parameter("can_id_list_int", can_id_list_int)))
    // if(!(node->get_parameter("device_id", device_id)))
    {
        RCLCPP_FATAL(node->get_logger(),"Either `device_id` or `can_id_list` parameter is not specified");
        return 0;
    }

    for (int can_id_int : can_id_list_int)
    {
        can_id_list.push_back(static_cast<canid_t>(0x86000000 + can_id_int));
        std::cout << "can_id: " << can_id_list[0] << std::endl;
    }

    rclcpp::Time now = node->get_clock()->now();
    rclcpp::Time prev_time = now;
    rclcpp::Rate rate(20);

    std::mutex mtx;


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}