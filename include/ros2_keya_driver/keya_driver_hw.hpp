#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_DRIVER_HW_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_DRIVER_HW_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/sensor_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <ros2_keya_driver/keya_codec.hpp>

#include <boost/asio.hpp>

#include <controller_manager/controller_manager.hpp>

#include <vector>

namespace keya_driver_hardware_interface
{
    class KeyaDriverHW : public hardware_interface::ActuatorInterface
    {
    public:
        KeyaDriverHW();
        virtual ~KeyaDriverHW();

        virtual void connect() = 0;
        virtual void disconnect() = 0;

        void reconnect();

        virtual void attachControllerManager(std::shared_ptr<controller_manager::ControllerManager> cm) = 0;

        double a_cmd_pos[1];
        double a_curr_pos[1];

    protected:
        virtual void run(std::chrono::steady_clock::duration timeout) = 0;

        KeyaCodec codec;

        boost::asio::io_context io_context;

    };
}

#endif
