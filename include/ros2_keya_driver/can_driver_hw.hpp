#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__CAN_DRIVER_HW_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__CAN_DRIVER_HW_HPP_

#include <ros2_keya_driver/keya_driver_hw.hpp>
#include <boost/asio.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace keya_driver_hardware_interface
{
    class CANDriverHW : public KeyaDriverHW
    {
    public:
        CANDriverHW(std::string _device_id, std::vector<canid_t> _can_id);
        ~CANDriverHW() override;

        void connect() override;
        void disconnect() override;

        // void attachControllerManager(std::shared_ptr<controller_manager::ControllerManager> cm) override;

        /* rclcpp::LifeCycleNodeInterface */
        virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override; 
        virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override; 

        /* hardware_interface::ActuatorInterface */
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        void can_read(std::chrono::steady_clock::duration timeout);
        void can_write(can_frame &message, std::chrono::steady_clock::duration timeout);
        void clear_buffer(can_frame &input_buffer);
        void run(std::chrono::steady_clock::duration timeout) override;

        // Input
        std::string device_id;
        std::vector<canid_t> can_id_list;

        boost::asio::posix::basic_stream_descriptor<> stream;
        can_frame input_buffer;
    
    };
}

#endif