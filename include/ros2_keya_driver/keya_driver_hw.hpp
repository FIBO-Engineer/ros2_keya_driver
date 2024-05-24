#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_DRIVER_HW_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_DRIVER_HW_HPP_

#include <hardware_interface/actuator_interface.hpp>

#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <ros2_keya_driver/keya_codec.hpp>

#include <boost/asio.hpp>
#include <boost/asio/posix/basic_stream_descriptor.hpp>

#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/macros.hpp"

#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

namespace keya_driver_hardware_interface
{
    class KeyaDriverHW : public hardware_interface::ActuatorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(KeyaDriverHW)
        // KeyaDriverHW(std::string _device_id, std::vector<canid_t> _can_id_list);
        // KeyaDriverHW() = delete;
        virtual ~KeyaDriverHW() override {
            rclcpp_lifecycle::State dummy_state;
            on_deactivate(dummy_state);
            on_cleanup(dummy_state);
            on_shutdown(dummy_state);
        }

        // virtual void connect() = 0;
        // virtual void disconnect() = 0;

        // void reconnect();

        /* rclcpp::LifeCycleNodeInterface */
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override; 
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        

        /* hardware_interface::ActuatorInterface */
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        double a_cmd_pos[1];
        double a_curr_pos[1];

    protected:

        void can_read(std::chrono::steady_clock::duration timeout);
        void can_write(can_frame &message, std::chrono::steady_clock::duration timeout);
        void clear_buffer(can_frame &input_buffer);
        
        void run(std::chrono::steady_clock::duration timeout);

        KeyaCodec codec;

        boost::asio::io_context io_context;

    private:
        std::string device_id = "can0";
        std::vector<canid_t> can_id_list = {0x86000001};
        // std::vector<canid_t> can_id_list;

        std::vector<double> hw_commands_;
        std::vector<double> hw_states_;

        std::shared_ptr<boost::asio::posix::basic_stream_descriptor<>> stream;
        // boost::asio::posix::basic_stream_descriptor<> stream;
        can_frame input_buffer;

        // raw object
        std::mutex read_mtx;

        // diagnostic
        rclcpp::Node::SharedPtr node;
        std::thread rcl_thread;
        void produce_diagnostics_0(diagnostic_updater::DiagnosticStatusWrapper &stat);
        void produce_diagnostics_1(diagnostic_updater::DiagnosticStatusWrapper &stat);
        std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater;

        // void homing();
        bool reach_current_threshold(double threshold);
        void handle_service();

        void homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // void set_offset(double input_pos);

        double set_offset();

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr homing_publisher;

        // products
        double raw_position;
        double current_position;
        double current_command;
        uint16_t alarm_code;
        ErrorSignal error_signal_0;
        ErrorSignal1 error_signal_1;
        double current_current;
        double current_threshold;
        double pos_offset = 0;
        double pos_set;
        // StatusSignal status_signal;

    };
}

#endif
