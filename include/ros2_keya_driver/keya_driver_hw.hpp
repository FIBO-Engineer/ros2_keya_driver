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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "transmission_interface/transmission.hpp"

#include <fstream>
#include <atomic>

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

        double a_pos[1];
        double a_cmd_pos[1];

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
        int natsock;

        std::vector<double> hw_commands_;
        double clamped_cmd;
        std::vector<double> hw_states_;

        std::vector<std::shared_ptr<transmission_interface::Transmission>> state_transmissions;
        std::vector<std::shared_ptr<transmission_interface::Transmission>> command_transmissions;

        std::vector<transmission_interface::JointHandle> state_joint_handles;
        std::vector<transmission_interface::ActuatorHandle> state_actuator_handles;
        std::vector<transmission_interface::JointHandle> command_joint_handles;
        std::vector<transmission_interface::ActuatorHandle> command_actuator_handles;

        std::shared_ptr<boost::asio::posix::basic_stream_descriptor<>> stream;
        // boost::asio::posix::basic_stream_descriptor<> stream;
        can_frame input_buffer;

        // lock for all read object
        std::mutex read_mtx;

        // diagnostic
        rclcpp::Node::SharedPtr node;
        std::thread rcl_thread;
        void produce_diagnostics_0(diagnostic_updater::DiagnosticStatusWrapper &stat);
        void produce_diagnostics_1(diagnostic_updater::DiagnosticStatusWrapper &stat);
        std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater;

        // Homing Service
        // void homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        //                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void centering_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        
        // mode switching
        void analog_mode_callback(const std::shared_ptr<std_msgs::msg::Bool> _mode);
        void manual_homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> reponse);

        // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr centering_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_homing_service;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr analog_mode_subscriber;
        realtime_tools::RealtimeBuffer<std_msgs::msg::Bool> analog_mode;
        
        double current_position_unoffset;
        double current_current;
        ErrorSignal error_signal_0;
        ErrorSignal1 error_signal_1;
        uint16_t alarm_code;

        double min_raw_position;

        std::atomic<double> pos_offset;

        enum OperationState: uint8_t {IDLE = 0, DONE = 1, DOING = 2, FAILED = 3};

        std::atomic<OperationState> homing_state;
        std::atomic<OperationState> centering_state;

        std::mutex centering_mtx;
        std::condition_variable centering_cv;
        
        double min;
        double max;

        static constexpr double max_wheel_right = -25.0;
        static constexpr double max_wheel_left = 25.0;

        static constexpr double CENTER_TO_RIGHT_DIST = -10.00;  //0.512 * 22.5
        static constexpr double CENTER_TO_LEFT_DIST = 11.20; //0.498 * 22.5
        static constexpr double CURRENT_THRESHOLD = 18.0;
        static constexpr double POSITION_TOLERANCE = 0.002; //0.5;
    };
}

#endif
