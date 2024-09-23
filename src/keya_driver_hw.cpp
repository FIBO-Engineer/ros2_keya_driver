#include <ros2_keya_driver/keya_driver_hw.hpp>
#include <ros2_keya_driver/keya_codec.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include "transmission_interface/simple_transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/u_int16.hpp>

#include "diagnostic_updater/diagnostic_updater.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>

#include <iostream>

#include <fstream>
#include <nlohmann/json.hpp>


namespace keya_driver_hardware_interface
{
    using namespace std::chrono_literals;
    using json = nlohmann::json;

    hardware_interface::CallbackReturn KeyaDriverHW::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),"Initialization failed...");
            return CallbackReturn::ERROR;
        }

        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        if(info_.joints.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),"This program supports only one actuator");
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::vector<double> pos_idx;

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),
                    "Joint '%s' has %zu state interface. 1 expected", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            int pos_if_idx = -1;
            for(size_t i = 0; i < joint.state_interfaces.size(); i++){
                if(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION){
                    pos_if_idx = i;
                } else {
                    RCLCPP_FATAL(
                        rclcpp::get_logger("KeyaDriverHW"),
                        "Joint '%s' has unsupported interface: %s", joint.name.c_str(),
                        info.name.c_str());
                    return CallbackReturn::ERROR;
                }
            }

            if(pos_if_idx == -1){
                RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"), "State Interface in URDF must provide position interface");
                return CallbackReturn::ERROR;
            }

            pos_idx.push_back(pos_if_idx);

        }

        transmission_interface::SimpleTransmissionLoader transmission_loader;
        // a_cmd_pos.resize(info_.transmissions.size());

        for(const auto & transmission_info : info_.transmissions)
        {
            if (transmission_info.type != "transmission_interface/SimpleTransmission")
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("KeyaDriverHW"), "Transmission '%s' of type '%s' not supported  in here",
                    transmission_info.name.c_str(), transmission_info.type.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            std::shared_ptr<transmission_interface::Transmission> state_transmission;
            std::shared_ptr<transmission_interface::Transmission> command_transmission;

            try
            {
                state_transmission = transmission_loader.load(transmission_info);
                command_transmission = transmission_loader.load(transmission_info);
            }
            catch (const transmission_interface::TransmissionInterfaceException & exc)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("KeyaDriverHW"),"Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // std::vector<transmission_interface::JointHandle> joint_handles;
            for (size_t i = 0; i < transmission_info.joints.size(); i++)
            {
                if (!(transmission_info.joints[i].state_interfaces.size() == 1 &&
                        transmission_info.joints[i].state_interfaces[pos_idx[i]] == hardware_interface::HW_IF_POSITION &&
                        transmission_info.joints[i].command_interfaces.size() == 1 &&
                        transmission_info.joints[i].command_interfaces[0] == hardware_interface::HW_IF_POSITION))
                {
                    RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"), "Invalid transmission joint '%s' configuration for this demo",
                        transmission_info.joints[i].name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                transmission_interface::JointHandle joint_handle_pos(transmission_info.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
                state_joint_handles.push_back(joint_handle_pos);

                transmission_interface::JointHandle joint_handle_cmd_pos(transmission_info.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
                command_joint_handles.push_back(joint_handle_cmd_pos);
            }

            // std::vector<transmission_interface::ActuatorHandle> actuator_handles;
            for (size_t i = 0; i < transmission_info.actuators.size(); i++)
            {
                transmission_interface::ActuatorHandle actuator_handle_pos(
                    transmission_info.actuators[i].name, hardware_interface::HW_IF_POSITION, &a_pos[i]
                );
                state_actuator_handles.push_back(actuator_handle_pos);

                transmission_interface::ActuatorHandle actuator_handle_cmd_pos(
                    transmission_info.actuators[i].name, hardware_interface::HW_IF_POSITION, &a_cmd_pos[i]
                );
                command_actuator_handles.push_back(actuator_handle_cmd_pos);
            }

            try
            {
                state_transmission->configure(state_joint_handles, state_actuator_handles);
                command_transmission->configure(command_joint_handles, command_actuator_handles);
            }
            catch(const transmission_interface::TransmissionInterfaceException & exc)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("KeyaDriverHW"), "Error while configuring %s: %s", transmission_info.name.c_str(), exc.what());
                return hardware_interface::CallbackReturn::ERROR;
            }

            state_transmissions.push_back(state_transmission);
            command_transmissions.push_back(command_transmission);        
        }

        hw_states_.resize(1);
        hw_commands_.resize(1);

        return hardware_interface::CallbackReturn::SUCCESS;
    
    }

    void KeyaDriverHW::produce_diagnostics_0(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        uint16_t _alarm_code;
        keya_driver_hardware_interface::ErrorSignal _error_signal_0;
        read_mtx.lock();
        _alarm_code = this->alarm_code;
        _error_signal_0 = this->error_signal_0;
        read_mtx.unlock();

        if (_alarm_code)
        {
            stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Alarm Code 0: %f", _alarm_code);
        }
        else
        {
            stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        }

        stat.add("Less Phase", _error_signal_0.LSPHS);
        stat.add("Motor Stall", _error_signal_0.MOTSTALL);
        stat.add("Reserved", _error_signal_0.RESERVED);
        stat.add("Hall Failer", _error_signal_0.HALLFAIL);
        stat.add("Current Sensing", _error_signal_0.CURRSENSE);
        stat.add("232 Disconnected", _error_signal_0.TTTDISC);
        stat.add("CAN Disconnected", _error_signal_0.CANDISC);
        stat.add("Motor Stalled", _error_signal_0.MOTSTALLED);   

    }

    void KeyaDriverHW::produce_diagnostics_1(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        uint16_t _alarm_code;
        keya_driver_hardware_interface::ErrorSignal1 _error_signal_1;
        read_mtx.lock();
        _alarm_code = this->alarm_code;
        _error_signal_1 = this->error_signal_1;
        read_mtx.unlock();

        if (_alarm_code)
        {
            stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Alarm Code 1: %f", _alarm_code);
        }
        else
        {
            stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        }

        stat.add("Disabled", _error_signal_1.DISABLE);
        stat.add("Overvoltage", _error_signal_1.OVRVOLT);
        stat.add("Hardware Protect", _error_signal_1.HRDWRPROT);
        stat.add("E2PROM", _error_signal_1.EEPROM);
        stat.add("Undervoltage", _error_signal_1.UNDRVOLT);
        stat.add("N/A", _error_signal_1.NA);
        stat.add("Overcurrent", _error_signal_1.OVRCURR);
        stat.add("Mode Failure", _error_signal_1.MODEFAIL);
        
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Configuring...");

        // device_id = "can0";

        sockaddr_can addr;
        ifreq ifr;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        strcpy(ifr.ifr_name, device_id.c_str());
        ioctl(natsock, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Error while binding CAN socket");
            throw std::runtime_error("Error while binding CAN socket");
        }

        // Set up CAN filters
        struct can_filter rfilter[1];
        rfilter[0].can_id = 0x05800001;
        rfilter[0].can_mask = CAN_SFF_MASK;

        setsockopt(natsock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));


        stream = std::make_shared<boost::asio::posix::basic_stream_descriptor<>>(io_context);

        stream->assign(natsock);

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "CAN socket connected");

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Configuration successful");

        // Always reset values when configuring hardware
        for (uint i = 0; i < hw_states_.size(); i++)
        {
            hw_states_[i] = 0;
            hw_commands_[i] = 0;
        }

        node = rclcpp::Node::make_shared("keya_driver_node");

        auto update_func = [this]()
        {
            diagnostic_updater = std::make_shared<diagnostic_updater::Updater>(node, 1.0);
            diagnostic_updater->setHardwareID("Keya-KY170G");
            diagnostic_updater->add("Hardware Status", this, &KeyaDriverHW::produce_diagnostics_0);
            diagnostic_updater->add("Hardware Status", this, &KeyaDriverHW::produce_diagnostics_1);

            // centering_service = node->create_service<std_srvs::srv::Trigger>("center", std::bind(&KeyaDriverHW::centering_callback, this,std::placeholders::_1, std::placeholders::_2));
            // centering_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

            mode_subscriber = node->create_subscription<std_msgs::msg::Bool>("/analog", 10, std::bind(&KeyaDriverHW::modeswitch_callback, this, std::placeholders::_1));
            center_subscriber = node->create_subscription<std_msgs::msg::Bool>("/center", 10, std::bind(&KeyaDriverHW::centering_callback, this, std::placeholders::_1));

            rclcpp::spin(node);
            rclcpp::shutdown();
        };

        rcl_thread = std::thread(update_func);
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Configuration successful");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Cleaning up...");

        std::cout << "[KeyaDriverHW] Cleaning up..." << std::endl;

        if (stream->is_open())
        {
            stream->close();
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Clean up successful");

        std::cout << "[KeyaDriverHW] Clean up successful" << std::endl; 
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> KeyaDriverHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interface;
        for (unsigned int i = 0; i < info_.joints.size(); i++)
        {
            state_interface.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        }

        return state_interface;
    }

    std::vector<hardware_interface::CommandInterface> KeyaDriverHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interface;
        for (unsigned int i = 0; i < info_.joints.size(); i++)
        {
            command_interface.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interface;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Enabling Motor Control...");

        for (uint i = 0; i < hw_states_.size(); i++)
        {
            hw_commands_[i] = hw_states_[i];
        }

        // std::vector<bool> res;

        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_enable_frame = codec.encode_position_control_enable_request(can_id_list[i]);           
            can_write(position_control_enable_frame, std::chrono::milliseconds(100));
            // RCLCPP_INFO(rclcpp::get_logger("READ CAN"),"READ CAN.");
            can_read(std::chrono::milliseconds(100));
            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Enabled.");

        /* -------------------------- Homing -----------------------------*/

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Homing Initialized...");
        
        homing_state = OperationState::DOING;

        /*--------------------------End Homing----------------------------*/

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Deactivating Motor Control...");

        std::cout << "[KeyaDriverHW]: Deactivating Motor Control..." << std::endl;

        std::vector<bool> res;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_disable_frame = codec.encode_position_control_disable_request(can_id_list[i]);
            can_write(position_control_disable_frame, std::chrono::milliseconds(100));
            can_read(std::chrono::milliseconds(100));
            res.push_back(codec.decode_command_response(input_buffer));
            clear_buffer(input_buffer);
        }

        std::cout << "[KeyaDriverHW]: Motor Control Deactivated." << std::endl;

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Deactivated.");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        /*Implement on_shutdown method where hardware is shutdown gracefully.*/

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Shutting Down Motor Control...");

        std::cout << "[KeyaDriverHW]: Shutting Down Motor Control..." << std::endl;

        rcl_thread.~thread();

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Shutdown.");

        std::cout << "[KeyaDriverHW]: Motor Control Shutdown." << std::endl;

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KeyaDriverHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (stream->is_open())
        {
            MessageType mt;
            int read_count = 0;
            do {
                clear_buffer(input_buffer);
                can_read(std::chrono::milliseconds(100));
                mt = codec.getResponseType(input_buffer);
                if(++read_count >= 5) {
                    RCLCPP_DEBUG(rclcpp::get_logger("READ"), "Heartbeat message was not found for five consequtive frames");
                    return hardware_interface::return_type::ERROR;
                }
            } while(mt != MessageType::HEARTBEAT);

            const std::lock_guard<std::mutex> lock(read_mtx);
            // Read Diagnostic Message
            error_signal_0 = codec.decode_error_0_response(input_buffer);
            RCLCPP_DEBUG(rclcpp::get_logger("Error0_Debug"), "Error0: %s", error_signal_0.getErrorMessage().c_str());
            error_signal_1 = codec.decode_error_1_response(input_buffer);
            RCLCPP_DEBUG(rclcpp::get_logger("Error1_Debug"), "Error1: %s", error_signal_1.getErrorMessage().c_str());

            // Read Motor Current
            current_current = codec.decode_current_response(input_buffer);// = codec.decode_current_response(current_response);
            RCLCPP_DEBUG(rclcpp::get_logger("READ"), "current*: %f", current_current);

            // Read Motor Position
            current_position = codec.decode_position_response(input_buffer) + pos_offset;
            a_pos[0] = current_position;
            state_transmissions[0]->actuator_to_joint();

            clear_buffer(input_buffer);
            return hardware_interface::return_type::OK;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"), "CAN socket is not opened yet: read");
            throw std::runtime_error("CAN socket is not opened yet: read");
            return hardware_interface::return_type::ERROR;
        }
    
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KeyaDriverHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!stream->is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"),"CAN socket is not opened yet: write");
            throw std::runtime_error("CAN socket is not opened yet: write");
            return hardware_interface::return_type::ERROR;
        }

        can_frame cmd_frame;
        if(mode_change)
        {
            cmd_frame = curr_mode ? codec.encode_position_control_disable_request(can_id_list[0]) 
                                    : codec.encode_position_control_enable_request(can_id_list[0]);
            mode_change = false;
        } else if(homing_state == OperationState::DOING)
        {
            cmd_frame = codec.encode_position_command_request(can_id_list[0], -1.0);
            const std::lock_guard<std::mutex> lock(read_mtx);
            if(std::fabs(current_current) > CURRENT_THRESHOLD)
            {
                pos_offset = 0.5 + current_position; 
                homing_state = OperationState::DONE;
                centering_state = OperationState::DOING;
            }
            
        } else if(centering_state == OperationState::DOING)
        {
            cmd_frame = codec.encode_position_command_request(can_id_list[0], 0.00);
            if(std::fabs(current_position) < POSITION_TOLERANCE)
            {
                centering_state = OperationState::DONE;
            }
        } else
        {
            double enc_pos = hw_commands_[0]; 
            command_transmissions[0]->joint_to_actuator();
            a_cmd_pos[0] = enc_pos; // - pos_offset;
            cmd_frame = codec.encode_position_command_request(can_id_list[0], a_cmd_pos[0] - pos_offset);
        }
        can_write(cmd_frame, std::chrono::milliseconds(100));
        
        return hardware_interface::return_type::OK;
    }    

    void KeyaDriverHW::can_read(std::chrono::steady_clock::duration timeout)
    {
        boost::system::error_code error;
        boost::asio::async_read(*stream,
                                boost::asio::buffer(&input_buffer, sizeof(input_buffer)),
                                [&](const boost::system::error_code &res_error, std::size_t)
                                {
                                    error = res_error;
                                });

        run(timeout);
        if (error)
        {
            throw std::system_error(error);
        }
    }
        
    void KeyaDriverHW::can_write(can_frame &message, std::chrono::steady_clock::duration timeout)
    {
        boost::system::error_code error;
        boost::asio::async_write(*stream, boost::asio::buffer(&message, sizeof(message)),
                                 [&](const boost::system::error_code &res_error, std::size_t)
                                 {
                                     error = res_error;
                                 });
        run(timeout);
        if (error)
        {
            throw std::system_error(error);
        }
    }

    void KeyaDriverHW::clear_buffer(can_frame &input_buffer)
    {
        for (int i = 0; i < CAN_MAX_DLC; i++)
        {
            input_buffer.data[i] = 0x00;
            input_buffer.can_id = 0x00000000;
        }
    }

    void KeyaDriverHW::run(std::chrono::steady_clock::duration timeout)
    {
        io_context.restart();
        io_context.run_for(timeout);
        if (!io_context.stopped())
        {
            RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"),"Operation Timeout, probably due to no data return from the device.");
            stream->close();
            io_context.run();
        }
    }

    void KeyaDriverHW::centering_callback(const std_msgs::msg::Bool income_center)
    {
        if(income_center.data == true)
        {
            is_centering = true;
        }
    }

    void KeyaDriverHW::modeswitch_callback(const std_msgs::msg::Bool income_mode)
    {
        if(curr_mode != income_mode.data)
        {
            curr_mode = income_mode.data;
            mode_change = true;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(keya_driver_hardware_interface::KeyaDriverHW, hardware_interface::ActuatorInterface)
