#include <ros2_keya_driver/keya_driver_hw.hpp>
#include <ros2_keya_driver/keya_codec.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include "transmission_interface/simple_transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include <std_srvs/srv/trigger.hpp>

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

                transmission_interface::JointHandle joint_handle_pos(transmission_info.joints[i].name, hardware_interface::HW_IF_POSITION, &j_pos[i]);
                state_joint_handles.push_back(joint_handle_pos);

                transmission_interface::JointHandle joint_handle_cmd_pos(transmission_info.joints[i].name, hardware_interface::HW_IF_POSITION, &j_cmd_pos[i]);
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

        // try
        // {
        //     std::ifstream f("/home/yamaha02/ros2_ws/src/ros2_keya_driver/config/offset.json");
        //     json data = json::parse(f);

        //     pos_offset = data["offset"];
        //     f.close();
        //     RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Offset file found.");
        // }
        // catch(std::runtime_error &e)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Offset file not found.");
        // }

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

            homing_service = node->create_service<std_srvs::srv::Trigger>("home", std::bind(&KeyaDriverHW::homing_callback, this,std::placeholders::_1, std::placeholders::_2));
            homing_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

            centering_service = node->create_service<std_srvs::srv::Trigger>("center", std::bind(&KeyaDriverHW::centering_callback, this,std::placeholders::_1, std::placeholders::_2));
            centering_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

            // init_center_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 1);

            rclcpp::spin(node);
            rclcpp::shutdown();
        };

        rcl_thread = std::thread(update_func);
        // rcl_thread_2 = std::thread();

        // homing_service = node->create_service<std_srvs::srv::Trigger>("home", std::bind(&KeyaDriverHW::homing_callback, this, std::placeholders::_1, std::placeholders::_2));

        // homing_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

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
            can_write(position_control_enable_frame, std::chrono::milliseconds(200));
            // RCLCPP_INFO(rclcpp::get_logger("READ CAN"),"READ CAN.");
            for (int k = 0; k < 5; k++)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                can_read(std::chrono::milliseconds(200));

                try
                {
                    if(codec.decode_command_response(input_buffer))
                    {
                        break;
                    }
                    else
                    {
                        throw 505;
                    }

                    clear_buffer(input_buffer);

                }
                catch(int myNum)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("enable_decode_logger"), "%d", myNum);
                }

                // if (codec.decode_command_response(input_buffer))
                // {
                //     break;
                // }
                // else if (!codec.decode_command_response(input_buffer) && k==4)
                // {
                //     RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"),"Cannot enable motor");
                //     return hardware_interface::CallbackReturn::ERROR;
                // }
            }
            // can_read(std::chrono::milliseconds(200));
            // std::cout << "decode_command_response: " << codec.decode_command_response(input_buffer) << std::endl;

            // if(!codec.decode_command_response(input_buffer))
            // {
            //     RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"),"Cannot enable motor");

            //     return hardware_interface::CallbackReturn::ERROR;
            // }
            

            // res.push_back(codec.decode_command_response(input_buffer));

            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Enabled.");

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
            can_write(position_control_disable_frame, std::chrono::milliseconds(200));
            can_read(std::chrono::milliseconds(200));
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

        // rclcpp_lifecycle::State test_state;

        rcl_thread.~thread();

        // on_deactivate(test_state);
        // on_cleanup(test_state);

        // // std::vector<bool> res;
        // for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        // {
        //     can_frame position_control_disable_frame = codec.encode_position_control_disable_request(can_id_list[i]);
        //     can_write(position_control_disable_frame, std::chrono::milliseconds(200));
        //     can_read(std::chrono::milliseconds(200));
        //     // res.push_back(codec.decode_command_response(input_buffer));
        //     clear_buffer(input_buffer);
        // }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Shutdown.");

        std::cout << "[KeyaDriverHW]: Motor Control Shutdown." << std::endl;

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KeyaDriverHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // double sleep_time = 0.001;
        // double sleep_time = 5;

        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            // can_frame req_curr_frame = codec.encode_current_request(can_id_list[i]);

            if (stream->is_open())
            {
                /* ---------------------------------------------------------------------------- */
                /* READ Error DATA0 and DATA1 */
                // std::cout << "-------------------------------------------------------" << std::endl;

                can_read(std::chrono::milliseconds(100));

                if(codec.decode_command_response(input_buffer))
                {
                    error_signal_0 = codec.decode_error_0_response(input_buffer);
                    RCLCPP_DEBUG(rclcpp::get_logger("Error0_Debug"), "Error0: %s", error_signal_0.getErrorMessage().c_str());
                    error_signal_1 = codec.decode_error_1_response(input_buffer);
                    RCLCPP_DEBUG(rclcpp::get_logger("Error1_Debug"), "Error1: %s", error_signal_1.getErrorMessage().c_str());
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("Error0_Debug"), "Error0: Cannot read error");
                }

                /* ---------------------------------------------------------------------------- */
                /* READ motor current */

                // can_write(req_curr_frame, std::chrono::milliseconds(100)); // Write a current read request
                can_read(std::chrono::milliseconds(100));
                can_frame current_response = input_buffer;

                try
                {
                    if(codec.decode_command_response(current_response))
                    {
                        current_current.store(codec.decode_current_response(current_response));// = codec.decode_current_response(current_response);
                        RCLCPP_DEBUG(rclcpp::get_logger("READ"), "current*: %f", current_current.load());
                        clear_buffer(input_buffer);
                    }
                    else
                    {
                        throw 505;
                    }

                    // return hardware_interface::return_type::OK;
                }

                catch (int myNum)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("curr_decode_logger"), "%d", myNum);
                }

                /* ---------------------------------------------------------------------------- */
                /* READ Current Position */

                can_read(std::chrono::milliseconds(100));
                    
                can_frame position_response = input_buffer;
                try
                {
                    // read_mtx.lock();

                    // error_signal = codec.decode_error_response(input_buffer);
                    if(codec.decode_command_response(position_response)){
                        const std::lock_guard<std::mutex> lock(rawpos_reading_mutex);
                        raw_position = codec.decode_position_response(position_response) + pos_offset;

                        // RCLCPP_INFO(rclcpp::get_logger("OFFSET_IN_READ"), "Offset in Read: %f", pos_offset);
                        // RCLCPP_INFO(rclcpp::get_logger("RAW_IN_READ"), "Raw in Read: %f", raw_position);

                        current_position = raw_position; // + pos_offset;

                        // RCLCPP_INFO(rclcpp::get_logger("CURRENTPOS_IN_READ"), "Current pos in Read: %f", current_position);

                        a_pos[i] = current_position;

                        state_transmissions[i]->actuator_to_joint();
                        hw_states_[0] = current_position;

                        // current_current = codec.decode_current_response(input_buffer);

                        // read_mtx.unlock();

                        clear_buffer(input_buffer);

                        return hardware_interface::return_type::OK;
                    }
                    else
                    {
                        throw 505;
                    }
                }

                // catch (std::runtime_error &e)
                catch (int myNum)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("pos_decode_logger"), "%d", myNum);
                }

                return hardware_interface::return_type::ERROR;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"), "CAN socket is not opened yet: read");
                throw std::runtime_error("CAN socket is not opened yet: read");
                
                return hardware_interface::return_type::ERROR;
            }
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

        can_frame req_pos_cmd;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            // RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "hw_command_[0]: %f", hw_commands_[0]);

            double enc_pos = hw_commands_[0]; 
            command_transmissions[i] -> joint_to_actuator();
            // RCLCPP_INFO(rclcpp::get_logger("KeyaCodec"),"hw_cmd: %f", hw_commands_[0]);
    
            a_cmd_pos[i] = enc_pos; // - pos_offset;
            
            req_pos_cmd = codec.encode_position_command_request(can_id_list[i], a_cmd_pos[i] - pos_offset);

            // RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Commanded position: %f", a_cmd_pos[i]);

            can_write(req_pos_cmd, std::chrono::milliseconds(100));
            can_read(std::chrono::milliseconds(100));

            // if (!codec.decode_position_command_response(input_buffer))
            // {
            //     RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"), "Cannot request position command");

            //     return hardware_interface::return_type::ERROR;
            // }
        }
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

    // void KeyaDriverHW::homing()
    // {
    //     double current_threshold = 10;
    //     // turning wheel all the way to the left
    //     publisher_ = node->create_publisher<std_msgs::msg::Float64>("/position_controllers/command", 10);

    // }

    // void KeyaDriverHW::set_offset(double input_pos)
    // {
    //     pos_set = 10;
    //     pos_offset = pos_set - input_pos;        
    // }

    double KeyaDriverHW::set_offset()
    {
        const std::lock_guard<std::mutex> lock(rawpos_reading_mutex);
        // pos_set = 10;
        RCLCPP_INFO(rclcpp::get_logger("RAWPOS_LOGGER"), "raw_pos: %f", raw_position);
        pos_offset = 0.5 - raw_position; 

        // create json file to save pos_offset
        json j;
        j["offset"] = pos_offset;

        std::ofstream file("/home/yamaha02/ros2_ws/src/ros2_keya_driver/config/offset.json");
        file << j;

        file.close(); 

        // RCLCPP_INFO(rclcpp::get_logger("POS_OFFSET"), "Pos_offset: %f", pos_offset);
        return pos_offset;
    }

    void KeyaDriverHW::homing_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {

        response->success = true;
        response->message = "";

        RCLCPP_INFO(rclcpp::get_logger("HOMING_LOG"), "Homing Initialized...");

        current_threshold = 17;
        std_msgs::msg::Float64MultiArray turn_left;
        turn_left.data.resize(1);
        turn_left.data[0] = 25.0;

        // turn wheel to the left
        while(!reach_current_threshold(current_threshold))
        {
            RCLCPP_INFO(rclcpp::get_logger("CURRENT_CURRENT_LOGGER"), "Current_current: %f", current_current.load());
            homing_publisher->publish(turn_left);
            std::this_thread::sleep_for(50ms);
        }

        set_offset();
        RCLCPP_INFO(rclcpp::get_logger("POS_OFFSET_OUT"), "Pos_offset: %f", pos_offset);
        RCLCPP_INFO(rclcpp::get_logger("CURRENT_POS_LOGGER"), "Current_POS: %f", current_position);
        std_msgs::msg::Float64MultiArray turn_right;
        turn_right.data.resize(1);
        turn_right.data[0] = 0.000000;
        homing_publisher->publish(turn_right);

    }

    void KeyaDriverHW::centering_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        response->success = true;
        response->message = "";

        RCLCPP_INFO(rclcpp::get_logger("CENTERING_LOG"), "Centering Initialized...");

        std_msgs::msg::Float64MultiArray center;
        center.data.resize(1);
        // center.data[0] = -pos_offset;
        center.data[0] = 0.00;
        centering_publisher->publish(center);
    }

    // void KeyaDriverHW::handle_service()
    // {
    //     homing_service = node->create_service<std_srvs::srv::Trigger>("home", std::bind(&KeyaDriverHW::homing_callback, this,std::placeholders::_1, std::placeholders::_2));

    //     homing_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    //     rclcpp::spin(node);
    // }

    bool KeyaDriverHW::reach_current_threshold(double current_threshold)
    {
        // const std::lock_guard<std::mutex> lock(current_reading_mutex);
        if (fabs(current_current.load()) >= current_threshold){
            RCLCPP_INFO(rclcpp::get_logger("THRESHOLD_LOGGER"), "THRESHOLD REACHED");
            return true;
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("THRESHOLD_LOGGER"), "FALSE %f %f" , fabs(current_current), fabs(current_threshold));
            return false;

        }
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(keya_driver_hardware_interface::KeyaDriverHW, hardware_interface::ActuatorInterface)
