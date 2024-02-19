#include <ros2_keya_driver/keya_driver_hw.hpp>
#include <ros2_keya_driver/keya_codec.hpp>
// #include <ros2_keya_driver/can_driver_hw.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>

namespace keya_driver_hardware_interface
{
    // KeyaDriverHW::KeyaDriverHW(std::string _device_id, std::vector<canid_t> _can_id_list) : device_id(_device_id), can_id_list(_can_id_list), stream(io_context)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("constructor logger"), "KeyaDriverHW Constructor");
    // }

    // KeyaDriverHW::~KeyaDriverHW()
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("destructor logger"), "KeyaDriverHW Destructor");
    // }

    // void CANDriverHW::connect()
    // {
    //     sockaddr_can addr;
    //     ifreq ifr;

    //     int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //     strcpy(ifr.ifr_name, device_id.c_str());
    //     ioctl(natsock, SIOCGIFINDEX, &ifr);

    //     addr.can_family = AF_CAN;
    //     addr.can_ifindex = ifr.ifr_ifindex;
    //     if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    //     {
    //         perror("Error while binding CAN socket");
    //         throw std::runtime_error("Error while binding CAN socket");
    //     }

    //     stream.assign(natsock);
    //     RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"), "CAN socket connected");
    // }

    // void CANDriverHW::disconnect()
    // {
    //     if (stream.is_open())
    //     {
    //         stream.close();
    //     }
    // }

    // void KeyaDriverHW::reconnect()
    // {
    //     disconnect();
    //     connect();
    // }


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
            RCLCPP_FATAL(rclcpp::get_logger("KeyaDriverHW"),"This program supports on one actuator");
            return hardware_interface::CallbackReturn::ERROR;
        }

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
        }

        // cfg_.device_id = info_.hardware_parameters["device_id"];

        // RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Connecting CAN...");

        // connect();

        // RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"CAN Connected.");

        return hardware_interface::CallbackReturn::SUCCESS;
    
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Configuring...");

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

        stream.assign(natsock);
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "CAN socket connected");

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Configuration successful");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Cleaning up...");

        if (stream.is_open())
        {
            stream.close();
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"), "Clean up successful");
        
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

        std::vector<bool> res;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_enable_frame = codec.encode_position_control_enable_request(can_id_list[i]);
            can_write(position_control_enable_frame, std::chrono::milliseconds(200));
            can_read(std::chrono::milliseconds(200));
            res.push_back(codec.decode_command_response(input_buffer));
            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Enabled.");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KeyaDriverHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Disabling Motor Control...");

        std::vector<bool> res;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_disable_frame = codec.encode_position_control_disable_request(can_id_list[i]);
            can_write(position_control_disable_frame, std::chrono::milliseconds(200));
            can_read(std::chrono::milliseconds(200));
            res.push_back(codec.decode_command_response(input_buffer));
            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("KeyaDriverHW"),"Motor Control Disabled.");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KeyaDriverHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame req_ang_frame = codec.encode_position_request(can_id_list[i]);
            if (stream.is_open())
            {
                can_write(req_ang_frame, std::chrono::milliseconds(100)); // Write a position read request
                can_read(std::chrono::milliseconds(100));
                double delta_pos = codec.decode_position_response(input_buffer);
                clear_buffer(input_buffer);

                a_curr_pos[i] += delta_pos;

                return hardware_interface::return_type::OK;
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
        if (!stream.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("KeyaDriverHW"),"CAN socket is not opened yet: write");
            throw std::runtime_error("CAN socket is not opened yet: write");

            return hardware_interface::return_type::ERROR;
        }

        can_frame req_pos_cmd;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            req_pos_cmd = codec.encode_position_command_request(can_id_list[i], a_cmd_pos[i]);
            can_write(req_pos_cmd, std::chrono::milliseconds(100));
            can_read(std::chrono::milliseconds(100));

            if (!codec.decode_command_response(input_buffer))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"), "Cannot request position command");

                return hardware_interface::return_type::ERROR;
            }
            clear_buffer(input_buffer);
        }

        return hardware_interface::return_type::OK;
    }

    void KeyaDriverHW::can_read(std::chrono::steady_clock::duration /*timeout*/)
    {
        boost::system::error_code error;
        boost::asio::async_read(stream,
                                boost::asio::buffer(&input_buffer, sizeof(input_buffer)),
                                [&](const boost::system::error_code &res_error, std::size_t)
                                {
                                    error = res_error;
                                });

        // run(timeout);
        if (error)
        {
            throw std::system_error(error);
        }
    }
        
    void KeyaDriverHW::can_write(can_frame &message, std::chrono::steady_clock::duration /*timeout*/)
    {
        boost::system::error_code error;
        boost::asio::async_write(stream, boost::asio::buffer(&message, sizeof(message)),
                                 [&](const boost::system::error_code &res_error, std::size_t)
                                 {
                                     error = res_error;
                                 });
        // run(timeout);
        if (error)
        {
            throw std::system_error(error);
        }
    }

    void KeyaDriverHW::clear_buffer(can_frame &input_buffer)
    {
        for (int i = 0; i < input_buffer.can_dlc; i++)
        {
            input_buffer.data[i] = 0x00;
        }
    }

    // void KeyaDriverHW::run(std::chrono::steady_clock::duration timeout)
    // {
    //     io_context.restart();
    //     io_context.run_for(timeout);
    //     if (!io_context.stopped())
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"),"Operation Timeout, probably due to no data return from the devicde.");
    //         stream.close();
    //         io_context.run();
    //     }
    // }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(keya_driver_hardware_interface::KeyaDriverHW, hardware_interface::ActuatorInterface)
