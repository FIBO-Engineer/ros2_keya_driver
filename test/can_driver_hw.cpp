#include <ros2_keya_driver/can_driver_hw.hpp>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include <algorithm>
#include <chrono>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <vector>

namespace keya_driver_hardware_interface
{
    CANDriverHW::CANDriverHW(std::string _device_id, std::vector<canid_t> _can_id_list) : device_id(_device_id), can_id_list(_can_id_list), stream(io_context)
    {
        connect();

        if (can_id_list.size() != 1)
        {
            throw std::runtime_error("This program only supports one steering wheel: " + std::to_string(_can_id_list.size()));
        }

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"), "[Constructor] Initializing Motor...");

    }

    CANDriverHW::~CANDriverHW()
    {
        if (!stream.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"), "Stream was already closed.");
        }
        disconnect();
        std::cout << "[~keya_motor] Motor drive disconnected" << std::endl;
    }

    void CANDriverHW::connect()
    {
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
        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"), "CAN socket connected");
    }

    void CANDriverHW::disconnect()
    {
        if (stream.is_open())
        {
            stream.close();
        }
    }

    
    // hardware_interface::CallbackReturn CANDriverHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    // {

    //     return CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn CANDriverHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    // {

    //     return CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn CANDriverHW::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
    // {

    //     CANDriverHW::disconnect();

    //     return CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn CANDriverHW::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    // {

    //     return CallbackReturn::SUCCESS;
    // }

    hardware_interface::CallbackReturn CANDriverHW::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CANDriverHW"),"Initialization failed...");
            return CallbackReturn::ERROR;
        }

        cfg_.device_id = info_.hardware_parameters["device_id"];

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Connecting CAN...");

        CANDriverHW::connect();

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"CAN Connected.");

        return CallbackReturn::SUCCESS;
    
    }

    std::vector<hardware_interface::StateInterface> CANDriverHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interface;


        return state_interface;
    }

    std::vector<hardware_interface::CommandInterface> CANDriverHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interface;


        return command_interface;
    }

    hardware_interface::CallbackReturn CANDriverHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Enabling Motor Control...");

        std::vector<bool> res;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_enable_frame = codec.encode_position_control_enable_request(can_id_list[i]);
            can_write(position_control_enable_frame, std::chrono::milliseconds(200));
            can_read(std::chrono::milliseconds(200));
            res.push_back(codec.decode_command_response(input_buffer));
            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Motor Control Enabled.");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CANDriverHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Disabling Motor Control...");

        std::vector<bool> res;
        for (std::vector<unsigned int>::size_type i = 0; i < can_id_list.size(); i++)
        {
            can_frame position_control_disable_frame = codec.encode_position_control_disable_request(can_id_list[i]);
            can_write(position_control_disable_frame, std::chrono::milliseconds(200));
            can_read(std::chrono::milliseconds(200));
            res.push_back(codec.decode_command_response(input_buffer));
            clear_buffer(input_buffer);
        }

        RCLCPP_INFO(rclcpp::get_logger("CANDriverHW"),"Motor Control Disabled.");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CANDriverHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
                RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"), "CAN socket is not opened yet: read");
                throw std::runtime_error("CAN socket is not opened yet: read");
                
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CANDriverHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!stream.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"),"CAN socket is not opened yet: write");
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

    void CANDriverHW::can_read(std::chrono::steady_clock::duration timeout)
    {
        boost::system::error_code error;
        boost::asio::async_read(stream,
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
        
    void CANDriverHW::can_write(can_frame &message, std::chrono::steady_clock::duration timeout)
    {
        boost::system::error_code error;
        boost::asio::async_write(stream, boost::asio::buffer(&message, sizeof(message)),
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

    void CANDriverHW::clear_buffer(can_frame &input_buffer)
    {
        for (int i = 0; i < input_buffer.can_dlc; i++)
        {
            input_buffer.data[i] = 0x00;
        }
    }

    void CANDriverHW::run(std::chrono::steady_clock::duration timeout)
    {
        io_context.restart();
        io_context.run_for(timeout);
        if (!io_context.stopped())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CANDriverHW"),"Operation Timeout, probably due to no data return from the devicde.");
            stream.close();
            io_context.run();
        }
    }

}