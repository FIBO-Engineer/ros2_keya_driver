#include <ros2_keya_driver/keya_codec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <cstdint>
#include <iostream>
#include <math.h>
#include <typeinfo>

namespace keya_driver_hardware_interface
{
    KeyaCodec::KeyaCodec() {}

    MessageType KeyaCodec::getResponseType(const can_frame &input_buffer)
    {
        if(input_buffer.can_id == 0x85800001)
        {
            return MessageType::CMD_RESPONSE;
        }
        else if(input_buffer.can_id == 0x87000001)
        {
            return MessageType::HEARTBEAT;
        }
        else
        {
            return MessageType::UNKNOWN;
        }
    }

    can_frame KeyaCodec::encode_position_control_enable_request(canid_t can_id)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x23;
        frame.data[1] = 0x0D;
        frame.data[2] = 0x20;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return frame;
    }

    can_frame KeyaCodec::encode_position_control_disable_request(canid_t can_id)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x23;
        frame.data[1] = 0x0C;
        frame.data[2] = 0x20;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return frame;
    }

    can_frame KeyaCodec::encode_position_command_request(canid_t can_id, double cmd)
    {
        // int32_t cmd_unit = cmd * 10000 / 360;
        int32_t cmd_unit = cmd * 10000 / (2 * M_PI) ;
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x23;
        frame.data[1] = 0x02;
        frame.data[2] = 0x20;
        frame.data[3] = 0x01;
        frame.data[4] = *((uint8_t *)(&cmd_unit) + 1);
        frame.data[5] = *(uint8_t *)(&cmd_unit);
        frame.data[6] = *((uint8_t *)(&cmd_unit) + 3);
        frame.data[7] = *((uint8_t *)(&cmd_unit) + 2);
        // RCLCPP_INFO(rclcpp::get_logger("KeyaCodec"),"pos_cmd_req: %f %d %d %d %d %d", cmd, cmd_unit, frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        return frame;
    }

    can_frame KeyaCodec::encode_position_request(canid_t can_id)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x04;
        frame.data[2] = 0x21;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return frame;
    }

    can_frame KeyaCodec::encode_current_request(canid_t can_id)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x00;
        frame.data[2] = 0x21;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return frame;
    }

    can_frame KeyaCodec::encode_error_request(canid_t can_id)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x40;
        frame.data[1] = 0x12;
        frame.data[2] = 0x21;
        frame.data[3] = 0x01;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return frame;
    }

    bool KeyaCodec::decode_command_response(can_frame &input_buffer)
    {
        if (input_buffer.can_id == 0x87000001)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }

    // bool KeyaCodec::decode_position_command_response(can_frame &input_buffer)
    // {
    //     return true;
    // }

    double KeyaCodec::decode_position_response(can_frame &input_buffer)
    {
        double curr_position_rad;

        // ------------------This block checks first three bytes of the response code-----------------------------

        // if ( input_buffer.data[0] == 0x60 && input_buffer.data[1] == 0x04 && input_buffer.data[2] == 0x21)
        if( input_buffer.can_id == 0x87000001 )
        {
            // curr_position_rad;

            int16_t curr_position;
            *(uint8_t *)(&curr_position) = input_buffer.data[1];
            *((uint8_t *)(&curr_position) + 1) = input_buffer.data[0];
            // RCLCPP_INFO(rclcpp::get_logger("RAW_POSITION"), "Raw Position %d", curr_position);

            curr_position_rad = curr_position * ( M_PI / 180 );
            // curr_position_rad = curr_position_rad / 22.5;

            return curr_position_rad;

        }
        else if( input_buffer.can_id == 0x00000000)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CAN_ID_POSITION_LOGGER"), "Motor disconnected");
            return 0.0;
        }
        else
        {
            std::cout << "[CAN_ID_POSITION_LOGGER]: " << std::hex << input_buffer.can_id << std::dec << std::endl;
            // std::cout << "[INPUT_BUFFER_DLC]: " << std::hex << input_buffer.data[0] << std::dec << std::endl;
            for (int i = 0; i < input_buffer.can_dlc; ++i) {
                std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(input_buffer.data[i]) << " ";
            }
            std::cout << "" << std::endl;
            RCLCPP_ERROR(rclcpp::get_logger("position_logger"), "Cannot read position.");
            return 0.0;
        }

    }

    double KeyaCodec::decode_current_response(can_frame &input_buffer)
    {
        double motor_current = 0.0;

        // ----------------------This block checks the first three bytes of the response code--------------------------

        if( input_buffer.can_id == 0x87000001 )
        {
            int16_t current_motor_current = motor_current;
            *(uint8_t *)(&current_motor_current) = input_buffer.data[5];
            *((uint8_t *)(&current_motor_current) + 1) = input_buffer.data[4];

            motor_current = current_motor_current;

            // RCLCPP_INFO(rclcpp::get_logger("current_logger"), "motor current: %f", motor_current);

            return motor_current;
        }
        else
        {
            // RCLCPP_ERROR(rclcpp::get_logger("current_logger"), "Cannot read current.");

            return 0.00;
        }
    }
    
    ErrorSignal KeyaCodec::decode_error_0_response(can_frame &input_buffer)
    {
        ErrorSignal es;
        if(input_buffer.can_id == 0x87000001 )
        {
            uint8_t err_sig = static_cast<uint8_t>(input_buffer.data[6]);

            es.MOTSTALLED = err_sig & (1 << 7);
            es.CANDISC = err_sig & (1 << 6);
            es.TTTDISC = err_sig & (1 << 5);
            es.CURRSENSE = err_sig & (1 << 4);
            es.HALLFAIL = err_sig & (1 << 3);
            es.RESERVED = err_sig & (1 << 2);
            es.MOTSTALL = err_sig & (1 << 1);
            es.LSPHS = err_sig & (1 << 0);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("error0_logger"),"Byte reading is incorrect.");
        }
        return es;
    }

    ErrorSignal1 KeyaCodec::decode_error_1_response(can_frame &input_buffer)
    {
        ErrorSignal1 es1;
        if( input_buffer.can_id == 0x87000001 )
        {
            uint8_t err_sig = static_cast<uint8_t>(input_buffer.data[7]);
            es1.MODEFAIL = err_sig & (1 << 7);
            es1.OVRCURR = err_sig & (1 << 6);
            es1.NA = err_sig & (1 << 5);
            es1.UNDRVOLT = err_sig & (1 << 4);
            es1.EEPROM = err_sig & (1 << 3);
            es1.HRDWRPROT = err_sig & (1 << 2);
            es1.OVRVOLT = err_sig & (1 << 1);
            es1.DISABLE = err_sig & (1 << 0);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("error0_logger"),"Byte reading is incorrect.");
        }

        return es1;
    }


}