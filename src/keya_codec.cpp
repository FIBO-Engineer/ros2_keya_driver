#include <ros2_keya_driver/keya_codec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <numeric>
#include <cstdint>
#include <iostream>
#include <math.h>

namespace keya_driver_hardware_interface
{
    KeyaCodec::KeyaCodec() {}

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
        
        if (input_buffer.data[0] == 0x60)
        {
            RCLCPP_INFO(rclcpp::get_logger("decode_logger"),"Byte 0 matched");
            return true;
        }
        else 
        {
            RCLCPP_ERROR(rclcpp::get_logger("decode_logger"),"Error byte 0 returns: %u", input_buffer.data[0]);
            return false;
        }
    }

    bool KeyaCodec::decode_position_command_response(can_frame &input_buffer)
    {
        RCLCPP_INFO(rclcpp::get_logger("cmd_decode_logger"), "Current byte 0: %u", input_buffer.data[0]);

        return true;
    }

    double KeyaCodec::decode_position_response(can_frame &input_buffer)
    {
        
        if(input_buffer.data[0] != 0x60)
        {
            throw std::runtime_error("Incorrect address while decoding position reponse: " + std::to_string(input_buffer.data[0]));
            // return 0.0;
        }

        static double prev_position = 0.00;
        double curr_position_rad;

        int32_t curr_position = prev_position;
        *(uint8_t *)(&curr_position) = input_buffer.data[4];
        *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
        *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
        *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

        curr_position_rad = curr_position * ( 2 * M_PI) / 10000;

        prev_position = curr_position_rad;

        RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: %f", prev_position);

        return prev_position;
        // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "Byte 0 returns 0x60");
    }

    double KeyaCodec::decode_current_response(can_frame &input_buffer)
    {
        if(input_buffer.data[0] != 0x60)
        {
            throw std::runtime_error("Incorrect address while decoding current response: " + std::to_string(input_buffer.data[0]));
        }

        double motor_current = 0;

        int32_t current_motor_current = motor_current;
        *(uint8_t *)(&current_motor_current) = input_buffer.data[4];

        motor_current = current_motor_current;

        RCLCPP_INFO(rclcpp::get_logger("current_logger"), "motor current: %f", motor_current);

        return motor_current;
    }
    
    ErrorSignal KeyaCodec::decode_error_response(can_frame &input_buffer)
    {

        if(input_buffer.data[0] != 0x60)
        {
            throw std::runtime_error("Incorrect address while decoding error response: " + std::to_string(input_buffer.data[0]));
            // return 0.0;
        }

        ErrorSignal es;

        int16_t error_dat1 = 0;
        int16_t error_dat2 = 0;

        // int32_t error_dat32 = error_data; 

        *(uint8_t *)(&error_dat1) = input_buffer.data[4];
        *((uint8_t *)(&error_dat2) + 1) = input_buffer.data[5];

        RCLCPP_INFO(rclcpp::get_logger("error_logger"),"Error DAT1: %d", error_dat1);
        RCLCPP_INFO(rclcpp::get_logger("error_logger"),"Error DAT2: %d", error_dat2);
        // RCLCPP_INFO(rclcpp::get_logger("error_logger"),"Error DAT2: %d", input_buffer.data[5]);


        return es;
    }

}