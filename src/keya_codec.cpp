#include <ros2_keya_driver/keya_codec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <numeric>

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
        int32_t cmd_unit = cmd * 10000 / 360;
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x23;
        frame.data[1] = 0x02;
        frame.data[2] = 0x20;
        frame.data[3] = 0x01;
        frame.data[4] = *((uint8_t *)(&cmd_unit) + 3);
        frame.data[5] = *((uint8_t *)(&cmd_unit) + 4);
        frame.data[6] = *(uint8_t *)(&cmd_unit);
        frame.data[7] = *((uint8_t *)(&cmd_unit) + 2);
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

    bool KeyaCodec::decode_command_response(can_frame &input_buffer)
    {
        
        if (input_buffer.data[0] == 0x60)
        {
            RCLCPP_INFO(rclcpp::get_logger("decode_logger"),"Byte 0 matched");
            return true;
        }
        // if (input_buffer.data[0] == 72)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("decode_logger"), "Byte 0 matched");
        //     return true;
        // }
        // if (input_buffer.data[0] == 0)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("decode_logger"), "Byte 0 is 0: Hardware already connected");
        //     return true;
        // }
        // if (input_buffer.data[0] == 72)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("decode_logger"), "Byte 0 is 72: Hardware connected but data is incorrect.");
        //     return false;
        // }
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
        // if(input_buffer.data[0] == 0x60)
        // {
        //     // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "Byte 0 matched");

        //     static int32_t prev_position = 0;

        //     int32_t curr_position = prev_position;
        //     *(uint8_t *)(&curr_position) = input_buffer.data[4];
        //     *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
        //     *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
        //     *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

        //     int32_t delta_position = curr_position - prev_position;

        //     double delta_deg = delta_position * 360 / 10000; // convert count to degree

        //     prev_position = curr_position;

        //     // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "Byte 0 returns 0x60");
        //     RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: %f", delta_deg);

        //     return delta_deg;
        // }
        // else
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: 0.000000");

        //     return 0.0;
        // }
        
        if(input_buffer.data[0] != 0x60)
        {
            throw std::runtime_error("Incorrect address while decoding position reponse: " + std::to_string(input_buffer.data[0]));
            // return 0.0;
        }

        static int32_t prev_position = 0;

        int32_t curr_position = prev_position;
        *(uint8_t *)(&curr_position) = input_buffer.data[4];
        *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
        *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
        *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

        // int32_t delta_position = curr_position - prev_position;

        // double delta_deg = delta_position * 360 / 10000; // convert count to degree

        curr_position = curr_position * 360 / 10000;

        prev_position = curr_position;

        if( prev_position == 2359 || prev_position == -2359 || prev_position == 2350 || prev_position == -2350)
        {
            RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: 0");

            return 0;

        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: %d", prev_position);

            return prev_position;
        }
        // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "Byte 0 returns 0x60");
    }
    
}