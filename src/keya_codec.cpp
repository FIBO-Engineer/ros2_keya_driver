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
            return true;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("decode_logger"),"Error byte 0 returns: %u", input_buffer.data[0]);
        }
    }

    double KeyaCodec::decode_position_response(can_frame &input_buffer)
    {
        static int32_t prev_position = 0;

        int32_t curr_position = prev_position;
        *(uint8_t *)(&curr_position) = input_buffer.data[4];
        *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
        *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
        *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

        int32_t delta_position = curr_position - prev_position;

        double delta_deg = delta_position * 360 / 10000; // convert count to degree

        prev_position = curr_position;

        return delta_deg; 
    }
    
}