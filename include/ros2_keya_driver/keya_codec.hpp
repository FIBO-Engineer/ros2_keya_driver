#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_CODEC_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_CODEC_HPP_

#include <cstdint>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros2_keya_driver/keya_error_signal.hpp>
#include <ros2_keya_driver/keya_error_signal_1.hpp>

namespace keya_driver_hardware_interface
{

    class KeyaCodec
    {
    public:
        KeyaCodec();

        // For all null value return request command
        bool decode_command_response(can_frame &input_buffer);

        // I/O
        can_frame encode_position_command_request(canid_t can_id, double cmd);
        bool decode_position_command_response(can_frame &input_buffer);
        can_frame encode_position_request(canid_t can_id);
        double decode_position_response(can_frame &input_buffer);
        can_frame encode_current_request(canid_t can_id);
        double decode_current_response(can_frame &input_buffer);

        // Status
        can_frame encode_error_0_request(canid_t can_id);
        static ErrorSignal decode_error_0_response(can_frame &input_buffer);  
        can_frame encode_error_1_request(canid_t can_id);
        static ErrorSignal1 decode_error_1_response(can_frame &input_buffer);    

        double decode_current_heartbeat_response(can_frame &input_buffer);  

        // Motor Enabling 
        can_frame encode_position_control_enable_request(canid_t can_id);
        
        // Motor Disabling
        can_frame encode_position_control_disable_request(canid_t can_id);

    };
}

#endif