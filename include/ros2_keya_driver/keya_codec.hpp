#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_CODEC_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__KEYA_CODEC_HPP_

#include <cstdint>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace keya_driver_hardware_interface
{
    enum CANStatus
    {
        MOTOR_CURRENT
    };

    class KeyaCodec
    {
    public:
        KeyaCodec();

        // For all null value return request command
        bool decode_command_response(can_frame &input_buffer);

        // I/O
        can_frame encode_position_command_request(canid_t can_id, double cmd);
        can_frame encode_position_request(canid_t can_id);
        double decode_position_response(can_frame &input_buffer);

        // Status
        // can_frame encode_current_request(canid_t can_id); // Motor Current Query (0x600021)
        CANStatus decode_current_response(can_frame &input_buffer);

        // Motor Enabling 
        can_frame encode_position_control_enable_request(canid_t can_id);
        
        // Motor Disabling
        can_frame encode_position_control_disable_request(canid_t can_id);

    };
}

#endif