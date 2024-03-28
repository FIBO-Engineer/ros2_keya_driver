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

    can_frame KeyaCodec::encode_error_0_request(canid_t can_id)
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

    can_frame KeyaCodec::encode_error_1_request(canid_t can_id)
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
        
        // if(input_buffer.data[0] != 0x60)
        // {
        //     throw std::runtime_error("Incorrect address while decoding position reponse: " + std::to_string(input_buffer.data[0]));
        //     // return 0.0;
        // }
        static double prev_position;
        double curr_position_rad;

        for ( auto &c : input_buffer.data)
        {
            if ( c == 0x60 )
            {
                prev_position = 0.00;
                // curr_position_rad;

                int32_t curr_position = prev_position;
                *(uint8_t *)(&curr_position) = input_buffer.data[4];
                *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
                *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
                *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

                curr_position_rad = curr_position * ( 2 * M_PI) / 10000;

                prev_position = curr_position_rad;

                RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: %f", prev_position);

                // return prev_position;
            }
        }

        return prev_position;

        // static double prev_position = 0.00;
        // double curr_position_rad;

        // int32_t curr_position = prev_position;
        // *(uint8_t *)(&curr_position) = input_buffer.data[4];
        // *((uint8_t *)(&curr_position) + 1) = input_buffer.data[5];
        // *((uint8_t *)(&curr_position) + 2) = input_buffer.data[6];
        // *((uint8_t *)(&curr_position) + 3) = input_buffer.data[7];

        // curr_position_rad = curr_position * ( 2 * M_PI) / 10000;

        // prev_position = curr_position_rad;

        // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "current pos: %f", prev_position);

        // return prev_position;
        // RCLCPP_INFO(rclcpp::get_logger("position_logger"), "Byte 0 returns 0x60");
    }

    double KeyaCodec::decode_current_response(can_frame &input_buffer)
    {
        // if(input_buffer.data[0] != 0x60)
        // {
        //     throw std::runtime_error("Incorrect address while decoding current response: " + std::to_string(input_buffer.data[0]));
        // }

        double motor_current;

        for( auto &c : input_buffer.data)
        {
            if( c == 0x60 )
            {
                motor_current = 0;

                int32_t current_motor_current = motor_current;
                *(uint8_t *)(&current_motor_current) = input_buffer.data[4];

                motor_current = current_motor_current;

                RCLCPP_INFO(rclcpp::get_logger("current_logger"), "motor current: %f", motor_current);

                // return motor_current;
            }
        }

        return motor_current;
        // double motor_current = 0;

        // int32_t current_motor_current = motor_current;
        // *(uint8_t *)(&current_motor_current) = input_buffer.data[4];

        // motor_current = current_motor_current;

        // RCLCPP_INFO(rclcpp::get_logger("current_logger"), "motor current: %f", motor_current);

        // return motor_current;
    }
    
    ErrorSignal KeyaCodec::decode_error_0_response(can_frame &input_buffer)
    {

        // if(input_buffer.data[0] != 0x60)
        // {
        //     std::cout << "packet size: " << sizeof(input_buffer) << std::endl;
        //     throw std::runtime_error("Incorrect address while decoding error0 response: " + std::to_string(input_buffer.data[0]));
        //     // return 0.0;
        // }

        ErrorSignal es;

        for( auto &c : input_buffer.data)
        {
            if( c == 0x60 )
            {
                // ErrorSignal es;

                uint16_t error_data = 0;

                // int32_t error_dat32 = error_data;

                *(uint16_t *)(&error_data) = input_buffer.data[5];

                uint16_t err_sig = static_cast<uint16_t>(error_data);

                RCLCPP_DEBUG(rclcpp::get_logger("Error Debug"), "Error_Signal_0: %d", err_sig);

                es.MOTSTALLED = err_sig & (1 << 7);
                es.CANDISC = err_sig & (1 << 6);
                es.TTTDISC = err_sig & (1 << 5);
                es.CURRSENSE = err_sig & (1 << 4);
                es.HALLFAIL = err_sig & (1 << 3);
                es.RESERVED = err_sig & (1 << 2);
                es.MOTSTALL = err_sig & (1 << 1);
                es.LSPHS = err_sig & (1 << 0);

                RCLCPP_DEBUG(rclcpp::get_logger("error_logger"),"Error DATA0: %d", error_data);

                // return es;
            }
        }

        return es;

        // std::cout << "packet size (error0): " << sizeof(input_buffer) << std::endl;

        // ErrorSignal es;

        // uint16_t error_data = 0;

        // // int32_t error_dat32 = error_data;

        // *(uint16_t *)(&error_data) = input_buffer.data[4];

        // uint16_t err_sig = static_cast<uint16_t>(error_data);

        // RCLCPP_DEBUG(rclcpp::get_logger("Error Debug"), "Error_Signal_0: %d", err_sig);

        // es.MOTSTALLED = err_sig & (1 << 7);
        // es.CANDISC = err_sig & (1 << 6);
        // es.TTTDISC = err_sig & (1 << 5);
        // es.CURRSENSE = err_sig & (1 << 4);
        // es.HALLFAIL = err_sig & (1 << 3);
        // es.RESERVED = err_sig & (1 << 2);
        // es.MOTSTALL = err_sig & (1 << 1);
        // es.LSPHS = err_sig & (1 << 0);

        // RCLCPP_DEBUG(rclcpp::get_logger("error_logger"),"Error DATA0: %d", error_data);

        // return es;
    }

    ErrorSignal1 KeyaCodec::decode_error_1_response(can_frame &input_buffer)
    {

        // if(input_buffer.data[0] != 0x60)
        // {
        //     std::cout << "packet size: " << sizeof(input_buffer) << std::endl;
        //     throw std::runtime_error("Incorrect address while decoding error1 response: " + std::to_string(input_buffer.data[0]));
        //     // return 0.0;
        // }
        
        // std::cout << "packet size (error1): " << sizeof(input_buffer) << std::endl;

        ErrorSignal1 es1;

        for(auto &c : input_buffer.data)
        {
            if( c == 0x60 )
            {
                // ErrorSignal es1;

                uint16_t error_data = 0;

                // int32_t error_dat32 = error_data;

                *(uint16_t *)(&error_data) = input_buffer.data[4];

                uint16_t err_sig = static_cast<uint16_t>(error_data);

                RCLCPP_DEBUG(rclcpp::get_logger("Error Debug"), "Error_Signal_1: %d", err_sig);

                es1.MODEFAIL = err_sig & (1 << 7);
                es1.OVRCURR = err_sig & (1 << 6);
                es1.NA = err_sig & (1 << 5);
                es1.UNDRVOLT = err_sig & (1 << 4);
                es1.EEPROM = err_sig & (1 << 3);
                es1.HRDWRPROT = err_sig & (1 << 2);
                es1.OVRVOLT = err_sig & (1 << 1);
                es1.DISABLE = err_sig & (1 << 0);

                std::cout << "es1.DISABLE: " << es1.DISABLE << std::endl;
                std::cout << "es1.OVRCURR: " << es1.OVRCURR << std::endl;

                RCLCPP_DEBUG(rclcpp::get_logger("error_logger"),"Error DATA1: %d", error_data);
                // RCLCPP_INFO(rclcpp::get_logger("error_logger"),"Error DAT2: %d", input_buffer.data[5]);

                // return es1;
            }
        }

        return es1;

        // ErrorSignal es1;

        // uint16_t error_data = 0;

        // // int32_t error_dat32 = error_data;

        // *(uint16_t *)(&error_data) = input_buffer.data[5];

        // uint16_t err_sig = static_cast<uint16_t>(error_data);

        // RCLCPP_DEBUG(rclcpp::get_logger("Error Debug"), "Error_Signal_1: %d", err_sig);

        // es1.MODEFAIL = err_sig & (1 << 7);
        // es1.OVRCURR = err_sig & (1 << 6);
        // es1.NA = err_sig & (1 << 5);
        // es1.UNDRVOLT = err_sig & (1 << 4);
        // es1.EEPROM = err_sig & (1 << 3);
        // es1.HRDWRPROT = err_sig & (1 << 2);
        // es1.OVRVOLT = err_sig & (1 << 1);
        // es1.DISABLE = err_sig & (1 << 0);

        // // std::cout << "es.DISABLE: " << es.DISABLE << std::endl;

        // RCLCPP_DEBUG(rclcpp::get_logger("error_logger"),"Error DATA1: %d", error_data);
        // // RCLCPP_INFO(rclcpp::get_logger("error_logger"),"Error DAT2: %d", input_buffer.data[5]);

        // return es1;
    }


}