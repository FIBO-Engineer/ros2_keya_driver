#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_HPP_

#include <string>

namespace keya_driver_hardware_interface
{
    struct ErrorSignal
    {
        bool LSPHS; // Less Phase
        bool MOTSTALL; // Motor Stall
        bool RESERVED; // Reserved
        bool HALLFAIL; // Hall Failure
        bool CURRSENSE; // Current Sensing
        bool TTTDISC; // 232 disconnected
        bool CANDISC; // CAN disconnected
        bool MOTSTALLED; // Motor stalled
        bool UNKNOWN;

        std::string getErrorMessage() const
        {
            std::string message = "";

            // DAT0 Error Set
            message += MOTSTALLED ? "Motor Stalled, " : "";
            message += CANDISC ? "CAN Disconnected, " : "";
            message += TTTDISC ? "232 Disconnected, " : "";
            message += CURRSENSE ? "Current Sensing, " : "";
            message += HALLFAIL ? "Hall Failure, " : "";
            message += RESERVED ? "Reserved, " : "";
            message += MOTSTALL ? "Motor Stall, " : "";
            message += LSPHS ? "Less Phase, " : "";

            // Remove trailing ", " if it exists
            if (!message.empty())
                message = message.substr(0, message.length() - 2);

            return message;
        }
    };
   
}

#endif // KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_HPP_