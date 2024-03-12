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
        bool DISABLE; // Disabled
        bool OVRVOLT; // Overvoltage
        bool HRDWRPROT; // Hardware protect
        bool EEPROM; // E2PROM
        bool UNDRVOLT; // Undervoltage
        bool NA; // N/A
        bool OVRCURR; // Overcurrent
        bool MODEFAIL; // Mode Failure

        std::string getErrorMessage() const
        {
            std::string message = "";

            message += LSPHS ? "Less Phase, " : "";
            message += MOTSTALL ? "Motor Stall, " : "";
            message += RESERVED ? "Reserved, " : "";
            message += HALLFAIL ? "Hall Failure, " : "";
            message += CURRSENSE ? "Current Sensing, " : "";
            message += TTTDISC ? "232 Disconnected, " : "";
            message += CANDISC ? "CAN Disconnected, " : "";
            message += MOTSTALLED ? "Motor Stalled, " : "";
            message += DISABLE ? "Disabled, " : "";
            message += OVRVOLT ? "Overvoltage, " : "";
            message += HRDWRPROT ? "Hardware Protect, " : "";
            message += EEPROM ? "E2PROM, " : "";
            message += UNDRVOLT ? "Undervoltage, " : "";
            message += NA ? "N/A, " : "";
            message += OVRCURR ? "Overcurrent, " : "";
            message += MODEFAIL ? "Mode Failure, " : "";

            // Remove trailing ", " if it exists
            if (!message.empty())
                message = message.substr(0, message.length() - 2);

            return message;
        }
    };
   
}

#endif // KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_HPP_