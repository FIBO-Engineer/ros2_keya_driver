#ifndef KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_1_HPP_
#define KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_1_HPP_

#include <string>

namespace keya_driver_hardware_interface
{
    struct ErrorSignal1
    {
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

            // DAT1 Error Set
            message += MODEFAIL ? "Mode Failure, " : "";
            message += OVRCURR ? "Overcurrent, " : "";
            message += NA ? "N/A, " : "";
            message += UNDRVOLT ? "Undervoltage, " : "";
            message += EEPROM ? "E2PROM, " : "";
            message += HRDWRPROT ? "Hardware Protect, " : "";
            message += OVRVOLT ? "Overvoltage, " : "";
            message += DISABLE ? "Disabled, " : "";

            // Remove trailing ", " if it exists
            if (!message.empty())
                message = message.substr(0, message.length() - 2);

            return message;
        }
    };
   
}

#endif // KEYA_DRIVER_HARDWARE_INTERFACE__STATUS_SIGNAL_HPP_