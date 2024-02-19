#ifndef KEYA_DRIVER_CONFIG_H
#define KEYA_DRIVER_CONFIG_H

#include <string>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>

struct Config 
{
    std::string device_id = "can0";
    std::vector<canid_t> can_id_list = {1};
};

#endif