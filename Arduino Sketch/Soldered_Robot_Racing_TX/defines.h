#ifndef __ROBOT_DEFINES_H__
#define __ROBOT_DEFINES_H__

#include "stdint.h"

// Message header (so proper message can be recognised). Random picked number, can be any 32 bit number.
// But keep in mind, the same number must be used on the robot as well as on controller.
// Otherwise it won't work.
#define MESSAGE_HEADER   0x95137456

// RF MAC Adresses. Again, randomly picked. Can be anything but it must be same on the both sides.
const uint8_t rxAddress[] = {0xE0, 0xE2, 0xE6, 0xB1, 0xE5, 0x47};
const uint8_t txAddress1[] = {0xE0, 0xE2, 0xE6, 0xB1, 0xE5, 0x48};
const uint8_t txAddress2[] = {0xE0, 0xE2, 0xE6, 0xB1, 0xE5, 0x49};

// From DFR_I2C.h
#define DFR_MOTOR_FW  0
#define DFR_MOTOR_RW  1

// Message received from the "air".
struct msg
{
    uint32_t header;
    uint8_t wheel;
    uint8_t acc;
    uint8_t lights;
    uint8_t siren;
    uint8_t checksum;
    unsigned long lastMessageTimestamp;
};

// Struct for the robot itself. Holds data about left and right motor speed, as well as status of the horn and lights.
struct robotData
{
    uint8_t lMotor;
    uint8_t rMotor;
    uint8_t dir;
    uint8_t horn;
    uint8_t frontLights;
    uint8_t brakeLights;
};

#endif