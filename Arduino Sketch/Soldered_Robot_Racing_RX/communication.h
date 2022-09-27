#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "defines.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "DFR_I2C.h"

class RF
{
    public:
    RF();
    int init(const uint8_t *_rx, const uint8_t *_tx);
    int available();
    int checksumCalculation(struct msg *_m);
    int parseData(struct robotData *_r);

    struct msg _myMessage;
    esp_now_peer_info_t peerInfo;
};

#endif