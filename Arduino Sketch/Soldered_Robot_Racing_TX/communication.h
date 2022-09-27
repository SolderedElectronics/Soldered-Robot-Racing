#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "defines.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

class RF
{
    public:
    RF();
    int init(const uint8_t *_rx, const uint8_t *_tx1, const uint8_t *_tx2);
    int available();
    int checksumCalculation(struct msg *_m);
    int sendData(int wheel, int acc, int lights, int siren, const uint8_t *_txAddress);

    struct msg _myMessage;
    esp_now_peer_info_t peerInfo;
};

#endif