#include "communication.h"

// These variables,struct and functions must be accessable in the interrupt, so they need to be outside the class.
static struct msg myMessage;
static bool incommingDataFlag = false;

// 
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // Empty, could be used for debugging
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // Empty, could be used for debugging
}

// Constructor
RF::RF()
{

}

// Init of the radio class (ESP-NOW)
int RF::init(const uint8_t *_rx, const uint8_t *_tx1, const uint8_t *_tx2)
{
    // Sleep, esp_wifi_start() needs to be called, otherwise ESP32 will hang.
    esp_wifi_start();

    // Power up WiFi Radio.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Force ESP32 to change MAC address
    esp_wifi_set_mac(WIFI_IF_STA, _rx);

    // It ESP-NOW init has failed, return error.
    if (esp_now_init() != ESP_OK)
    {
        return false;
    }

    // Attach interrupt functions for recevied and sent data.
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Register first peer
    memcpy(peerInfo.peer_addr, _tx1, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add first peer to ESP-NOW
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer1");
        return 0;
    }

    // Register second peer
    memcpy(peerInfo.peer_addr, _tx2, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add second peer to ESP-NOW
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer2");
        return 0;
    }

    // Everything went ok? Return success!
    return true;
}

int RF::available()
{
    return (int)(incommingDataFlag);
}

int RF::checksumCalculation(struct msg *_m)
{
    // Point to the start of the struct and treat stuct as array of 8 bit data.
    uint8_t *_p = (uint8_t*)_m;

    // Variable for storing sum.
    uint8_t _sum = 0;

    // Go trough whole struct. Skip checksum itself (uint8_t) and lastMessageTimestamp (unsigned long).
    for (int i = 0; i < (sizeof(struct msg) - 8); i++)
    {
        _sum += *(_p++);
    }

    //Return calculated sum.
    return _sum;
}

int RF::sendData(int wheel, int acc, int lights, int siren, const uint8_t *_txAddress)
{
    // First make a struct message.
    // Clear whole struct (just to be sure there are no garbage data left)
    memset(&myMessage, 0, sizeof(myMessage));

    // Fill message struct with data
    myMessage.acc = acc;
    myMessage.wheel = wheel;
    myMessage.lights = lights;
    myMessage.siren = siren;
    myMessage.header = MESSAGE_HEADER;
    myMessage.checksum = checksumCalculation(&myMessage);

    esp_now_send(_txAddress, (uint8_t*)(&myMessage), sizeof(myMessage));

    return 1;
}