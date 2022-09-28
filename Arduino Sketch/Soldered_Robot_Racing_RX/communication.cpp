#include "communication.h"

// These variables,struct and functions must be accessable in the interrupt, so they need to be outside the class.
static struct msg myMessage;
static bool incommingDataFlag = false;

// 
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&myMessage, incomingData, sizeof(myMessage));
    myMessage.lastMessageTimestamp = micros();
    incommingDataFlag = true;
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
int RF::init(const uint8_t *_rx, const uint8_t *_tx)
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

    // Register peer
    memcpy(peerInfo.peer_addr, _tx, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer to ESP-NOW
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
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

int RF::parseData(struct robotData *_r)
{
    // First clear the flag
    incommingDataFlag = false;

    //Serial.printf("Wheel: %d, Acc:%d\r\n", myMessage.wheel, myMessage.acc);

    // Then check the checksum
    uint8_t _calculatedChecksum = checksumCalculation(&myMessage);
    
    // Checksum is wrong? Return error!
    if ((_calculatedChecksum != myMessage.checksum) || (myMessage.header != MESSAGE_HEADER)) return 0;

    // Now let's extract the data from the struct
    _r->frontLights = myMessage.lights;
    _r->horn = myMessage.siren;
    
    // For the motors, we need a little bit of calculation

    // If acceleration data is bigger or equal than 127, go forward!
    // Otherwise, go reverese.
    if (myMessage.acc > 128)
    {
        _r->dir = DFR_MOTOR_RW;
        // Convert speed into normal range (from 0...128)
        myMessage.acc -= 128;
    }
    else
    {
        _r->dir = DFR_MOTOR_FW;
        // Convert speed into normal range (from 0...128)
        myMessage.acc = 128 - myMessage.acc;
    }

    // Set the speed of the motors
    _r->lMotor = map(myMessage.acc, 0, 128, 0, 255);
    _r->rMotor = _r->lMotor;

    // If there is rotation needed, substract percetange of the speed on the motor in direction of the robot rotation.
    if (myMessage.wheel > 128)
    {
        _r->rMotor -= map(myMessage.wheel, 129, 255, 0, _r->rMotor);
    }
    else if (myMessage.wheel < 128)
    {
        _r->lMotor -= map(myMessage.wheel, 127, 0, 0, _r->lMotor);
    }

    return 1;
}

unsigned long RF::lastMessageTime()
{
    return myMessage.lastMessageTimestamp;
}

void RF::clearLastMessageTime()
{
    myMessage.lastMessageTimestamp = 0;
}