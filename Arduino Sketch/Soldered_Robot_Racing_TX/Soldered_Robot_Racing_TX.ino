/**
 **************************************************
 *
 * @file        Soldered_Robotic_Racing_TX.ino
 * @brief       Control small robot car Micro Maqueen Plus v2 with a PS2 contorller or a USB wheel.
 *              This code recives data from STM32L496 MCU with UART on GPIO14 and GPIO15 and sends it to the robot using a ESP32 ESP-NOW communication.
 *              For his project you will need:
 *              2 x Dasduino ConnectPlus Boards (or even more, depends on how much robots you have, for one robot, you will need two) www.solde.red/333170
 *              1 x MICRO MAQUEEN PLUS V2 Robot www.solde.red/458746
 *              1 x 18650 Lihium Battery
 *              1 x STM32L496 Nucleo Board
 *  	          1 x USB Wheel (or PS2 controller with PS2 to USB controller)
 *              1 x USB Type A Female Adapter Breakout www.solde.red/333132
 *              1 x Micro USB Cable www.solde.red/101295
 *              Jumper wires www.solde.red/100865
 *              To use Dasduino Connect, Arduino Dasduino Connect Core needs to be installed.
 *              
 * @note        Probablly your controller and/or wheel won't work out of the box, some reverse eng. and code change will be needed.
 *
 * @authors     borna@soldered.com
 ***************************************************/

#include "communication.h"  // Library that handles all communication stuff.

// Object for communication library
RF myRF;

// Define pins for second serial for receving data from STM32.
#define RXD2 14
#define TXD2 15

// Array for UART message buffer.
char tempBuffer[110];
// Variable for counting recived chars.
int bufferCounter = 0;

void setup()
{
  // Enable debug serial
  Serial.begin(115200);

  // Print out a message
  Serial.println("Code has started");

  // Serial for communication with STM32.
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Init library for ESP-NOW communication
  myRF.init(rxAddress, txAddress1, txAddress2);
}

void loop()
{
  if (Serial2.available() > 0)
  {
    // Clear whole buffer
    memset(tempBuffer, 0, sizeof(tempBuffer));

    // Reset the counter
    bufferCounter = 0;

    // Variable stores timestamp when last char has been recived.
    unsigned long lastRxChar = micros();

    // Try to catch all chars!
    while ((unsigned long)(micros() - lastRxChar) < 1000)
    {
      if (Serial2.available())
      {
        // There is still some space in the buffer? Read the char.
        if (bufferCounter < 99)
        {
          tempBuffer[bufferCounter++] = Serial2.read();
          lastRxChar = micros();
        }
        else
        {
          // If not, start dropping chars from UART.
          Serial2.read();
        }
      }
    }
  }

  // If there is some data in the buffer, try to read it and parse it.
  if (bufferCounter != 0)
  {
    // Variables for recived data. Last two are not used.
    int id = 0;
    int player = 0;
    int wheel = 0;
    int pedal = 0;
    int buttons1 = 0;
    int buttons2 = 0;
    int res = sscanf(tempBuffer, "%d,%d,%d,%d,%d,%d\r\n", &id, &player, &wheel, &pedal, &buttons1, &buttons2);

    if (res == 6)
    {
        uint8_t siren;
        uint8_t lights;

        // If data is sent from the USB Wheel, there is only one player.
        if (id == 31)
        {
            // Extract pushbutton data for horn
            siren = ((~buttons1) & 15) ? 1 : 0;
            // Extract pushbutton data for "high beam" lights
            lights = (buttons2 & 0b11000000) ? 1 : 0;

            // Debug only
            //Serial.printf("Wheel: %d, ACC: %d, Brake: %d Lights %d Siren %d \r\n", (int)(wheel) - 128, pedal < 128 ? 128 - pedal : 0, pedal > 128 ? pedal - 128 : 0, lights, siren);

            // Send the data to the ESP32 on the robot with ESP-NOW.
            myRF.sendData(wheel, pedal, lights, siren, txAddress1);
        }

        // If data is sent from the PS2 to USB controller, we have a two players now.
        if (id == 63)
        {
            // Extract pushbutton data for horn
            siren = ((buttons1) & 16) ? 1 : 0;
            // Extract pushbutton data for "high beam" lights
            lights = (buttons2 & 0b00000011) ? 1 : 0;

            // Treshold level for analog sticks
            if (wheel < 140 && wheel > 110) wheel = 127;
            if (pedal < 140 && pedal > 110) pedal = 127;

            // If D-Pads are used
            uint8_t dPads = (buttons1) & 15;
            processDPads(dPads, buttons1, &wheel, &pedal);

            // Add offset (on the wheel center position is 128, on the PS2 controller is 127).
            wheel += 1;
            pedal += 1;

            // Check for the limits
            if (wheel > 255) wheel = 255;
            if (pedal > 255) pedal = 255;

            // Debug only
            //Serial.printf("Wheel: %d, ACC: %d, Brake: %d Lights %d Siren %d \r\n", (int)(wheel) - 128, pedal < 128 ? 128 - pedal : 0, pedal > 128 ? pedal - 128 : 0, lights, siren);
        
            // Send the data to the ESP32 on the robot with ESP-NOW.
            myRF.sendData(wheel, pedal, lights, siren, player==1?txAddress1:txAddress2);
        }
    }

    // Reset the buffer counter.
    bufferCounter = 0;
  }
}

void processDPads(uint8_t _dPads, uint8_t _btn, int *_wheel, int *_acc)
{
    // DPad = 0 -> UP
    // DPad = 1 -> UP & RIGHT
    // DPad = 2 -> RIGHT
    // DPad = 3 -> DOWN & RIGHT
    // DPad = 4 -> DOWN
    // DPad = 5 -> DOWN & LEFT
    // DPad = 6 -> LEFT
    // DPad = 7 -> UP & LEFT

    // If X is pressed, every movement must be in FW direction
    if (_btn & 64)
    {
        *_acc = 0;
        if (_dPads == 1) *_wheel = 234;
        if (_dPads == 2) *_wheel = 254;
        if (_dPads == 6) *_wheel = 0;
        if (_dPads == 7) *_wheel = 19;
    }

    // If box is pressed, every movement must be in RW direction
    if (_btn & 128)
    {
        *_acc = 255;
        if (_dPads == 1) *_wheel = 234;
        if (_dPads == 2) *_wheel = 254;
        if (_dPads == 6) *_wheel = 0;
        if (_dPads == 7) *_wheel = 19;
    }
}