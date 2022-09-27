/**
 **************************************************
 *
 * @file        Soldered_Robotic_Racing_RX.ino
 * @brief       Control small robot car Micro Maqueen Plus v2 with a PS2 contorller or a USB wheel.
 *              This code recives data from Dasduino ConnectPlus, parse it and sends it to the robot with I2C communication.
 *              For his project you will need:
 *              2 x Dasduino ConnectPlus Boards (or even more, depends on how much robots you have, for one robot, you will need two) www.solde.red/333170
 *              1 x MICRO MAQUEEN PLUS V2 Robot www.solde.red/458746
 *              1 x 18650 Lihium Battery
 *              1 x STM32L496 Nucleo Board
 *  	        1 x USB Wheel (or PS2 controller with PS2 to USB controller)
 *              Jumper wires www.solde.red/100865
 *              To use Dasduino Connect, Arduino Dasduino Connect Core needs to be installed.
 *              
 * @note        Probablly your controller and/or wheel won't work out of the box, some reverse eng. and code change will be needed.
 *              Also you will need install two libraries:
 *              - Soldered WS2812 LED Library: https://github.com/e-radionicacom/Soldered-WS2812-Smart-Leds-Arduino-Library
 *              - Tone32 Library for the buzzer: https://github.com/lbernstone/Tone32
 *
 * @authors     borna@soldered.com
 ***************************************************/
#include <Wire.h>               // Arduino Wire Library (needed for the robot)
#include "DFR_I2C.h"            // Library for controlling a robot
#include "Tone32.h"             // Library for buzzer
#include "WS2812-SOLDERED.h"    // Library for WS2812 LEDs
#include "communication.h"      // Library for capturing and parsing RF data
#include "defines.h"

// Objects for the libraries.
DFR robot;
RF rf;
WS2812 led(NUMPIXELS, WS_PIN);

// Structure that holds the data of the current robot status (speed of each motor, direction, lights, horn, etc)
struct robotData myRobotData = {0};

void setup()
{
    // Init serial communication (for debug purpose only)
    Serial.begin(115200);

    // Init I2C communication for the robot
    Wire.begin();

    // Init library for controlling the robot
    robot.begin();

    // Init library for RF communication (ESP-NOW and data parsing)
    rf.init(rxAddress1, txAddress);

    // Init library for the WS2812 LEDs on the robot.
    led.begin();

    // Turn on Front light to "low beam"
    led.setPixelColor(0, led.Color(32, 32, 32));
    led.setPixelColor(3, led.Color(32, 32, 32));
    led.show();

    // Turn on the tailgate lights
    led.setPixelColor(1, led.Color(128, 0, 0));
    led.setPixelColor(2, led.Color(128, 0, 0));
    led.show();

    // Set buzzer GPIO to output, low.
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

void loop()
{
    // If there is some new data, try to parse it.
    if (rf.available())
    {
        struct robotData myRobotDataNew;
        // If data is parsed successfully (checksum is ok, as well as data header) set new parameters on the robot
        if (rf.parseData(&myRobotDataNew))
        {
            // Set new motor speed
            robot.startMotors(myRobotDataNew.lMotor, myRobotDataNew.dir, myRobotDataNew.rMotor, myRobotDataNew.dir);

            // Check if the state of the buzzer / horn / siren has changed.
            if (myRobotDataNew.horn != myRobotData.horn)
            {
                // If buzzer / horn / siren in now set, turn the buzzer on. Otherwise, turn it of.
                if (myRobotDataNew.horn)
                {
                    // Set buzzer frequency to 500Hz on GPIO15.
                    ledcAttachPin(BUZZER_PIN, 0);
                    ledcWriteTone(0, BUZZER_FREQ);
                }
                else
                {
                    // Trun off the buzzer.
                    ledcDetachPin(BUZZER_PIN);
                    ledcWrite(0, 0);
                }
            }

            // Check if the state of the front lights needs to be changed.
            if (myRobotDataNew.frontLights != myRobotData.frontLights)
            {
                // Frontlights are now on? Turn the WS LEDs at the front of the robot on (high beam)
                if (myRobotDataNew.frontLights)
                {
                    led.setPixelColor(0, led.Color(255, 255, 255));
                    led.setPixelColor(3, led.Color(255, 255, 255));
                    led.show();
                    robot.setLeds(255, DFR_LED_LEFT);
                    robot.setLeds(255, DFR_LED_RIGHT);
                }
                else
                {
                    // Frontlights are now off? Dim the WS LEDs.
                    led.setPixelColor(0, led.Color(32, 32, 32));
                    led.setPixelColor(3, led.Color(32, 32, 32));
                    led.show();
                    robot.setLeds(0, DFR_LED_LEFT);
                    robot.setLeds(0, DFR_LED_RIGHT);
                }
            }

            // If robot is stopped, show brake lights
            if (myRobotDataNew.lMotor == 0 && myRobotDataNew.rMotor == 0)
            {
                led.setPixelColor(1, led.Color(128, 0, 0));
                led.setPixelColor(2, led.Color(128, 0, 0));
                led.show();
            }
            else
            {
                // Otherwise, show tailgate lights if robot is goint forward or reverse lights if the robot is going in reverese.
                if (myRobotDataNew.dir == DFR_MOTOR_FW)
                {
                    // Tailgate lights
                    led.setPixelColor(1, led.Color(32, 0, 0));
                    led.setPixelColor(2, led.Color(32, 0, 0));
                    led.show();
                }
                else
                {
                    // Reverese lights
                    led.setPixelColor(1, led.Color(128, 128, 128));
                    led.setPixelColor(2, led.Color(128, 128, 128));
                    led.show();
                }
            }

            // Update structure for current robot status.
            myRobotData = myRobotDataNew;
        }
        else
        {
            // Display an error message
            Serial.println("Failed to parse data");
        }
    }
}