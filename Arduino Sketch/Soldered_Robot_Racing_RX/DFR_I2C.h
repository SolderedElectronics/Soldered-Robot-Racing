#ifndef __DFR_I2C_H__
#define __DFR_I2C_H__

#include <Arduino.h>
#include <Wire.h>

#define I2CADDR_ROBOT        0x10
#define ADC0_REGISTER        0X1E
#define ADC1_REGISTER        0X20
#define ADC2_REGISTER        0X22
#define ADC3_REGISTER        0X24
#define ADC4_REGISTER        0X26
#define LEFT_LED_REGISTER    0X0B
#define RIGHT_LED_REGISTER   0X0C
#define LEFT_MOTOR_REGISTER  0X00
#define RIGHT_MOTOR_REGISTER 0X02
#define LINE_STATE_REGISTER  0X1D
#define VERSION_CNT_REGISTER 0X32
#define VERSION_DATA_REGISTER

#define DFR_LED_LEFT    0
#define DFR_LED_RIGHT   1
#define DFR_MOTOR_LEFT  0
#define DFR_MOTOR_RIGHT 1

#define DFR_SENSOR_R2 0
#define DFR_SENSOR_R1 1
#define DFR_SENSOR_M  2
#define DFR_SENSOR_L1 3
#define DFR_SENSOR_L2 4
#define DFR_MOTOR_FW  0
#define DFR_MOTOR_RW  1

#define DFR_LINE_R2 0b00000001
#define DFR_LINE_R1 0b00000010
#define DFR_LINE_M  0b00000100
#define DFR_LINE_L1 0b00001000
#define DFR_LINE_L2 0b00010000

struct regs
{
    uint16_t adc[5];
    uint8_t lineState;
    uint8_t leds[2];
    uint8_t motorSpeed[2];
    uint8_t motorDir[2];
    uint8_t verC;
    uint8_t verD;
};

class DFR
{
  public:
    DFR();
    void begin();
    struct regs getAllRegs();

    void printRegs(struct regs _r);
    uint16_t getAdc(int _ch);
    uint8_t getLineState();
    uint8_t getLed(int _ch);
    uint16_t getMotor(int _ch);
    uint16_t getVersion();

    void setLeds(uint8_t _state, int _ch);
    void setMotor(uint8_t _speed, uint8_t _dir, int _ch);
    void stopMotors();
    void startMotors(uint8_t _speedL, uint8_t _dirL, uint8_t _speedR, uint8_t _dirR);

  private:
    struct regs _myRegs;
    uint8_t get8Bits(uint8_t _reg);
    uint16_t get16Bits(uint8_t _reg);
    void send8Bits(uint8_t _reg, uint8_t _data);
    void send16Bits(uint8_t _reg, uint16_t _data);
};

#endif
