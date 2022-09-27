#include "DFR_I2C.h"

DFR::DFR()
{
}

void DFR::begin()
{
    Wire.begin();
    memset(&_myRegs, 0, sizeof(regs));
    setLeds(0, DFR_LED_RIGHT);
    setLeds(0, DFR_LED_LEFT);
    stopMotors();
}

struct regs DFR::getAllRegs()
{
    for (int i = 0; i < 5; i++)
    {
        getAdc(i);
    }
    getLineState();
    getLed(0);
    getLed(1);
    getMotor(0);
    getMotor(1);
    getVersion();
    return _myRegs;
}

void DFR::printRegs(struct regs _r)
{
    Serial.println("=========Registers=========");
    for (int i = 0; i < 5; i++)
    {
        Serial.print("ADC");
        Serial.print(i, DEC);
        Serial.print(": ");
        Serial.println(_r.adc[i], DEC);
    }
    Serial.print("Line state: ");
    Serial.println(_r.lineState, DEC);

    for (int i = 0; i < 2; i++)
    {
        Serial.print("Motor Speed");
        Serial.print(i, DEC);
        Serial.print(" state");
        Serial.print(": ");
        Serial.println(_r.motorSpeed[i], DEC);
    }

    for (int i = 0; i < 2; i++)
    {
        Serial.print("Motor Direction");
        Serial.print(i, DEC);
        Serial.print(" state");
        Serial.print(": ");
        Serial.println(_r.motorDir[i], DEC);
    }

    for (int i = 0; i < 2; i++)
    {
        Serial.print("LED ");
        Serial.print(i, DEC);
        Serial.print(" state");
        Serial.print(": ");
        Serial.println(_r.leds[i], DEC);
    }

    Serial.print("VERSION_CNT_REGISTER: ");
    Serial.println(_r.verC, DEC);
    Serial.print("VERSION_DATA_REGISTER: ");
    Serial.println(_r.verD, DEC);

    Serial.println("\r\n\r\n\r\n\r\n");
}

uint16_t DFR::getAdc(int _ch)
{
    _ch = _ch % 5;
    uint8_t _regAddr = ADC0_REGISTER + (2 * _ch);
    _myRegs.adc[_ch] = get16Bits(_regAddr);
    return _myRegs.adc[_ch];
}

uint8_t DFR::getLineState()
{
    _myRegs.lineState = get8Bits(LINE_STATE_REGISTER);
    return _myRegs.lineState;
}

uint8_t DFR::getLed(int _ch)
{
    _ch = _ch % 2;
    uint8_t _regAddr = LEFT_LED_REGISTER + _ch;
    _myRegs.leds[_ch] = get8Bits(_regAddr);
    return _myRegs.leds[_ch];
}

uint16_t DFR::getMotor(int _ch)
{
    _ch = _ch % 2;
    uint8_t _regAddr = LEFT_MOTOR_REGISTER + (2 * _ch);
    uint16_t _data = get16Bits(_regAddr);
    _myRegs.motorSpeed[_ch] = _data >> 8;
    _myRegs.motorDir[_ch] = _data & 0xFF;
    return _data;
}

uint16_t DFR::getVersion()
{
    uint16_t _data = get16Bits(VERSION_CNT_REGISTER);
    _myRegs.verC = _data >> 8;
    _myRegs.verD = _data & 0xff;
    return _data;
}

void DFR::setLeds(uint8_t _state, int _ch)
{
    _ch = _ch % 2;
    _myRegs.leds[_ch] = _state;
    uint8_t _regAddr = LEFT_LED_REGISTER + _ch;
    send8Bits(_regAddr, _myRegs.leds[_ch]);
}

void DFR::setMotor(uint8_t _speed, uint8_t _dir, int _ch)
{
    _ch = _ch % 2;
    _myRegs.motorSpeed[_ch] = _speed;
    _myRegs.motorDir[_ch] = _dir;
    uint8_t _regAddr = LEFT_MOTOR_REGISTER + (2 * _ch);
    send16Bits(_regAddr, (_myRegs.motorDir[_ch]) | _myRegs.motorSpeed[_ch] << 8);
}

void DFR::stopMotors()
{
    _myRegs.motorSpeed[DFR_MOTOR_RIGHT] = 0;
    _myRegs.motorDir[DFR_MOTOR_RIGHT] = DFR_MOTOR_RIGHT;
    _myRegs.motorSpeed[DFR_MOTOR_LEFT] = 0;
    _myRegs.motorDir[DFR_MOTOR_LEFT] = DFR_MOTOR_LEFT;
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(LEFT_MOTOR_REGISTER);
    Wire.write(_myRegs.motorDir[DFR_MOTOR_LEFT]);
    Wire.write(_myRegs.motorSpeed[DFR_MOTOR_LEFT]);
    Wire.write(_myRegs.motorDir[DFR_MOTOR_RIGHT]);
    Wire.write(_myRegs.motorSpeed[DFR_MOTOR_RIGHT]);
    Wire.endTransmission();
}

void DFR::startMotors(uint8_t _speedL, uint8_t _dirL, uint8_t _speedR, uint8_t _dirR)
{
    _myRegs.motorSpeed[DFR_MOTOR_RIGHT] = _speedR;
    _myRegs.motorDir[DFR_MOTOR_RIGHT] = _dirR;
    _myRegs.motorSpeed[DFR_MOTOR_LEFT] = _speedL;
    _myRegs.motorDir[DFR_MOTOR_LEFT] = _dirL;
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(LEFT_MOTOR_REGISTER);
    Wire.write(_myRegs.motorDir[DFR_MOTOR_LEFT]);
    Wire.write(_myRegs.motorSpeed[DFR_MOTOR_LEFT]);
    Wire.write(_myRegs.motorDir[DFR_MOTOR_RIGHT]);
    Wire.write(_myRegs.motorSpeed[DFR_MOTOR_RIGHT]);
    Wire.endTransmission();
}

// ========Low level stuff========
uint8_t DFR::get8Bits(uint8_t _reg)
{
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(_reg);
    Wire.endTransmission();

    Wire.requestFrom(I2CADDR_ROBOT, 1);
    while (Wire.available() != 1)
        ;
    return (Wire.read());
}

uint16_t DFR::get16Bits(uint8_t _reg)
{
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(_reg);
    Wire.endTransmission();

    Wire.requestFrom(I2CADDR_ROBOT, 2);
    while (Wire.available() != 2)
        ;
    return ((Wire.read()) | (Wire.read() << 8));
}

void DFR::send8Bits(uint8_t _reg, uint8_t _data)
{
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(_reg);
    Wire.write(_data);
    Wire.endTransmission();
}

void DFR::send16Bits(uint8_t _reg, uint16_t _data)
{
    Wire.beginTransmission(I2CADDR_ROBOT);
    Wire.write(_reg);
    Wire.write(_data & 0xFF);
    Wire.write(_data >> 8);
    Wire.endTransmission();
}
