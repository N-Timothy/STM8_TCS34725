#pragma once
#ifndef TCS34725_H
#define TCS34725_H

//#include Wire.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

#define I2C_ADDR (0x29)
#define ID_REG_PART_NUMBER (0x44)
#define COMMAND_BIT (0x80)

#define INTEGRATION_CYCLES_MIN (1.f)
#define INTEGRATION_CYCLES_MAX (256.f)
#define INTEGRATION_TIME_MS_MIN (2.4f)
#define INTEGRATION_TIME_MS_MAX (INTEGRATION_TIME_MS_MIN * INTEGRATION_CYCLES_MAX)

struct Reg {
    volatile uint8_t ENABLE;
    volatile uint8_t ATIME;
    volatile uint8_t WTIME;
    volatile uint8_t AILTL;
    volatile uint8_t AILTH;
    volatile uint8_t AIHTL;
    volatile uint8_t AIHTH;
    volatile uint8_t PERS;
    volatile uint8_t CONFIG;
    volatile uint8_t CONTROL;
    volatile uint8_t ID;
    volatile uint8_t STATUS;
    volatile uint8_t CDATAL;
    volatile uint8_t CDATAH;
    volatile uint8_t RDATAL;
    volatile uint8_t RDATAH;
    volatile uint8_t GDATAL;
    volatile uint8_t GDATAH;
    volatile uint8_t BDATAL;
    volatile uint8_t BDATAH;
} reg={0x00, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07,
0x0C, 0x0D, 0x0F, 0x12, 0x13, 0x14, 0x15,
0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B};

struct Mask {

uint8_t ENABLE_AIEN;
uint8_t ENABLE_WEN;
uint8_t ENABLE_AEN;
uint8_t ENABLE_PON;
uint8_t STATUS_AINT;
uint8_t STATUS_AVALID;
} mask = {0x10, 0x08, 0x02, 0x01, 0x10, 0x01};

struct gain {
    uint8_t X01;
    uint8_t X04;
    uint8_t X16;
    uint8_t X60;
}

struct Color
{
    float r, g, b;
};

union RawData
    {
        struct
        {
            uint16_t c;
            uint16_t r;
            uint16_t g;
            uint16_t b;
        };

        uint8_t raw[sizeof(uint16_t) * 4];
} raw_data;

bool b_ct_lux_calc = true;
//WireType* wire;
float scaling = 2.5f;

    //// for lux & temperature
float lx;
float color_temp;
float gain_value = 1.f;
uint8_t atime = 0xFF;
float integration_time = 2.4f; // [ms]
float glass_attenuation = 1.f;


void write8(uint8_t reg, uint8_t value)
{
    uint8_t test1 = reg;
    uint8_t test2 = value;
        //Wire_beginTransmission(I2C_ADDR);
        //Wire_write(COMMAND_BIT | (uint8_t)reg);
        //wire_write(value);
        //wire_endTransmission();
}

uint8_t read8(uint8_t id)
{
        //Wire_beginTransmission(I2C_ADDR);
        //wire_write(COMMAND_BIT | (uint8_t)reg);
        //wire_endTransmission();
        //wire_requestFrom(I2C_ADDR, (uint8_t)1);
        //return wire_read();

        return id; //example
}

void update()
{
        //wire_beginTransmission(I2C_ADDR);
        //wire_write(COMMAND_BIT | (uint8_t)CDATAL);
        //wire_endTransmission();
        //wire_requestFrom(I2C_ADDR, sizeof(raw_data));
        for (uint8_t i = 0; i < sizeof(raw_data); i++)
            raw_data.raw[i] = 0x00;
      //      raw_data.raw[i] = wire_read();
}

void interrupt(bool b)
{
        volatile uint8_t r = read8(reg.ENABLE);
        if(b)
        {
           r |= (uint8_t)mask.ENABLE_AIEN;
        }

        else
        {
            r &= ~(uint8_t)mask.ENABLE_AIEN;
        }
        write8(reg.ENABLE, r);
}

void persistence(uint16_t data)
{
    uint16_t test = data;
    write8(reg.PERS, data);
}

void power(bool b)
{
        if (b)
        {
            write8(reg.ENABLE, (uint8_t)mask.ENABLE_PON);
            //delay(3); // 2.4 ms must pass after PON is asserted before an RGBC can be initiated
            write8(reg.ENABLE, (uint8_t)mask.ENABLE_PON | (uint8_t)mask.ENABLE_AEN);
        }
        else
        {
            uint8_t val = read8(reg.ENABLE);
            write8(reg.ENABLE, val & ~((uint8_t)mask.ENABLE_PON | (uint8_t)mask.ENABLE_AEN));
        }
}

bool attach(/*WireType& w = Wire*/)
{
        //wire = &w;
        uint8_t x = read8(reg.ID);
        if (x != ID_REG_PART_NUMBER) return false;

        power(true);
        interrupt(true);   // use to detect availability (available())
        persistence(0x00); // every RGBC cycle generates an interrupt

        return true;
}
//
//
void enableColorTempAndLuxCalculation(bool b)
{
    bool b_ct_lux_calc = b;
}
//
void integrationTime(volatile float ms) // 2.4 - 614.4 ms
{
        if (ms < INTEGRATION_TIME_MS_MIN) ms = INTEGRATION_TIME_MS_MIN;
        if (ms > INTEGRATION_TIME_MS_MAX) ms = INTEGRATION_TIME_MS_MAX;
        uint8_t data = (uint8_t)(256.f - ms / INTEGRATION_TIME_MS_MIN);
        write8(reg.ATIME, data);
        atime = data;
        //integration_time = ms;
        integration_time = ms;
}
//
void gain(/*Gain g*/)
{
        //write8(CONTROL, (uint8_t)g);
        //switch (g)
        //{
            //case Gain::X01: gain_value =  1.f; break;
            //case Gain::X04: gain_value =  4.f; break;
            //case Gain::X16: gain_value = 16.f; break;
            //case Gain::X60: gain_value = 60.f; break;
            //default:        gain_value =  1.f; break;
        //}
}
//
void scale(float s) {
    scaling = s;
}
//
    //// The Glass Attenuation (FA) factor used to compensate for lower light
    //// levels at the device due to the possible presence of glass. The GA is
    //// the inverse of the glass transmissivity (T), so GA = 1/T. A transmissivity
    //// of 50% gives GA = 1 / 0.50 = 2. If no glass is present, use GA = 1.
    //// See Application Note: DN40-Rev 1.0 â€“ Lux and CCT Calculations using
    //// ams Color Sensors for more details.

void glassAttenuation(float v) { if (v < 1.f) v = 1.f; glass_attenuation = v; }

void clearInterrupt()
{
        //wire->beginTransmission(I2C_ADDR);
        //wire->write(COMMAND_BIT | 0x66);
        //wire->endTransmission();
}

void calcTemperatureAndLuxDN40()
{
        //// Device specific values (DN40 Table 1 in Appendix I)
        const float GA = glass_attenuation;        // Glass Attenuation Factor
        static const float DF = 310.f;             // Device Factor
        static const float R_Coef = 0.136f;        //
        static const float G_Coef = 1.f;           // used in lux computation
        static const float B_Coef = -0.444f;       //
        static const float CT_Coef = 3810.f;       // Color Temperature Coefficient
        static const float CT_Offset = 1391.f;     // Color Temperatuer Offset

        // Analog/Digital saturation (DN40 3.5)
        float saturation = (256 - atime > 63) ? 65535 : 1024 * (256 - atime);

        // Ripple saturation (DN40 3.7)
        if (integration_time < 150)
            saturation -= saturation / 4;

        // Check for saturation and mark the sample as invalid if true
        if (raw_data.c >= saturation)
            return;

        // IR Rejection (DN40 3.1)
        float sum = raw_data.r + raw_data.g + raw_data.b;
        float c = raw_data.c;
        float ir = (sum > c) ? ((sum - c) / 2.f) : 0.f;
        float r2 = raw_data.r - ir;
        float g2 = raw_data.g - ir;
        float b2 = raw_data.b - ir;

        // Lux Calculation (DN40 3.2)
        float g1 = R_Coef * r2 + G_Coef * g2 + B_Coef * b2;
        float cpl = (integration_time * gain_value) / (GA * DF);
        lx = g1 / cpl;

        // CT Calculations (DN40 3.4)
        color_temp = CT_Coef * b2 / r2 + CT_Offset;
}

bool available()
{
        bool b = read8(reg.STATUS) & (uint8_t)mask.STATUS_AINT;
        if (b)
        {
            update();
            if (b_ct_lux_calc) calcTemperatureAndLuxDN40();
            clearInterrupt();
        }
        return b;
}
//
float pow(float x, int n) {
    float result = 1;
    int minus = 1;

    if (n < 0) {
        minus = -1;
        n = -n;
    }

    if (0 == n) {
        return 1;
    } else if (0 == x) {
        return 0;
    }

    while (n) {
        if (n & 1) {
            result *= x;
        }
        x *= x;
        n /= 2;
    }

    if (minus < 0) {
        return 1.0 / result;
    } else {
        return result;
    }
}

void color()
{
        struct Color clr;
        if (raw_data.c == 0) clr.r = clr.g = clr.b = 0;
        else
        {
            clr.r = pow((float)raw_data.r / (float)raw_data.c, scaling) * 255.f;
            clr.g = pow((float)raw_data.g / (float)raw_data.c, scaling) * 255.f;
            clr.b = pow((float)raw_data.b / (float)raw_data.c, scaling) * 255.f;
            if (clr.r > 255.f) clr.r = 255.f;
            if (clr.g > 255.f) clr.g = 255.f;
            if (clr.b > 255.f) clr.b = 255.f;
        }
        //return clr;
}

//const RawData& raw() const { return raw_data; }
float lux() { return lx; }
float colorTemperature() { return color_temp; }
//
//
//
//
//
uint16_t read16(/*Reg reg*/)
{
        uint16_t x = 0x00;
        uint16_t t = 0x00;
//
        //wire_beginTransmission(I2C_ADDR);
        //wire_write(COMMAND_BIT | (uint8_t)reg);
        //wire_endTransmission();
//
        //wire_requestFrom(I2C_ADDR, (uint8_t)2);
        //t = wire_read();
        //x = wire_read();
        x <<= 8;
        x |= t;
        return x;
}
//
//
    //// https://github.com/adafruit/Adafruit_CircuitPython_TCS34725/blob/master/adafruit_tcs34725.py
//
//

//#ifdef TEENSYDUINO
//using TCS34725 = TCS34725_<i2c_t3>;
//#else
//using TCS34725 = TCS34725_<TwoWire>;
//#endif

#endif // TCS34725_H
