/**
 * @modified picospuch
 */

#ifndef __PN532_I2C_H__
#define __PN532_I2C_H__

#include <Arduino.h>
#include <Wire.h>
#include "PN532Interface.h"
#include "PN532_debug.h"

class PN532_I2C : public PN532Interface {
public:
    PN532_I2C(TwoWire &wire);

    void begin();
    void wakeup();
    virtual int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int16_t readResponse(uint8_t command, uint8_t buf[], uint8_t len, uint16_t timeout);

    uint8_t RequestFrom(uint8_t u8_Quantity);
    int Read();
    void BeginTransmission(uint8_t u8_Address);
    void Write(uint8_t u8_Data);
    void EndTransmission();

private:
    TwoWire *_wire;
    uint8_t command_x;

    int8_t readAckFrame();
    int16_t getResponseLength(uint8_t buf[], uint8_t len, uint16_t timeout);

    inline uint8_t write(uint8_t data)
    {
        DMSG_HEX(data);
#if ARDUINO >= 100
        return _wire->write(data);
#else
        return _wire->send(data);
#endif
    }

    inline uint8_t read() {
#if ARDUINO >= 100
        uint8_t data = _wire->read();
#else
        uint8_t data = _wire->receive();
#endif
        DMSG_HEX(data);
        return data;
    }
};

#endif
