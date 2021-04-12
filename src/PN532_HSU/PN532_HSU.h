
#ifndef __PN532_HSU_H__
#define __PN532_HSU_H__

#include "PN532/PN532Interface.h"
#include "Arduino.h"

#define PN532_HSU_DEBUG

#define PN532_HSU_READ_TIMEOUT (1000)

class PN532_HSU : public PN532Interface
{
public:
    PN532_HSU(int8_t tx, int8_t rx);

    void begin();
    void wakeup();
    virtual int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);
    int16_t readResponse(uint8_t command, uint8_t buf[], uint8_t len, uint16_t timeout);

    /*
    virtual uint8_t RequestFrom(uint8_t u8_Quantity);
    virtual int Read();
    virtual void BeginTransmission(uint8_t u8_Address);
    virtual void Write(uint8_t u8_Data);
    virtual void EndTransmission();
*/

private:
    HardwareSerial *_serial;
    uint8_t command;
    int8_t tx_pin;
    int8_t rx_pin;

    int8_t readAckFrame();
    int8_t receive(uint8_t *buf, int len, uint16_t timeout = PN532_HSU_READ_TIMEOUT);
};

#endif
