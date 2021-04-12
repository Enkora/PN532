/**
 * @modified picospuch
 */

#include "PN532_I2C.h"
#include "PN532/PN532_debug.h"
#include "Arduino.h"

#define PN532_I2C_ADDRESS (0x48 >> 1)

PN532_I2C::PN532_I2C(TwoWire &wire)
{
    _wire = &wire;
    command_x = 0;
}

void PN532_I2C::begin()
{
    _wire->begin(21,22);
}

void PN532_I2C::wakeup()
{
    delay(500); // wait for all ready to manipulate pn532
}

uint8_t PN532_I2C::RequestFrom(uint8_t u8_Quantity)
{
    return _wire->requestFrom(PN532_I2C_ADDRESS, u8_Quantity);
}

// Read one uint8_t from the buffer that has been read when calling RequestFrom()
int PN532_I2C::Read()
{
    return _wire->read();
}
// --------------------- WRITE -------------------------

// Initiates a Send transmission
void PN532_I2C::BeginTransmission(uint8_t u8_Address)
{
    Wire.beginTransmission(u8_Address);
}

// Write one uint8_t to the I2C bus
void PN532_I2C::Write(uint8_t u8_Data)
{
    Wire.write(u8_Data);
}

// Ends a Send transmission
void PN532_I2C::EndTransmission()
{
    Wire.endTransmission();
}

int8_t PN532_I2C::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command_x = header[0];
    _wire->beginTransmission(PN532_I2C_ADDRESS);

    Serial.print(String("Sending: "));
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    write(length);
    write(~length + 1); // checksum of length

    write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("write: ");

    for (uint8_t i = 0; i < hlen; i++)
    {
        if (write(header[i]))
        {
            sum += header[i];
            //DMSG_HEX(header[i]);
        }
        else
        {
            DMSG("\nToo many data to send, I2C doesn't support such a big packet\n"); // I2C max packet: 32 bytes
            return PN532_INVALID_FRAME;
        }
    }

    for (uint8_t i = 0; i < blen; i++)
    {
        if (write(body[i]))
        {
            sum += body[i];

            DMSG_HEX(body[i]);
        }
        else
        {
            DMSG("\nToo many data to send, I2C doesn't support such a big packet\n"); // I2C max packet: 32 bytes
            return PN532_INVALID_FRAME;
        }
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);

    _wire->endTransmission();

    DMSG('\n');

    return readAckFrame();
}

int16_t PN532_I2C::getResponseLength(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    const uint8_t PN532_NACK[] = {0, 0, 0xFF, 0xFF, 0, 0};
    uint16_t time = 0;

    do
    {
        if (_wire->requestFrom(PN532_I2C_ADDRESS, 6))
        {
            if (read() & 1)
            {          // check first uint8_t --- status
                break; // PN532 is ready
            }
        }

        delay(1);
        time++;
        if ((0 != timeout) && (time > timeout))
        {
            return -1;
        }
    } while (1);

    if (0x00 != read() || // PREAMBLE
        0x00 != read() || // STARTCODE1
        0xFF != read()    // STARTCODE2
    )
    {

        return PN532_INVALID_FRAME;
    }

    uint8_t length = read();

    // request for last respond msg again
    _wire->beginTransmission(PN532_I2C_ADDRESS);
    for (uint16_t i = 0; i < sizeof(PN532_NACK); ++i)
    {
        write(PN532_NACK[i]);
    }
    _wire->endTransmission();

    return length;
}

int16_t PN532_I2C::readResponse(uint8_t command, uint8_t buf[], uint8_t len, uint16_t timeout) {
    uint16_t time = 0;
    uint8_t length;

    length = getResponseLength(buf, len, timeout);
    Serial.println(String("length: ") + String(length) + ", len: " + String(len));

    // [RDY] 00 00 FF LEN LCS (TFI PD0 ... PDn) DCS 00
    do
    {
        if (_wire->requestFrom(PN532_I2C_ADDRESS, 6 + length + 2))
        {
            if (read() & 1)
            {          // check first uint8_t --- status
                break; // PN532 is ready
            }
        }

        delay(1);
        time++;
        if ((0 != timeout) && (time > timeout))
        {
            return -1;
        }
    } while (1);

    if (0x00 != read() || // PREAMBLE
        0x00 != read() || // STARTCODE1
        0xFF != read()    // STARTCODE2
    )
    {
        DMSG("PN532_INVALID_FRAME");
        return PN532_INVALID_FRAME;
    }

    length = read();
    Serial.println(String("length: ") + String(length) + ", len: " + String(len));

    if (0 != (uint8_t)(length + read()))
    { // checksum of length
        DMSG("2 PN532_INVALID_FRAME");
        return PN532_INVALID_FRAME;
    }

    uint8_t cmd = command + 1; // response command
    uint8_t read_1 = read();
    uint8_t read_2 = read();
    if (PN532_PN532TOHOST != read_1 || (cmd) != read_2)
    {
        Serial.println("3 PN532_INVALID_FRAME");
        if (PN532_PN532TOHOST != read_1)    Serial.println(String("PN532_PN532TOHOST [") + String(PN532_PN532TOHOST, HEX) + "] != read_1 [" + String(read_1, HEX) + "]");
        if ((cmd) != read_2)                Serial.println(String("(cmd) [") + String((cmd), HEX) + "] != read_2 [" + String(read_2, HEX) + "]");
        return PN532_INVALID_FRAME;
    }
    buf[0] = PN532_PN532TOHOST;
    buf[1] = (cmd);

    Serial.println(String("length: ") + String(length) + ", len: " + String(len));

    if (length > len)
    {
        DMSG("PN532_NO_SPACE");
        return PN532_NO_SPACE; // not enough space
    }

    //DMSG("read:  ");
    //DMSG_HEX(cmd);

    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 2; i < length; i++)
    {
        buf[i] = read();
        sum += buf[i];

        //DMSG_HEX(buf[i]);
    }
    //DMSG('\n');

    uint8_t checksum = read();
    if (0 != (uint8_t)(sum + checksum))
    {
        DMSG("checksum is not ok\n");
        return PN532_INVALID_FRAME;
    }
    read(); // POSTAMBLE

    Serial.println(String("length: ") + String(length) + ", len: " + String(len));
    return length;
}

int8_t PN532_I2C::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
    uint8_t ackBuf[sizeof(PN532_ACK)];

    //DMSG("wait for ack at : ");
    //DMSG(millis());
    //DMSG('\n');

    uint16_t time = 0;
    do
    {
        if (_wire->requestFrom(PN532_I2C_ADDRESS, sizeof(PN532_ACK) + 1))
        {
            if (read() & 1)
            {          // check first uint8_t --- status
                break; // PN532 is ready
            }
        }

        delay(1);
        time++;
        if (time > PN532_ACK_WAIT_TIME)
        {
            DMSG("Time out when waiting for ACK\n");
            return PN532_TIMEOUT;
        }
    } while (1);

    //DMSG("ready at : ");
    //DMSG(millis());
    //DMSG('\n');

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++)
    {
        ackBuf[i] = read();
    }

    if (memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK)))
    {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }

    return 0;
}
