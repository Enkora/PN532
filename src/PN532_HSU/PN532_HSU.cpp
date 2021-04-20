
#include "PN532_HSU.h"
#include <HardwareSerial.h>
#include "PN532/PN532_debug.h"

HardwareSerial MySerial(1);

PN532_HSU::PN532_HSU(int8_t tx, int8_t rx)
{
    _serial = &MySerial;
    command = 0;
    tx_pin = tx;
    rx_pin = rx;
}

void PN532_HSU::begin()
{
    _serial->begin(115200, SERIAL_8N1, tx_pin, rx_pin);
}

void PN532_HSU::wakeup()
{
    _serial->write(0x55);
    _serial->write(0x55);
    _serial->write(uint8_t(0x00));
    _serial->write(uint8_t(0x00));
    _serial->write(uint8_t(0x00));

    /** dump serial buffer */
    if (_serial->available()) {
        DMSG("Dump serial buffer: ");
    }
    while (_serial->available()) {
        uint8_t ret = _serial->read();
        DMSG_HEX(ret);
    }
}

int8_t PN532_HSU::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{

    /** dump serial buffer */
    if (_serial->available())
    {
        DMSG("Dump serial buffer: ");
    }
    while (_serial->available())
    {
        uint8_t ret = _serial->read();
        DMSG_HEX(ret);
    }

    command = header[0];

    _serial->write(uint8_t(PN532_PREAMBLE));
    _serial->write(uint8_t(PN532_STARTCODE1));
    _serial->write(uint8_t(PN532_STARTCODE2));

    uint8_t length = hlen + blen + 1; // length of data field: TFI + DATA
    _serial->write(length);
    _serial->write(~length + 1); // checksum of length

    _serial->write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532; // sum of TFI + DATA

    DMSG("\nWrite: ");

    _serial->write(header, hlen);
    for (uint8_t i = 0; i < hlen; i++)
    {
        sum += header[i];

        DMSG_HEX(header[i]);
    }

    _serial->write(body, blen);
    for (uint8_t i = 0; i < blen; i++)
    {
        sum += body[i];

        DMSG_HEX(body[i]);
    }

    uint8_t checksum = ~sum + 1; // checksum of TFI + DATA
    _serial->write(checksum);
    _serial->write(uint8_t(PN532_POSTAMBLE));

    DMSG("\n");
    return readAckFrame();
}

int16_t PN532_HSU::readResponse(uint8_t command, uint8_t buf[], uint8_t len, uint16_t timeout) {
    return readResponse(buf, len, timeout);
}

int16_t PN532_HSU::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout) {
    uint8_t tmp[3];
    DMSG("Read:  ");

    /** Frame Preamble and Start Code */
    if (receive(tmp, 3, timeout) <= 0) {
        DMSG("Timeout 1");
        return PN532_TIMEOUT;
    }
    if (0 != tmp[0] || 0 != tmp[1] || 0xFF != tmp[2]) {
        DMSG("Preamble error");
        return PN532_INVALID_FRAME;
    }

    /** receive length and check */
    uint8_t length[2];
    if (receive(length, 2, timeout) <= 0) {
        DMSG("Timeout 2");
        return PN532_TIMEOUT;
    }
    if (0 != (uint8_t)(length[0] + length[1]))
    {
        DMSG("Length error");
        return PN532_INVALID_FRAME;
    }
    length[0] -= 2;
    if (length[0] > len) {
        DMSG("No space error");
        return PN532_NO_SPACE;
    }

    /** receive command byte */
    uint8_t cmd = command + 1; // response command
    if (receive(tmp, 2, timeout) <= 0) {
        DMSG("Timeout 3");
        return PN532_TIMEOUT;
    }
    if (PN532_PN532TOHOST != tmp[0] || cmd != tmp[1]) {
        DMSG("Command error");
        return PN532_INVALID_FRAME;
    }

    if (receive(buf, length[0], timeout) != length[0]) {
        DMSG("Timeout 4");
        return PN532_TIMEOUT;
    }
    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length[0]; i++) {
        sum += buf[i];
    }

    /** checksum and postamble */
    if (receive(tmp, 2, timeout) <= 0) {
        DMSG("Timeout 5");
        return PN532_TIMEOUT;
    }

    if (0 != (uint8_t)(sum + tmp[0]) || 0 != tmp[1]) {
        DMSG("Checksum error");
        return PN532_INVALID_FRAME;
    }

    return length[0];
}

int8_t PN532_HSU::readAckFrame() {
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
    uint8_t ackBuf[sizeof(PN532_ACK)];

    DMSG("Ack: ");

    if (receive(ackBuf, sizeof(PN532_ACK), PN532_ACK_WAIT_TIME) <= 0) {
        DMSG("Timeout\n");
        return PN532_TIMEOUT;
    }

    if (memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK))) {
        DMSG("Invalid\n");
        return PN532_INVALID_ACK;
    }
    return 0;
}

/**
    @brief receive data .
    @param buf --> return value buffer.
           len --> length expect to receive.
           timeout --> time of reveiving
    @retval number of received bytes, 0 means no data received.
*/
int8_t PN532_HSU::receive(uint8_t *buf, int len, uint16_t timeout) {
    int read_bytes = 0;
    int ret;
    unsigned long start_millis;

    while (read_bytes < len) {
        start_millis = millis();
        do {
            ret = _serial->read();
            if (ret >= 0) break;
        } while ((timeout == 0) || ((millis() - start_millis) < timeout));

        if (ret < 0) return read_bytes ? read_bytes : PN532_TIMEOUT;

        buf[read_bytes] = (uint8_t) ret;
        DMSG_HEX(ret);
        read_bytes++;
    }
    DMSG("\n");
    return read_bytes;
}

/*
uint8_t PN532_HSU::RequestFrom(uint8_t u8_Quantity) {
    return;

    const int BUFFER_SIZE = u8_Quantity;
    char buf[BUFFER_SIZE];
    rlen = Serial.readBytes(buf, u8_Quantity);

    if (rlen = u8_Quantity) return buf;
    return
    return _wire->requestFrom(PN532_I2C_ADDRESS, u8_Quantity);

 }

int PN532_HSU::Read() {
    return _serial->read();
}

void PN532_HSU::BeginTransmission(uint8_t u8_Address) {
    return;
}

void PN532_HSU::Write(uint8_t u8_Data) {
    write(u8_Data);
}

void PN532_HSU::EndTransmission() {
    return;
}
*/
