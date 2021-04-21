/**************************************************************************/
/*!
    @file     PN532.cpp
    @author   Adafruit Industries & Seeed Studio
    @license  BSD
*/
/**************************************************************************/

#include "Arduino.h"
#include "PN532.h"
#include "PN532_debug.h"
#include "Secrets.h"
#include <string.h>
#include "Utils.h"

#define HAL(func)   (_interface->func)

#define FELICA false
#define PN532_I2C_ADDRESS (0x48 >> 1)
#define PN532_I2C_READY 0x01
#define PN532_I2C_TIMEOUT  1000

PN532::PN532(PN532Interface &interface) : mi_CmacBuffer(mu8_CmacBuffer_Data, sizeof(mu8_CmacBuffer_Data)) {
    _interface = &interface;
    mpi_SessionKey = NULL;
    mu8_LastAuthKeyNo = NOT_AUTHENTICATED;
    mu8_LastPN532Error = 0;
    mu32_LastApplication = 0x000000; // No application selected

    // The PICC master key on an empty card is a simple DES key filled with 8 zeros
    const uint8_t ZERO_KEY[24] = {0};
    DES2_DEFAULT_KEY.SetKeyData(ZERO_KEY, 8, 0); // simple DES
    DES3_DEFAULT_KEY.SetKeyData(ZERO_KEY, 24, 0); // triple DES
    AES_DEFAULT_KEY.SetKeyData(ZERO_KEY, 16, 0);
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void PN532::begin() {
    HAL(begin)();
    HAL(wakeup)();
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the uint8_t data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void PN532::PrintHex(const uint8_t *data, const uint32_t numBytes) {
#ifdef ARDUINO
    for (uint8_t i = 0; i < numBytes; i++) {
        if (data[i] < 0x10) {
            DEBUG_SERIAL.print(" 0");
        } else {
            DEBUG_SERIAL.print(' ');
        }
        DEBUG_SERIAL.print(data[i], HEX);
    }
    DEBUG_SERIAL.println("");
#else
    for (uint8_t i = 0; i < numBytes; i++) {
        printf(" %2X", data[i]);
    }
    printf("\n");
#endif
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters, along with
            the char equivalents in the following format

            00 00 00 00 00 00  ......

    @param  data      Pointer to the data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void PN532::PrintHexChar(const uint8_t *data, const uint32_t numBytes) {
#ifdef ARDUINO
    for (uint8_t i = 0; i < numBytes; i++) {
        if (data[i] < 0x10) {
            DEBUG_SERIAL.print(" 0");
        } else {
            DEBUG_SERIAL.print(' ');
        }
        DEBUG_SERIAL.print(data[i], HEX);
    }
    DEBUG_SERIAL.print("    ");
    for (uint8_t i = 0; i < numBytes; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) {
            DEBUG_SERIAL.print('.');
        } else {
            DEBUG_SERIAL.print(c);
        }
    }
    DEBUG_SERIAL.println("");
#else
    for (uint8_t i = 0; i < numBytes; i++) {
        printf(" %2X", data[i]);
    }
    printf("    ");
    for (uint8_t i = 0; i < numBytes; i++) {
        char c = data[i];
        if (c <= 0x1f || c > 0x7f) {
            printf(".");
        } else {
            printf("%c", c);
        }
        printf("\n");
    }
#endif
}

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t PN532::getFirmwareVersion(void) {
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    if (HAL(writeCommand)(pn532_packetbuffer, 1)) {
        Serial.println("write failed");
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(PN532_COMMAND_GETFIRMWAREVERSION, pn532_packetbuffer,
                                       sizeof(pn532_packetbuffer));
    if (0 > status) {
        Serial.println("read failed");
        return 0;
    }

    response = pn532_packetbuffer[0];
    response <<= 8;
    response |= pn532_packetbuffer[1];
    response <<= 8;
    response |= pn532_packetbuffer[2];
    response <<= 8;
    response |= pn532_packetbuffer[3];

    return response;
}


/**************************************************************************/
/*!
    @brief  Read a PN532 register.

    @param  reg  the 16-bit register address.

    @returns  The register value.
*/
/**************************************************************************/
uint32_t PN532::readRegister(uint16_t reg) {
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_READREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(PN532_COMMAND_READREGISTER, pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (0 > status) {
        return 0;
    }

    response = pn532_packetbuffer[0];

    return response;
}

/**************************************************************************/
/*!
    @brief  Write to a PN532 register.

    @param  reg  the 16-bit register address.
    @param  val  the 8-bit value to write.

    @returns  0 for failure, 1 for success.
*/
/**************************************************************************/
uint32_t PN532::writeRegister(uint16_t reg, uint8_t val) {
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;
    pn532_packetbuffer[3] = val;


    if (HAL(writeCommand)(pn532_packetbuffer, 4)) {
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(PN532_COMMAND_WRITEREGISTER, pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (0 > status) {
        return 0;
    }

    return 1;
}

/**************************************************************************/
/*!
    Writes an 8-bit value that sets the state of the PN532's GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::writeGPIO(uint8_t pinstate) {
    // Make sure pinstate does not try to toggle P32 or P34
    pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

    // Fill command buffer
    pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
    pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
    pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by I2C)

    DMSG("Writing P3 GPIO: ");
    DMSG_HEX(pn532_packetbuffer[1]);
    DMSG("\n");

    // Send the WRITEGPIO command (0x0E)
    if (HAL(writeCommand)(pn532_packetbuffer, 3))
        return 0;

    return (0 < HAL(readResponse)(PN532_COMMAND_WRITEGPIO, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Reads the state of the PN532's GPIO pins

    @returns An 8-bit value containing the pin state where:

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
uint8_t PN532::readGPIO(void) {
    pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

    // Send the READGPIO command (0x0C)
    if (HAL(writeCommand)(pn532_packetbuffer, 1))
        return 0x0;

    HAL(readResponse)(PN532_COMMAND_READGPIO, pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* READGPIO response without prefix and suffix should be in the following format:

      byte            Description
      -------------   ------------------------------------------
      b0              P3 GPIO Pins
      b1              P7 GPIO Pins (not used ... taken by I2C)
      b2              Interface Mode Pins (not used ... bus select pins)
    */


    DMSG("P3 GPIO: "); DMSG_HEX(pn532_packetbuffer[7]);
    DMSG("P7 GPIO: "); DMSG_HEX(pn532_packetbuffer[8]);
    DMSG("I0I1 GPIO: "); DMSG_HEX(pn532_packetbuffer[9]);
    DMSG("\n");

    return pn532_packetbuffer[0];
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool PN532::SAMConfig(void) {
    pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode;
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x01; // use IRQ pin!

    DMSG("SAMConfig\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 4))
        return false;

    return (0 < HAL(readResponse)(PN532_COMMAND_SAMCONFIGURATION, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    @brief  Turn the module into power mode  will wake up on I2C or SPI request
*/
/**************************************************************************/
bool PN532::powerDownMode() {
    pn532_packetbuffer[0] = PN532_COMMAND_POWERDOWN;
    pn532_packetbuffer[1] = 0xC0; // I2C or SPI Wakeup
    pn532_packetbuffer[2] = 0x00; // no IRQ

    DMSG("POWERDOWN\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 4))
        return false;

    return (0 < HAL(readResponse)(PN532_COMMAND_POWERDOWN, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::setPassiveActivationRetries(uint8_t maxRetries) {
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
    pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
    pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
    pn532_packetbuffer[4] = maxRetries;

    if (HAL(writeCommand)(pn532_packetbuffer, 5))
        return 0x0;  // no ACK

    return (0 < HAL(readResponse)(PN532_COMMAND_RFCONFIGURATION, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Sets the RFon/off uint8_t of the RFConfiguration register

    @param  autoRFCA    0x00 No check of the external field before
                        activation

                        0x02 Check the external field before
                        activation

    @param  rFOnOff     0x00 Switch the RF field off, 0x01 switch the RF
                        field on

    @returns    1 if everything executed properly, 0 for an error
*/
/**************************************************************************/

bool PN532::setRFField(uint8_t autoRFCA, uint8_t rFOnOff) {
    if (rFOnOff == 0) { // if RF field is switched off
        mu8_LastAuthKeyNo = NOT_AUTHENTICATED;
        mu32_LastApplication = 0x000000; // No application selected
    }

    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0x00 | autoRFCA | rFOnOff;

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return 0x0;  // command failed
    }

    return (0 < HAL(readResponse)(PN532_COMMAND_RFCONFIGURATION, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.
    @param  timeout       The number of tries before timing out
    @param  inlist        If set to true, the card will be inlisted

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout, bool inlist) {
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    pn532_packetbuffer[2] = cardbaudrate;

    /*
    DMSG_STR("readPassiveTargetID writeCommand");
    for (int i = 0; i < 16; i++) {
        DMSG_HEX(pn532_packetbuffer[i]);
    }
*/

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        Serial.println("writeCommand failed");
        return false;  // command failed
    }

    Serial.print("pn532_packetbuffer after write command: ");
    for (int i = 0; i < 16; i++) {
        DMSG_HEX(pn532_packetbuffer[i]);
    }
    DMSG("\n");

    if (0 >
        HAL(readResponse)(PN532_COMMAND_INLISTPASSIVETARGET, pn532_packetbuffer, sizeof(pn532_packetbuffer), timeout)) {
        Serial.println("readResponse failed");
        return false;
    }

    /* ISO14443A card response should be in the following format:
      byte            Description
      -------------   ------------------------------------------
      b0              Tags Found
      b1              Tag Number (only one used in this example)
      b2..3           SENS_RES (ATQA = Answer to Request Type A)
      b4              SEL_RES (SAK  = Select Acknowledge)
      b5              NFCID Length
      b6..NFCIDLen    NFCID
      nn              ATS Length     (Desfire only)
      nn..Length-1    ATS data bytes (Desfire only)
    */

    // if no tags were found
    if (pn532_packetbuffer[2] != 1) {
        PRINT_DEBUG("pn532_packetbuffer[2] failed");
        return false;
    }

    int uidLen = pn532_packetbuffer[7];
    *uidLength = uidLen;

    for (uint8_t i = 0; i < uidLen; i++) {
        uid[i] = pn532_packetbuffer[8 + i];
    }

    // ATQA and SAK codes can be found here: https://www.nxp.com/docs/en/application-note/AN10833.pdf
    uint16_t ATQA = ((uint16_t) pn532_packetbuffer[4] << 8) | pn532_packetbuffer[5];
    byte SAK = pn532_packetbuffer[6];

    DMSG("ATQA: 0x");  DMSG_HEX(ATQA);
    DMSG("SAK: 0x");  DMSG_HEX(SAK);
    DMSG("\n");

#ifdef DEBUG
    Serial.println(uidLen);
    Serial.println(uid[0]);
    Serial.println(ATQA);
    Serial.println(SAK);

    if (uidLen == 7 && uid[0] != 0x80 && ATQA == 0x68 && SAK == 0x0) {
        Serial.println("Ultralight");
    }

    if (uidLen == 4 && uid[0] != 0x80 && ATQA == 0x4 && SAK == 0x8) {
        Serial.println("Classic 1K");
    }

    if (uidLen == 4 && uid[0] != 0x80 && ATQA == 0x2 && SAK == 0x18) {
        Serial.println("Classic 4K");
    }

    if (uidLen == 7 && uid[0] != 0x80 && ATQA == 0x0344 && SAK == 0x20) {
        Serial.println("DESFire");
    }

    if (uidLen == 4 && uid[0] == 0x80 && ATQA == 0x0304 && SAK == 0x20) {
        Serial.println("DESFIRE RANDOM");
    }
#endif

    if (inlist) {
        inListedTag = pn532_packetbuffer[3];
    }

    return 1;
}

/**************************************************************************
    Waits for an ISO14443A target to enter the field.
    If the RF field has been turned off before, this command switches it on.

    param u8_UidBuffer  Pointer to an 8 byte buffer that will be populated with the card's UID (4 or 7 bytes)
    param pu8_UidLength Pointer to the variable that will hold the length of the card's UID.
    param pe_CardType   Pointer to the variable that will hold if the card is a Desfire card

    returns false only on error!
    returns true and *UidLength = 0 if no card was found
    returns true and *UidLength > 0 if a card has been read successfully
**************************************************************************/
bool PN532::ReadPassiveTargetID(byte *u8_UidBuffer, byte *pu8_UidLength, eCardType *pe_CardType) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** ReadPassiveTargetID()\r\n");

    *pu8_UidLength = 0;
    *pe_CardType = CARD_Unknown;
    memset(u8_UidBuffer, 0, 8);

    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // read data of 1 card (The PN532 can read max 2 targets at the same time)
    pn532_packetbuffer[2] = CARD_TYPE_106KB_ISO14443A; // This function currently does not support other card types.

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return false; // Error (no valid ACK received or timeout)
    }


    /*
    ISO14443A card response:
    pn532_packetbuffer Description
    -------------------------------------------------------
    b0               D5 (always) (PN532_PN532TOHOST)
    b1               4B (always) (PN532_COMMAND_INLISTPASSIVETARGET + 1)
    b2               Amount of cards found
    b3               Tag number (always 1)
    b4,5             SENS_RES (ATQA = Answer to Request Type A)
    b6               SEL_RES  (SAK  = Select Acknowledge)
    b7               UID Length
    b8..Length       UID (4 or 7 bytes)
    nn               ATS Length     (Desfire only)
    nn..Length-1     ATS data bytes (Desfire only)
    */

    if (0 > HAL(readResponse)(PN532_COMMAND_INLISTPASSIVETARGET, pn532_packetbuffer, sizeof(pn532_packetbuffer))) {
        Utils::Print("ReadPassiveTargetID failed\r\n");
        return false;
    }

    byte cardsFound = pn532_packetbuffer[2];
    if (mu8_DebugLevel > 0) {
        Utils::Print("Cards found: ");
        Utils::PrintDec(cardsFound, LF);
    }
    if (cardsFound != 1)
        return true; // no card found -> this is not an error!

    byte u8_IdLength = pn532_packetbuffer[7];
    if (u8_IdLength != 4 && u8_IdLength != 7) {
        Utils::Print("Card has unsupported UID length: ");
        Utils::PrintDec(u8_IdLength, LF);
        return true; // unsupported card found -> this is not an error!
    }

    memcpy(u8_UidBuffer, pn532_packetbuffer + 8, u8_IdLength);
    *pu8_UidLength = u8_IdLength;

    // See "Mifare Identification & Card Types.pdf" in the ZIP file
    uint16_t u16_ATQA = ((uint16_t) pn532_packetbuffer[4] << 8) | pn532_packetbuffer[5];
    byte u8_SAK = pn532_packetbuffer[6];

    if (u8_IdLength == 7 && u8_UidBuffer[0] != 0x80 && u16_ATQA == 0x0344 && u8_SAK == 0x20)
        *pe_CardType = CARD_Desfire;
    if (u8_IdLength == 4 && u8_UidBuffer[0] == 0x80 && u16_ATQA == 0x0304 && u8_SAK == 0x20)
        *pe_CardType = CARD_DesRandom;

    if (mu8_DebugLevel > 0) {
        Utils::Print("Card UID:    ");
        Utils::PrintHexBuf(u8_UidBuffer, u8_IdLength, LF);

        // Examples:              ATQA    SAK  UID length
        // MIFARE Mini            00 04   09   4 bytes
        // MIFARE Mini            00 44   09   7 bytes
        // MIFARE Classic 1k      00 04   08   4 bytes
        // MIFARE Classic 4k      00 02   18   4 bytes
        // MIFARE Ultralight      00 44   00   7 bytes
        // MIFARE DESFire Default 03 44   20   7 bytes
        // MIFARE DESFire Random  03 04   20   4 bytes
        // See "Mifare Identification & Card Types.pdf"
        char s8_Buf[80];
        sprintf(s8_Buf, "Card Type:   ATQA= 0x%04X, SAK= 0x%02X", u16_ATQA, u8_SAK);

        if (*pe_CardType == CARD_Desfire) strcat(s8_Buf, " (Desfire Default)");
        if (*pe_CardType == CARD_DesRandom) strcat(s8_Buf, " (Desfire RandomID)");

        Utils::Print(s8_Buf, LF);
    }
    return true;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool PN532::mifareclassic_IsFirstBlock(uint32_t uiBlock) {
    // Test if we are in the small or big sectors
    if (uiBlock < 128)
        return ((uiBlock) % 4 == 0);
    else
        return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool PN532::mifareclassic_IsTrailerBlock(uint32_t uiBlock) {
    // Test if we are in the small or big sectors
    if (uiBlock < 128)
        return ((uiBlock + 1) % 4 == 0);
    else
        return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 bytes
                          key value

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_AuthenticateBlock(uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber,
                                               uint8_t *keyData) {
    uint8_t i;

    // Hang on to the key and uid data
    memcpy(_key, keyData, 6);
    memcpy(_uid, uid, uidLen);
    _uidLen = uidLen;

    // Prepare the authentication command //
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
    pn532_packetbuffer[1] = 1;                              /* Max card numbers */
    pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
    pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
    memcpy(pn532_packetbuffer + 4, _key, 6);
    for (i = 0; i < _uidLen; i++) {
        pn532_packetbuffer[10 + i] = _uid[i];              /* 4 bytes card ID */
    }

    if (HAL(writeCommand)(pn532_packetbuffer, 10 + _uidLen))
        return 0;

    // Read the response packet
    HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer));

    // Check if the response is valid and we are authenticated???
    // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
    // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
    if (pn532_packetbuffer[0] != 0x00) {
        DMSG("Authentification failed\n");
        return 0;
    }

    return 1;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-bytes data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the
                          retrieved data (if any)

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_ReadDataBlock(uint8_t blockNumber, uint8_t *data) {
    DMSG("Trying to read 16 bytes from block ");
    DMSG_INT(blockNumber);

    /* Prepare the command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                      /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 4)) {
        return 0;
    }

    /* Read the response packet */
    HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* If byte 8 isn't 0x00 we probably have an error */
    if (pn532_packetbuffer[0] != 0x00) {
        return 0;
    }

    /* Copy the 16 data bytes to the output buffer        */
    /* Block content starts at byte 9 of a valid response */
    memcpy(data, pn532_packetbuffer + 1, 16);

    return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-bytes data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_WriteDataBlock(uint8_t blockNumber, uint8_t *data) {
    /* Prepare the first command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                      /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
    memcpy(pn532_packetbuffer + 4, data, 16);        /* Data Payload */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 20)) {
        return 0;
    }

    /* Read the response packet */
    return (0 < HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Formats a Mifare Classic card to store NDEF Records

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_FormatNDEF(void) {
    uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1,
                                 0x03, 0xE1};
    uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1,
                                 0x03, 0xE1};
    uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF};

    // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
    // for the MAD sector in NDEF records (sector 0)

    // Write block 1 and 2 to the card
    if (!(mifareclassic_WriteDataBlock(1, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock(2, sectorbuffer2)))
        return 0;
    // Write key A and access rights card
    if (!(mifareclassic_WriteDataBlock(3, sectorbuffer3)))
        return 0;

    // Seems that everything was OK (?!)
    return 1;
}

/**************************************************************************/
/*!
    Writes an NDEF URI Record to the specified sector (1..15)

    Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_WriteNDEFURI(uint8_t sectorNumber, uint8_t uriIdentifier, const char *url) {
    // Figure out how long the string is
    uint8_t len = strlen(url);

    // Make sure we're within a 1K limit for the sector number
    if ((sectorNumber < 1) || (sectorNumber > 15))
        return 0;

    // Make sure the URI payload is between 1 and 38 chars
    if ((len < 1) || (len > 38))
        return 0;

    // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
    // in NDEF records

    // Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
    uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len + 5, 0xD1, 0x01, len + 1, 0x55, uriIdentifier, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00};
    uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00};
    uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF};
    if (len <= 6) {
        // Unlikely we'll get a url this short, but why not ...
        memcpy(sectorbuffer1 + 9, url, len);
        sectorbuffer1[len + 9] = 0xFE;
    } else if (len == 7) {
        // 0xFE needs to be wrapped around to next block
        memcpy(sectorbuffer1 + 9, url, len);
        sectorbuffer2[0] = 0xFE;
    } else if ((len > 7) && (len <= 22)) {
        // Url fits in two blocks
        memcpy(sectorbuffer1 + 9, url, 7);
        memcpy(sectorbuffer2, url + 7, len - 7);
        sectorbuffer2[len - 7] = 0xFE;
    } else if (len == 23) {
        // 0xFE needs to be wrapped around to final block
        memcpy(sectorbuffer1 + 9, url, 7);
        memcpy(sectorbuffer2, url + 7, len - 7);
        sectorbuffer3[0] = 0xFE;
    } else {
        // Url fits in three blocks
        memcpy(sectorbuffer1 + 9, url, 7);
        memcpy(sectorbuffer2, url + 7, 16);
        memcpy(sectorbuffer3, url + 23, len - 23);
        sectorbuffer3[len - 23] = 0xFE;
    }

    // Now write all three blocks back to the card
    if (!(mifareclassic_WriteDataBlock(sectorNumber * 4, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 1, sectorbuffer2)))
        return 0;
    if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 2, sectorbuffer3)))
        return 0;
    if (!(mifareclassic_WriteDataBlock((sectorNumber * 4) + 3, sectorbuffer4)))
        return 0;

    // Seems that everything was OK (?!)
    return 1;
}

/***** Mifare Ultralight Functions ******/

/**************************************************************************/
/*!
    Tries to read an entire 4-bytes page at the specified address.

    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
/**************************************************************************/
uint8_t PN532::mifareultralight_ReadPage(uint8_t page, uint8_t *buffer) {
    /* Prepare the command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                   /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
    pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 4)) {
        return 0;
    }

    /* Read the response packet */
    HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* If byte 8 isn't 0x00 we probably have an error */
    if (pn532_packetbuffer[0] == 0x00) {
        /* Copy the 4 data bytes to the output buffer         */
        /* Block content starts at byte 9 of a valid response */
        /* Note that the command actually reads 16 bytes or 4  */
        /* pages at a time ... we simply discard the last 12  */
        /* bytes                                              */
        memcpy(buffer, pn532_packetbuffer + 1, 4);
    } else {
        return 0;
    }

    // Return OK signal
    return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 4-bytes data buffer at the specified page
    address.

    @param  page     The page number to write into.  (0..63).
    @param  buffer   The byte array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareultralight_WritePage(uint8_t page, uint8_t *buffer) {
    /* Prepare the first command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                           /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_WRITE_ULTRALIGHT; /* Mifare UL Write cmd = 0xA2 */
    pn532_packetbuffer[3] = page;                        /* page Number (0..63) */
    memcpy(pn532_packetbuffer + 4, buffer, 4);          /* Data Payload */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 8)) {
        return 0;
    }

    /* Read the response packet */
    return (0 < HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************/
bool PN532::inDataExchange(uint8_t *send, uint8_t sendLength, uint8_t *response, uint8_t *responseLength) {
    uint8_t i;

    pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = inListedTag;

    if (HAL(writeCommand)(pn532_packetbuffer, 2, send, sendLength)) {
        return false;
    }

    int16_t status = HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, response, *responseLength, 1000);
    if (status < 0) {
        return false;
    }

    if ((response[0] & 0x3f) != 0) {
        DMSG("Status code indicates an error\n");
        return false;
    }

    uint8_t length = status;
    length -= 1;

    if (length > *responseLength) {
        length = *responseLength; // silent truncation...
    }

    for (uint8_t i = 0; i < length; i++) {
        response[i] = response[i + 1];
    }
    *responseLength = length;

    return true;
}

/**************************************************************************/
/*!
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/**************************************************************************/
bool PN532::inListPassiveTarget() {
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0;

    DMSG("inList passive target\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return false;
    }

    int16_t status = HAL(readResponse)(PN532_COMMAND_INLISTPASSIVETARGET, pn532_packetbuffer,
                                       sizeof(pn532_packetbuffer), 30000);
    if (status < 0) {
        return false;
    }

    if (pn532_packetbuffer[0] != 1) {
        return false;
    }

    inListedTag = pn532_packetbuffer[1];

    return true;
}

int8_t PN532::tgInitAsTarget(const uint8_t *command, const uint8_t len, const uint16_t timeout) {

    int8_t status = HAL(writeCommand)(command, len);
    if (status < 0) {
        return -1;
    }

    status = HAL(readResponse)(command[0], pn532_packetbuffer, sizeof(pn532_packetbuffer), timeout);
    if (status > 0) {
        return 1;
    } else if (PN532_TIMEOUT == status) {
        return 0;
    } else {
        return -2;
    }
}

/**
 * Peer to Peer
 */
int8_t PN532::tgInitAsTarget(uint16_t timeout) {
    const uint8_t command[] = {
            PN532_COMMAND_TGINITASTARGET,
            0,
            0x00, 0x00,         //SENS_RES
            0x00, 0x00, 0x00,   //NFCID1
            0x40,               //SEL_RES

            0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, // POL_RES
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0xFF,

            0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, 0x00, 0x00, //NFCID3t: Change this to desired value

            0x0a, 0x46, 0x66, 0x6D, 0x01, 0x01, 0x10, 0x02, 0x02, 0x00,
            0x80, // LLCP magic number, version parameter and MIUX
            0x00
    };
    return tgInitAsTarget(command, sizeof(command), timeout);
}

int16_t PN532::tgGetData(uint8_t *buf, uint8_t len) {
    buf[0] = PN532_COMMAND_TGGETDATA;

    if (HAL(writeCommand)(buf, 1)) {
        return -1;
    }

    int16_t status = HAL(readResponse)(PN532_COMMAND_TGGETDATA, buf, len, 3000);
    if (0 >= status) {
        return status;
    }

    uint16_t length = status - 1;


    if (buf[0] != 0) {
        DMSG("status is not ok\n");
        return -5;
    }

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = buf[i + 1];
    }

    return length;
}

bool PN532::tgSetData(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen) {
    if (hlen > (sizeof(pn532_packetbuffer) - 1)) {
        if ((body != 0) || (header == pn532_packetbuffer)) {
            DMSG("tgSetData:buffer too small\n");
            return false;
        }

        pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;
        if (HAL(writeCommand)(pn532_packetbuffer, 1, header, hlen)) {
            return false;
        }
    } else {
        for (int8_t i = hlen - 1; i >= 0; i--) {
            pn532_packetbuffer[i + 1] = header[i];
        }
        pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;

        if (HAL(writeCommand)(pn532_packetbuffer, hlen + 1, body, blen)) {
            return false;
        }
    }

    if (0 > HAL(readResponse)(PN532_COMMAND_TGSETDATA, pn532_packetbuffer, sizeof(pn532_packetbuffer), 3000)) {
        return false;
    }

    if (0 != pn532_packetbuffer[0]) {
        return false;
    }

    return true;
}

int16_t PN532::inSelectCard(const uint8_t relevantTarget) {

    pn532_packetbuffer[0] = PN532_COMMAND_INSELECT;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(PN532_COMMAND_INSELECT, pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

int16_t PN532::inDeselectCard(const uint8_t relevantTarget) {

    pn532_packetbuffer[0] = PN532_COMMAND_INDESELECT;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(PN532_COMMAND_INDESELECT, pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

int16_t PN532::inRelease(const uint8_t relevantTarget) {

    pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(PN532_COMMAND_INRELEASE, pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

#if FELICA
/***** FeliCa Functions ******/
/**************************************************************************/
/*!
    @brief  Poll FeliCa card. PN532 acting as reader/initiator,
            peer acting as card/responder.
    @param[in]  systemCode             Designation of System Code. When sending FFFFh as System Code,
                                       all FeliCa cards can return response.
    @param[in]  requestCode            Designation of Request Data as follows:
                                         00h: No Request
                                         01h: System Code request (to acquire System Code of the card)
                                         02h: Communication perfomance request
    @param[out] idm                    IDm of the card (8 bytes)
    @param[out] pmm                    PMm of the card (8 bytes)
    @param[out] systemCodeResponse     System Code of the card (Optional, 2bytes)
    @return                            = 1: A FeliCa card has detected
                                       = 0: No card has detected
                                       < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_Polling(uint16_t systemCode, uint8_t requestCode, uint8_t *idm, uint8_t *pmm,
                             uint16_t *systemCodeResponse, uint16_t timeout) {
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 1;
    pn532_packetbuffer[3] = FELICA_CMD_POLLING;
    pn532_packetbuffer[4] = (systemCode >> 8) & 0xFF;
    pn532_packetbuffer[5] = systemCode & 0xFF;
    pn532_packetbuffer[6] = requestCode;
    pn532_packetbuffer[7] = 0;

    if (HAL(writeCommand)(pn532_packetbuffer, 8)) {
        DMSG("Could not send Polling command\n");
        return -1;
    }

    int16_t status = HAL(readResponse)(PN532_COMMAND_INLISTPASSIVETARGET, pn532_packetbuffer, 22, timeout);
    if (status < 0) {
        DMSG("Could not receive response\n");
        return -2;
    }

    // Check NbTg (pn532_packetbuffer[7])
    if (pn532_packetbuffer[0] == 0) {
        DMSG("No card had detected\n");
        return 0;
    } else if (pn532_packetbuffer[0] != 1) {
        DMSG("Unhandled number of targets inlisted. NbTg: ");
        DMSG_HEX(pn532_packetbuffer[7]);
        DMSG("\n");
        return -3;
    }

    inListedTag = pn532_packetbuffer[1];
    DMSG("Tag number: ");
    DMSG_HEX(pn532_packetbuffer[1]);
    DMSG("\n");

    // length check
    uint8_t responseLength = pn532_packetbuffer[2];
    if (responseLength != 18 && responseLength != 20) {
        DMSG("Wrong response length\n");
        return -4;
    }

    uint8_t i;
    for (i = 0; i < 8; ++i) {
        idm[i] = pn532_packetbuffer[4 + i];
        _felicaIDm[i] = pn532_packetbuffer[4 + i];
        pmm[i] = pn532_packetbuffer[12 + i];
        _felicaPMm[i] = pn532_packetbuffer[12 + i];
    }

    if (responseLength == 20) {
        *systemCodeResponse = (uint16_t)((pn532_packetbuffer[20] << 8) + pn532_packetbuffer[21]);
    }

    return 1;
}

/**************************************************************************/
/*!
    @brief  Sends FeliCa command to the currently inlisted peer

    @param[in]  command         FeliCa command packet. (e.g. 00 FF FF 00 00  for Polling command)
    @param[in]  commandlength   Length of the FeliCa command packet. (e.g. 0x05 for above Polling command )
    @param[out] response        FeliCa response packet. (e.g. 01 NFCID2(8 bytes) PAD(8 bytes)  for Polling response)
    @param[out] responselength  Length of the FeliCa response packet. (e.g. 0x11 for above Polling command )
    @return                          = 1: Success
                                     < 0: error
*/
/**************************************************************************/
int8_t
PN532::felica_SendCommand(const uint8_t *command, uint8_t commandlength, uint8_t *response, uint8_t *responseLength) {
    if (commandlength > 0xFE) {
        DMSG("Command length too long\n");
        return -1;
    }

    pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = inListedTag;
    pn532_packetbuffer[2] = commandlength + 1;

    if (HAL(writeCommand)(pn532_packetbuffer, 3, command, commandlength)) {
        DMSG("Could not send FeliCa command\n");
        return -2;
    }

    // Wait card response
    int16_t status = HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer),
                                       200);
    if (status < 0) {
        DMSG("Could not receive response\n");
        return -3;
    }

    // Check status (pn532_packetbuffer[0])
    if ((pn532_packetbuffer[0] & 0x3F) != 0) {
        DMSG("Status code indicates an error: ");
        DMSG_HEX(pn532_packetbuffer[0]);
        DMSG("\n");
        return -4;
    }

    // length check
    *responseLength = pn532_packetbuffer[1] - 1;
    if ((status - 2) != *responseLength) {
        DMSG("Wrong response length\n");
        return -5;
    }

    memcpy(response, &pn532_packetbuffer[2], *responseLength);

    return 1;
}


/**************************************************************************/
/*!
    @brief  Sends FeliCa Request Service command

    @param[in]  numNode           length of the nodeCodeList
    @param[in]  nodeCodeList      Node codes(Big Endian)
    @param[out] keyVersions       Key Version of each Node (Big Endian)
    @return                          = 1: Success
                                     < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_RequestService(uint8_t numNode, uint16_t *nodeCodeList, uint16_t *keyVersions) {
    if (numNode > FELICA_REQ_SERVICE_MAX_NODE_NUM) {
        DMSG("numNode is too large\n");
        return -1;
    }

    uint8_t i, j = 0;
    uint8_t cmdLen = 1 + 8 + 1 + 2 * numNode;
    uint8_t cmd[cmdLen];
    cmd[j++] = FELICA_CMD_REQUEST_SERVICE;
    for (i = 0; i < 8; ++i) {
        cmd[j++] = _felicaIDm[i];
    }
    cmd[j++] = numNode;
    for (i = 0; i < numNode; ++i) {
        cmd[j++] = nodeCodeList[i] & 0xFF;
        cmd[j++] = (nodeCodeList[i] >> 8) & 0xff;
    }

    uint8_t response[10 + 2 * numNode];
    uint8_t responseLength;

    if (felica_SendCommand(cmd, cmdLen, response, &responseLength) != 1) {
        DMSG("Request Service command failed\n");
        return -2;
    }

    // length check
    if (responseLength != 10 + 2 * numNode) {
        DMSG("Request Service command failed (wrong response length)\n");
        return -3;
    }

    for (i = 0; i < numNode; i++) {
        keyVersions[i] = (uint16_t)(response[10 + i * 2] + (response[10 + i * 2 + 1] << 8));
    }
    return 1;
}


/**************************************************************************/
/*!
    @brief  Sends FeliCa Request Service command

    @param[out]  mode         Current Mode of the card
    @return                   = 1: Success
                              < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_RequestResponse(uint8_t *mode) {
    uint8_t cmd[9];
    cmd[0] = FELICA_CMD_REQUEST_RESPONSE;
    memcpy(&cmd[1], _felicaIDm, 8);

    uint8_t response[10];
    uint8_t responseLength;
    if (felica_SendCommand(cmd, 9, response, &responseLength) != 1) {
        DMSG("Request Response command failed\n");
        return -1;
    }

    // length check
    if (responseLength != 10) {
        DMSG("Request Response command failed (wrong response length)\n");
        return -2;
    }

    *mode = response[9];
    return 1;
}

/**************************************************************************/
/*!
    @brief  Sends FeliCa Read Without Encryption command

    @param[in]  numService         Length of the serviceCodeList
    @param[in]  serviceCodeList    Service Code List (Big Endian)
    @param[in]  numBlock           Length of the blockList
    @param[in]  blockList          Block List (Big Endian, This API only accepts 2-byte block list element)
    @param[out] blockData          Block Data
    @return                        = 1: Success
                                   < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_ReadWithoutEncryption(uint8_t numService, const uint16_t *serviceCodeList, uint8_t numBlock,
                                           const uint16_t *blockList, uint8_t blockData[][16]) {
    if (numService > FELICA_READ_MAX_SERVICE_NUM) {
        DMSG("numService is too large\n");
        return -1;
    }
    if (numBlock > FELICA_READ_MAX_BLOCK_NUM) {
        DMSG("numBlock is too large\n");
        return -2;
    }

    uint8_t i, j = 0, k;
    uint8_t cmdLen = 1 + 8 + 1 + 2 * numService + 1 + 2 * numBlock;
    uint8_t cmd[cmdLen];
    cmd[j++] = FELICA_CMD_READ_WITHOUT_ENCRYPTION;
    for (i = 0; i < 8; ++i) {
        cmd[j++] = _felicaIDm[i];
    }
    cmd[j++] = numService;
    for (i = 0; i < numService; ++i) {
        cmd[j++] = serviceCodeList[i] & 0xFF;
        cmd[j++] = (serviceCodeList[i] >> 8) & 0xff;
    }
    cmd[j++] = numBlock;
    for (i = 0; i < numBlock; ++i) {
        cmd[j++] = (blockList[i] >> 8) & 0xFF;
        cmd[j++] = blockList[i] & 0xff;
    }

    uint8_t response[12 + 16 * numBlock];
    uint8_t responseLength;
    if (felica_SendCommand(cmd, cmdLen, response, &responseLength) != 1) {
        DMSG("Read Without Encryption command failed\n");
        return -3;
    }

    // length check
    if (responseLength != 12 + 16 * numBlock) {
        DMSG("Read Without Encryption command failed (wrong response length)\n");
        return -4;
    }

    // status flag check
    if (response[9] != 0 || response[10] != 0) {
        DMSG("Read Without Encryption command failed (Status Flag: ");
        DMSG_HEX(pn532_packetbuffer[9]);
        DMSG_HEX(pn532_packetbuffer[10]);
        DMSG(")\n");
        return -5;
    }

    k = 12;
    for (i = 0; i < numBlock; i++) {
        for (j = 0; j < 16; j++) {
            blockData[i][j] = response[k++];
        }
    }

    return 1;
}


/**************************************************************************/
/*!
    @brief  Sends FeliCa Write Without Encryption command

    @param[in]  numService         Length of the serviceCodeList
    @param[in]  serviceCodeList    Service Code List (Big Endian)
    @param[in]  numBlock           Length of the blockList
    @param[in]  blockList          Block List (Big Endian, This API only accepts 2-byte block list element)
    @param[in]  blockData          Block Data (each Block has 16 bytes)
    @return                        = 1: Success
                                   < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_WriteWithoutEncryption(uint8_t numService, const uint16_t *serviceCodeList, uint8_t numBlock,
                                            const uint16_t *blockList, uint8_t blockData[][16]) {
    if (numService > FELICA_WRITE_MAX_SERVICE_NUM) {
        DMSG("numService is too large\n");
        return -1;
    }
    if (numBlock > FELICA_WRITE_MAX_BLOCK_NUM) {
        DMSG("numBlock is too large\n");
        return -2;
    }

    uint8_t i, j = 0, k;
    uint8_t cmdLen = 1 + 8 + 1 + 2 * numService + 1 + 2 * numBlock + 16 * numBlock;
    uint8_t cmd[cmdLen];
    cmd[j++] = FELICA_CMD_WRITE_WITHOUT_ENCRYPTION;
    for (i = 0; i < 8; ++i) {
        cmd[j++] = _felicaIDm[i];
    }
    cmd[j++] = numService;
    for (i = 0; i < numService; ++i) {
        cmd[j++] = serviceCodeList[i] & 0xFF;
        cmd[j++] = (serviceCodeList[i] >> 8) & 0xff;
    }
    cmd[j++] = numBlock;
    for (i = 0; i < numBlock; ++i) {
        cmd[j++] = (blockList[i] >> 8) & 0xFF;
        cmd[j++] = blockList[i] & 0xff;
    }
    for (i = 0; i < numBlock; ++i) {
        for (k = 0; k < 16; k++) {
            cmd[j++] = blockData[i][k];
        }
    }

    uint8_t response[11];
    uint8_t responseLength;
    if (felica_SendCommand(cmd, cmdLen, response, &responseLength) != 1) {
        DMSG("Write Without Encryption command failed\n");
        return -3;
    }

    // length check
    if (responseLength != 11) {
        DMSG("Write Without Encryption command failed (wrong response length)\n");
        return -4;
    }

    // status flag check
    if (response[9] != 0 || response[10] != 0) {
        DMSG("Write Without Encryption command failed (Status Flag: ");
        DMSG_HEX(pn532_packetbuffer[9]);
        DMSG_HEX(pn532_packetbuffer[10]);
        DMSG(")\n");
        return -5;
    }

    return 1;
}

/**************************************************************************/
/*!
    @brief  Sends FeliCa Request System Code command

    @param[out] numSystemCode        Length of the systemCodeList
    @param[out] systemCodeList       System Code list (Array length should longer than 16)
    @return                          = 1: Success
                                     < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_RequestSystemCode(uint8_t *numSystemCode, uint16_t *systemCodeList) {
    uint8_t cmd[9];
    cmd[0] = FELICA_CMD_REQUEST_SYSTEM_CODE;
    memcpy(&cmd[1], _felicaIDm, 8);

    uint8_t response[10 + 2 * 16];
    uint8_t responseLength;
    if (felica_SendCommand(cmd, 9, response, &responseLength) != 1) {
        DMSG("Request System Code command failed\n");
        return -1;
    }
    *numSystemCode = response[9];

    // length check
    if (responseLength < 10 + 2 * *numSystemCode) {
        DMSG("Request System Code command failed (wrong response length)\n");
        return -2;
    }

    uint8_t i;
    for (i = 0; i < *numSystemCode; i++) {
        systemCodeList[i] = (uint16_t)((response[10 + i * 2] << 8) + response[10 + i * 2 + 1]);
    }

    return 1;
}


/**************************************************************************/
/*!
    @brief  Release FeliCa card
    @return                          = 1: Success
                                     < 0: error
*/
/**************************************************************************/
int8_t PN532::felica_Release() {
    // InRelease
    pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
    pn532_packetbuffer[1] = 0x00;   // All target
    DMSG("Release all FeliCa target\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        DMSG("No ACK\n");
        return -1;  // no ACK
    }

    // Wait card response
    int16_t frameLength = HAL(readResponse)(PN532_COMMAND_INRELEASE, pn532_packetbuffer, sizeof(pn532_packetbuffer),
                                            1000);
    if (frameLength < 0) {
        DMSG("Could not receive response\n");
        return -2;
    }

    // Check status (pn532_packetbuffer[0])
    if ((pn532_packetbuffer[0] & 0x3F) != 0) {
        DMSG("Status code indicates an error: ");
        DMSG_HEX(pn532_packetbuffer[7]);
        DMSG("\n");
        return -3;
    }

    return 1;
}
#endif

// DesFire functions below

/**************************************************************************
    Enables random ID mode in which the card sends another UID each time.
    In Random UID mode the card sends a 4 byte UID that always starts with 0x80.
    To get the real UID of the card call GetRealCardID()
    ---------------------------------------------------------------------
    ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
    ---------------------------------------------------------------------
    NXP does not provide any way to turn off Random ID mode.
    If you once call this funtion the card will send random ID FOREVER!
    ---------------------------------------------------------------------
    ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION ATTENTION
    ---------------------------------------------------------------------
**************************************************************************/
bool PN532::EnableRandomIDForever() {
    //if (mu8_DebugLevel > 0) Utils::Print("\r\n*** EnableRandomIDForever()\r\n");

    TX_BUFFER(i_Command, 2);
    i_Command.AppendUint8(DFEV1_INS_SET_CONFIGURATION);
    i_Command.AppendUint8(0x00); // subcommand 00

    TX_BUFFER(i_Params, 16);
    i_Params.AppendUint8(0x02); // 0x02 = enable random ID, 0x01 = disable format

    // The TX CMAC must not be calculated here because a CBC encryption operation has already been executed
    return (0 == DataExchange(&i_Command, &i_Params, NULL, 0, NULL, MAC_TcryptRmac));
}

/**************************************************************************
    This command makes only sense if the card is in Random UID mode.
    It allows to obtain the real UID of the card.
    If Random ID mode is not active use ReadPassiveTargetID() or GetCardVersion()
    instead to get the UID.
    A previous authentication is required.
**************************************************************************/
bool PN532::GetRealCardID(byte u8_UID[7]) {
    //if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetRealCardID()\r\n");

    if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED) {
        //Utils::Print("Not authenticated\r\n");
        return false;
    }

    RX_BUFFER(i_Data, 16);
    if (16 != DataExchange(DFEV1_INS_GET_CARD_UID, NULL, i_Data, 16, NULL, MAC_TmacRcrypt))
        return false;

    // The card returns UID[7] + CRC32[4] encrypted with the session key
    // Copy the 7 bytes of the UID to the output buffer
    i_Data.ReadBuf(u8_UID, 7);

    // Get the CRC sent by the card
    uint32_t u32_Crc1 = i_Data.ReadUint32();

    // The CRC must be calculated over the UID + the status byte appended
    byte u8_Status = ST_Success;
    uint32_t u32_Crc2 = Utils::CalcCrc32(u8_UID, 7, &u8_Status, 1);

#ifdef DEBUG
    //Utils::Print("* CRC:       0x");
    //Utils::PrintHex32(u32_Crc2, LF);
#endif

    if (u32_Crc1 != u32_Crc2) {
        //Utils::Print("Invalid CRC\r\n");
        return false;
    }

#ifdef DEBUG
    //Utils::Print("Real UID: ");
    //Utils::PrintHexBuf(u8_UID, 7, LF);
#endif
    return true;
}


// ########################################################################
// ####                      LOW LEVEL FUNCTIONS                      #####
// ########################################################################

// If this value is != 0, the PN532 has returned an error code while executing the latest command.
// Typically a Timeout error (Value = 0x01) means that the card is too far away from the reader.
// Interestingly a timeout occurres typically when authenticating.
// The commands that are executed first (GetKeyVersion and SelectApplication) execute without problems.
// But it when it comes to Authenticate() the card suddenly does not respond anymore -> Timeout from PN532.
// Conclusion: It seems that a Desfire card increases its power consumption in the moment when encrypting data,
// so when it is too far away from the antenna -> the connection dies.
byte PN532::GetLastPN532Error() {
    return mu8_LastPN532Error;
}

/**************************************************************************
    Sends data to the card and receives the response.
    u8_Command    = Desfire command without additional paramaters
    pi_Command    = Desfire command + possible additional paramaters that will not be encrypted
    pi_Params     = Desfire command parameters that may be encrypted (MAC_Tcrypt). This paramater may also be null.
    u8_RecvBuf    = buffer that receives the received data (should be the size of the expected recv data)
   s32_RecvSize   = buffer size of u8_RecvBuf
    pe_Status     = if (!= NULL) -> receives the status byte
    e_Mac         = defines CMAC calculation
    returns the byte count that has been read into u8_RecvBuf or -1 on error
**************************************************************************/
int
PN532::DataExchange(byte u8_Command, TxBuffer *pi_Params, byte *u8_RecvBuf, int s32_RecvSize, DESFireStatus *pe_Status,
                    DESFireCmac e_Mac) {
    TX_BUFFER(i_Command, 1);
    i_Command.AppendUint8(u8_Command);

    return DataExchange(&i_Command, pi_Params, u8_RecvBuf, s32_RecvSize, pe_Status, e_Mac);
}

int PN532::DataExchangeOLD(TxBuffer *pi_Command,               // in (command + params that are not encrypted)
                           TxBuffer *pi_Params,                // in (parameters that may be encrypted)
                           byte *u8_RecvBuf, int s32_RecvSize, // out
                           DESFireStatus *pe_Status,           // out
                           DESFireCmac e_Mac)               // in
{
    if (pe_Status) *pe_Status = ST_Success;
    mu8_LastPN532Error = 0;
    TX_BUFFER(i_Empty, 1);
    if (pi_Params == NULL)
        pi_Params = &i_Empty;
    // The response for INDATAEXCHANGE is always:
    // - 0xD5
    // - 0x41
    // - Status byte from PN532        (0 if no error)
    // - Status byte from Desfire card (0 if no error)
    // - data bytes ...
    int s32_Overhead = 11; // Overhead added to payload = 11 bytes = 7 bytes for PN532 frame + 3 bytes for INDATAEXCHANGE response + 1 card status byte
    if (e_Mac & MAC_Rmac) s32_Overhead += 8; // + 8 bytes for CMAC
    // pn532_packetbuffer is used for input and output
    if (2 + pi_Command->GetCount() + pi_Params->GetCount() > PN532_PACKBUFFSIZE ||
        s32_Overhead + s32_RecvSize > PN532_PACKBUFFSIZE) {
        Utils::Print("DataExchange(): Invalid parameters\r\n");
        return -1;
    }
    if (e_Mac & (MAC_Tcrypt | MAC_Rcrypt)) {
        if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED) {
            Utils::Print("Not authenticated\r\n");
            return -1;
        }
    }
    if (e_Mac & MAC_Tcrypt) // CRC and encrypt pi_Params
    {
        if (mu8_DebugLevel > 0) {
            Utils::Print("* Sess Key IV: ");
            mpi_SessionKey->PrintIV(LF);
        }
        // The CRC is calculated over the command (which is not encrypted) and the parameters to be encrypted.
        uint32_t u32_Crc = Utils::CalcCrc32(pi_Command->GetData(), pi_Command->GetCount(), pi_Params->GetData(),
                                            pi_Params->GetCount());
        if (!pi_Params->AppendUint32(u32_Crc))
            return -1; // buffer overflow
        int s32_CryptCount = mpi_SessionKey->CalcPaddedBlockSize(pi_Params->GetCount());
        if (!pi_Params->SetCount(s32_CryptCount))
            return -1; // buffer overflow
        if (mu8_DebugLevel > 0) {
            Utils::Print("* CRC Params:  0x");
            Utils::PrintHex32(u32_Crc, LF);
            Utils::Print("* Params:      ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }
        if (!mpi_SessionKey->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, pi_Params->GetData(), pi_Params->GetData(),
                                          s32_CryptCount))
            return -1;
        if (mu8_DebugLevel > 0) {
            Utils::Print("* Params_enc:  ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }
    }
    byte u8_Command = pi_Command->GetData()[0];
    byte u8_CalcMac[16];
    if ((e_Mac & MAC_Tmac) &&                       // Calculate the TX CMAC only if the caller requests it
        (u8_Command != DF_INS_ADDITIONAL_FRAME) &&
        // In case of DF_INS_ADDITIONAL_FRAME there are never parameters passed -> nothing to do here
        (mu8_LastAuthKeyNo != NOT_AUTHENTICATED))   // No session key -> no CMAC calculation possible
    {
        mi_CmacBuffer.Clear();
        if (!mi_CmacBuffer.AppendBuf(pi_Command->GetData(), pi_Command->GetCount()) ||
            !mi_CmacBuffer.AppendBuf(pi_Params->GetData(), pi_Params->GetCount()))
            return -1;
        // The CMAC must be calculated here although it is not transmitted, because it maintains the IV up to date.
        // The initialization vector must always be correct otherwise the card will give an integrity error the next time the session key is used.
        if (!mpi_SessionKey->CalculateCmac(mi_CmacBuffer, u8_CalcMac))
            return -1;
        /*

1A 2A 0F 0D 49 D2 FF 14 48 E3 F2 5F F3 63 1F 89 (TX CMAC)
Sending:  00 00 FF 0B F5 <D4 40 01 BD 01 00 00 00 0A 00 00> 23 00
Response: 00 00 FF 14 EC <D5 41 00 00 F1 DA 99 E6 5F AF 90 6E B8 39 17 02 D7 43 FF 82> EF 00 E4 E0
f1 da 99 e6 5f af 90 6e b8 39 17  2 d7 43 ff 82 (pn532_packetbuffer)
F1 DA 99 E6 5F AF 90 6E 00 (mi_CmacBuffer 2)
1D 72 F8 47 43 27 D0 0A A2 C4 A1 CB AB 44 6E 04 (RX CMAC)
B8 39 17 02 D7 43 FF 82 E0 B1 E4 E0 B1 3C 39 2E (u8_RxMac)

         */


        if (mu8_DebugLevel > 1) {
            Utils::Print("TX CMAC:  ");
            Utils::PrintHexBuf(u8_CalcMac, mpi_SessionKey->GetBlockSize(), LF);
        }
    }
    int P = 0;
    pn532_packetbuffer[P++] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[P++] = 1; // Card number (Logical target number)
    memcpy(pn532_packetbuffer + P, pi_Command->GetData(), pi_Command->GetCount());
    P += pi_Command->GetCount();
    memcpy(pn532_packetbuffer + P, pi_Params->GetData(), pi_Params->GetCount());
    P += pi_Params->GetCount();
    if (!SendCommandCheckAck(pn532_packetbuffer, P))
        return -1;
    byte s32_Len = ReadData(pn532_packetbuffer, s32_RecvSize + s32_Overhead);
    // ReadData() returns 3 byte if status error from the PN532
    // ReadData() returns 4 byte if status error from the Desfire card
    if (s32_Len < 3 || pn532_packetbuffer[1] != PN532_COMMAND_INDATAEXCHANGE + 1) {
        PRINT_DEBUG("DataExchange() failed")
        return -1;
    }
    // Here we get two status bytes that must be checked
    byte u8_PN532Status = pn532_packetbuffer[2]; // contains errors from the PN532
    byte u8_CardStatus = pn532_packetbuffer[3]; // contains errors from the Desfire card

    Serial.println();
    Serial.println("4B + CmacBuf + RxMac");
    Serial.print(String("pn532_packetbuffer 3: "));
    for (int i = 0; i < s32_Len; i++) {
        Serial.print(String(pn532_packetbuffer[i], HEX) + " ");
    }
    Serial.println();

    mu8_LastPN532Error = u8_PN532Status;
    if (!CheckPN532Status(u8_PN532Status) || s32_Len < 4)
        return -1;
    // After any error that the card has returned the authentication is invalidated.
    // The card does not send any CMAC anymore until authenticated anew.
    if (u8_CardStatus != ST_Success && u8_CardStatus != ST_MoreFrames) {
        mu8_LastAuthKeyNo = NOT_AUTHENTICATED; // A new authentication is required now
    }
    if (!CheckCardStatus((DESFireStatus) u8_CardStatus))
        return -1;
    if (pe_Status)
        *pe_Status = (DESFireStatus) u8_CardStatus;
    s32_Len -= 4; // 3 bytes for INDATAEXCHANGE response + 1 byte card status
    // A CMAC may be appended to the end of the frame.
    // The CMAC calculation is important because it maintains the IV of the session key up to date.
    // If the IV is out of sync with the IV in the card, the next encryption with the session key will result in an Integrity Error.
    if ((e_Mac & MAC_Rmac) &&// Calculate RX CMAC only if the caller requests it
        (u8_CardStatus == ST_Success || u8_CardStatus == ST_MoreFrames) &&
        // In case of an error there is no CMAC in the response
        (mu8_LastAuthKeyNo !=
         NOT_AUTHENTICATED))                          // No session key -> no CMAC calculation possible
    {
        // For example GetCardVersion() calls DataExchange() 3 times:
        // 1. u8_Command = DF_INS_GET_VERSION      -> clear CMAC buffer + append received data
        // 2. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        // 3. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        if (u8_Command != DF_INS_ADDITIONAL_FRAME) {
            mi_CmacBuffer.Clear();
        }
        // This is an intermediate frame. More frames will follow. There is no CMAC in the response yet.
        if (u8_CardStatus == ST_MoreFrames) {
            Serial.println("ST_MoreFrames");
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, s32_Len))
                return -1;
        }
        if ((s32_Len >= 8) &&             // If the response is shorter than 8 bytes it surely does not contain a CMAC
            (u8_CardStatus == ST_Success)) // Response contains CMAC only in case of success
        {
            s32_Len -= 8; // Do not return the received CMAC to the caller and do not include it into the CMAC calculation
            byte *u8_RxMac = pn532_packetbuffer + 4 + s32_Len;

            // The CMAC is calculated over the RX data + the status byte appended to the END of the RX data!
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, s32_Len) ||
                !mi_CmacBuffer.AppendUint8(u8_CardStatus)) {
                return -1;
            }

            Serial.print("mi_CmacBuffer 2: ");
            Utils::PrintHexBuf(mi_CmacBuffer.GetData(), mi_CmacBuffer.GetCount(), LF);

            if (!mpi_SessionKey->CalculateCmac(mi_CmacBuffer, u8_CalcMac)) {
                return -1;
            }

            if (mu8_DebugLevel > 1) {
                Utils::Print("RX CMAC:  ");
                Utils::PrintHexBuf(u8_CalcMac, mpi_SessionKey->GetBlockSize(), LF);
            }

            // For AES the CMAC is 16 byte, but only 8 are transmitted
            if (memcmp(u8_RxMac, u8_CalcMac, 8) != 0) {
                PRINT_DEBUG("CMAC Mismatch\r\n");
                Serial.print("u8_RxMac: ");
                Utils::PrintHexBuf(u8_RxMac, mpi_SessionKey->GetBlockSize(), LF);
                return -1;
            }
        }
    }

    if (s32_Len > s32_RecvSize) {
        Utils::Print("DataExchange() Buffer overflow\r\n");
        return -1;
    }

    if (u8_RecvBuf && s32_Len) {
        memcpy(u8_RecvBuf, pn532_packetbuffer + 4, s32_Len);
        if (e_Mac & MAC_Rcrypt) // decrypt received data with session key
        {
            if (!mpi_SessionKey->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RecvBuf, u8_RecvBuf, s32_Len))
                return -1;
            if (mu8_DebugLevel > 1) {
                Utils::Print("Decrypt:  ");
                Utils::PrintHexBuf(u8_RecvBuf, s32_Len, LF);
            }
        }
    }
    return s32_Len;
}

int PN532::DataExchangeReadFile(byte u8_Command, TxBuffer *pi_Params, byte *u8_RecvBuf, int s32_RecvSize,
                                DESFireStatus *pe_Status, DESFireCmac e_Mac) {
    TX_BUFFER(i_Command, 1);
    i_Command.AppendUint8(u8_Command);

    return DataExchangeReadFile(&i_Command, pi_Params, u8_RecvBuf, s32_RecvSize, pe_Status, e_Mac);
}

int PN532::DataExchangeReadFile(TxBuffer *pi_Command,               // in (command + params that are not encrypted)
                                TxBuffer *pi_Params,                // in (parameters that may be encrypted)
                                byte *u8_RecvBuf, int s32_RecvSize, // out
                                DESFireStatus *pe_Status,
                                DESFireCmac e_Mac)               // in
{
    if (pe_Status) *pe_Status = ST_Success;
    mu8_LastPN532Error = 0;

    TX_BUFFER(i_Empty, 1);
    if (pi_Params == NULL)
        pi_Params = &i_Empty;

    int s32_Overhead = 11; // Overhead added to payload = 11 bytes = 7 bytes for PN532 frame + 3 bytes for INDATAEXCHANGE response + 1 card status byte
    if (e_Mac & MAC_Rmac) s32_Overhead += 8; // + 8 bytes for CMAC

    if (2 + pi_Command->GetCount() + pi_Params->GetCount() > PN532_PACKBUFFSIZE ||
        s32_Overhead + s32_RecvSize > PN532_PACKBUFFSIZE) {
        Utils::Print("DataExchange(): Invalid parameters\r\n");
        return -1;
    }

    if (e_Mac & (MAC_Tcrypt | MAC_Rcrypt)) {
        if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED) {
            Utils::Print("Not authenticated\r\n");
            return -1;
        }
    }

    if (e_Mac & MAC_Tcrypt) // CRC and encrypt pi_Params
    {
        // The CRC is calculated over the command (which is not encrypted) and the parameters to be encrypted.
        uint32_t u32_Crc = Utils::CalcCrc32(pi_Command->GetData(), pi_Command->GetCount(), pi_Params->GetData(),
                                            pi_Params->GetCount());
        if (!pi_Params->AppendUint32(u32_Crc))
            return -1; // buffer overflow

        int s32_CryptCount = mpi_SessionKey->CalcPaddedBlockSize(pi_Params->GetCount());
        if (!pi_Params->SetCount(s32_CryptCount))
            return -1; // buffer overflow

        if (mu8_DebugLevel > 0) {
            Utils::Print("* CRC Params:  0x");
            Utils::PrintHex32(u32_Crc, LF);
            Utils::Print("* Params:      ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }

        if (!mpi_SessionKey->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, pi_Params->GetData(), pi_Params->GetData(),
                                          s32_CryptCount))
            return -1;

        if (mu8_DebugLevel > 0) {
            Utils::Print("* Params_enc:  ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }
    }

    byte u8_Command = pi_Command->GetData()[0];

    byte u8_CalcMac[16];
    if ((e_Mac & MAC_Tmac) &&                       // Calculate the TX CMAC only if the caller requests it
        (u8_Command != DF_INS_ADDITIONAL_FRAME) &&
        // In case of DF_INS_ADDITIONAL_FRAME there are never parameters passed -> nothing to do here
        (mu8_LastAuthKeyNo != NOT_AUTHENTICATED))   // No session key -> no CMAC calculation possible
    {
        mi_CmacBuffer.Clear();
        if (!mi_CmacBuffer.AppendBuf(pi_Command->GetData(), pi_Command->GetCount()) ||
            !mi_CmacBuffer.AppendBuf(pi_Params->GetData(), pi_Params->GetCount()))
            return -1;

        // The CMAC must be calculated here although it is not transmitted, because it maintains the IV up to date.
        // The initialization vector must always be correct otherwise the card will give an integrity error the next time the session key is used.
        if (!mpi_SessionKey->CalculateCmac(mi_CmacBuffer, u8_CalcMac))
            return -1;

        if (mu8_DebugLevel > 1) {
            Utils::Print("TX CMAC:  ");
            Utils::PrintHexBuf(u8_CalcMac, mpi_SessionKey->GetBlockSize(), LF);
        }
    }

    int P = 0;
    pn532_packetbuffer[P++] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[P++] = 1; // Card number (Logical target number)

    memcpy(pn532_packetbuffer + P, pi_Command->GetData(), pi_Command->GetCount());

    P += pi_Command->GetCount();

    memcpy(pn532_packetbuffer + P, pi_Params->GetData(), pi_Params->GetCount());

    P += pi_Params->GetCount();

    TX_BUFFER(commandbuf, 16);
    commandbuf.AppendBuf(pn532_packetbuffer+2, 8);

    if (!mpi_SessionKey->CalculateCmac(commandbuf, u8_CalcMac))
        return -1;

    if (!SendCommandCheckAck(pn532_packetbuffer, P)) {
        PRINT_DEBUG("SendCommandCheckAck failed");
        return -1;
    }

    byte status = ReadData(pn532_packetbuffer, s32_RecvSize + s32_Overhead);

    if (status < 3 || pn532_packetbuffer[1] != PN532_COMMAND_INDATAEXCHANGE + 1) {
        Serial.println(String("status: ") + String(status));
        PRINT_DEBUG("DataExchange() failed")
        return -1;
    }

    uint8_t u8_PN532Status = pn532_packetbuffer[2]; // contains errors from the PN532
    uint8_t u8_CardStatus = pn532_packetbuffer[3]; // contains errors from the Desfire card

    mu8_LastPN532Error = u8_PN532Status;

    if (u8_PN532Status != ST_Success || status < 4) {
        Serial.println(String("reading file failed: u8_PN532Status != ST_Success || status < 4"));
        Serial.println(String("status: ") + String(status));
        Serial.println(String("u8_PN532Status: ") + String(u8_PN532Status));
        return -1;
    }

    if (u8_CardStatus != ST_Success && u8_CardStatus != ST_MoreFrames) {
        mu8_LastAuthKeyNo = NOT_AUTHENTICATED; // A new authentication is required now
    }

    if (!CheckCardStatus((DESFireStatus) u8_CardStatus)) {
        Serial.println(String("(DESFireStatus)u8_CardStatus: ") + String((DESFireStatus) u8_CardStatus, HEX));
        return -1;
    }

    if (pe_Status)
        *pe_Status = (DESFireStatus) u8_CardStatus;

    status -= 4; // 3 bytes for INDATAEXCHANGE response + 1 byte card status

    if ((e_Mac & MAC_Rmac) &&// Calculate RX CMAC only if the caller requests it
        (u8_CardStatus == ST_Success || u8_CardStatus == ST_MoreFrames) &&
        // In case of an error there is no CMAC in the response
        (mu8_LastAuthKeyNo !=
         NOT_AUTHENTICATED))                          // No session key -> no CMAC calculation possible
    {
        // For example GetCardVersion() calls DataExchange() 3 times:
        // 1. u8_Command = DF_INS_GET_VERSION      -> clear CMAC buffer + append received data
        // 2. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        // 3. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        if (u8_Command != DF_INS_ADDITIONAL_FRAME) {
            mi_CmacBuffer.Clear();
        }

        // This is an intermediate frame. More frames will follow. There is no CMAC in the response yet.
        if (u8_CardStatus == ST_MoreFrames) {
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, status))
                return -1;
        }

        if ((status >= 8) &&             // If the response is shorter than 8 bytes it surely does not contain a CMAC
            (u8_CardStatus == ST_Success)) // Response contains CMAC only in case of success
        {
            status -= 8; // Do not return the received CMAC to the caller and do not include it into the CMAC calculation
            byte *u8_RxMac = pn532_packetbuffer + 4 + status;

            // The CMAC is calculated over the RX data + the status byte appended to the END of the RX data!
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, status) ||
                !mi_CmacBuffer.AppendUint8(u8_CardStatus))
                return -1;
        }
    }

    if (status > s32_RecvSize) {
        Utils::Print("DataExchange() Buffer overflow\r\n");
        return -1;
    }

    if (u8_RecvBuf && status) {
        if (!mpi_SessionKey->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RecvBuf, pn532_packetbuffer + 4, 16)) {
            PRINT_DEBUG("Decrypting failed")
            return -1;
        }
    }
    return status;
}

int PN532::DataExchange(TxBuffer *pi_Command,               // in (command + params that are not encrypted)
                        TxBuffer *pi_Params,                // in (parameters that may be encrypted)
                        byte *u8_RecvBuf, int s32_RecvSize, // out
                        DESFireStatus *pe_Status,
                        DESFireCmac e_Mac)               // in
{
    if (pe_Status) *pe_Status = ST_Success;
    mu8_LastPN532Error = 0;

    TX_BUFFER(i_Empty, 1);
    if (pi_Params == NULL)
        pi_Params = &i_Empty;

    // The response for INDATAEXCHANGE is always:
    // - 0xD5
    // - 0x41
    // - Status byte from PN532        (0 if no error)
    // - Status byte from Desfire card (0 if no error)
    // - data bytes ...
    int s32_Overhead = 11; // Overhead added to payload = 11 bytes = 7 bytes for PN532 frame + 3 bytes for INDATAEXCHANGE response + 1 card status byte
    if (e_Mac & MAC_Rmac) s32_Overhead += 8; // + 8 bytes for CMAC

    // pn532_packetbuffer is used for input and output
    if (2 + pi_Command->GetCount() + pi_Params->GetCount() > PN532_PACKBUFFSIZE ||
        s32_Overhead + s32_RecvSize > PN532_PACKBUFFSIZE) {
        Utils::Print("DataExchange(): Invalid parameters\r\n");
        return -1;
    }

    if (e_Mac & (MAC_Tcrypt | MAC_Rcrypt)) {
        if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED) {
            Utils::Print("Not authenticated\r\n");
            return -1;
        }
    }

    if (e_Mac & MAC_Tcrypt) // CRC and encrypt pi_Params
    {
        if (mu8_DebugLevel > 0) {
            Utils::Print("* Sess Key IV: ");
            mpi_SessionKey->PrintIV(LF);
        }

        // The CRC is calculated over the command (which is not encrypted) and the parameters to be encrypted.
        uint32_t u32_Crc = Utils::CalcCrc32(pi_Command->GetData(), pi_Command->GetCount(), pi_Params->GetData(),
                                            pi_Params->GetCount());
        if (!pi_Params->AppendUint32(u32_Crc))
            return -1; // buffer overflow

        int s32_CryptCount = mpi_SessionKey->CalcPaddedBlockSize(pi_Params->GetCount());
        if (!pi_Params->SetCount(s32_CryptCount))
            return -1; // buffer overflow

        if (mu8_DebugLevel > 0) {
            Utils::Print("* CRC Params:  0x");
            Utils::PrintHex32(u32_Crc, LF);
            Utils::Print("* Params:      ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }

        if (!mpi_SessionKey->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, pi_Params->GetData(), pi_Params->GetData(),
                                          s32_CryptCount))
            return -1;

        if (mu8_DebugLevel > 0) {
            Utils::Print("* Params_enc:  ");
            Utils::PrintHexBuf(pi_Params->GetData(), s32_CryptCount, LF);
        }
    }

    byte u8_Command = pi_Command->GetData()[0];

    byte u8_CalcMac[16];
    if ((e_Mac & MAC_Tmac) &&                       // Calculate the TX CMAC only if the caller requests it
        (u8_Command != DF_INS_ADDITIONAL_FRAME) &&
        // In case of DF_INS_ADDITIONAL_FRAME there are never parameters passed -> nothing to do here
        (mu8_LastAuthKeyNo != NOT_AUTHENTICATED))   // No session key -> no CMAC calculation possible
    {
        mi_CmacBuffer.Clear();
        if (!mi_CmacBuffer.AppendBuf(pi_Command->GetData(), pi_Command->GetCount()) ||
            !mi_CmacBuffer.AppendBuf(pi_Params->GetData(), pi_Params->GetCount()))
            return -1;

        // The CMAC must be calculated here although it is not transmitted, because it maintains the IV up to date.
        // The initialization vector must always be correct otherwise the card will give an integrity error the next time the session key is used.
        if (!mpi_SessionKey->CalculateCmac(mi_CmacBuffer, u8_CalcMac))
            return -1;

        if (mu8_DebugLevel > 1) {
            Utils::Print("TX CMAC:  ");
            Utils::PrintHexBuf(u8_CalcMac, mpi_SessionKey->GetBlockSize(), LF);
        }
    }

    int P = 0;
    pn532_packetbuffer[P++] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[P++] = 1; // Card number (Logical target number)

    memcpy(pn532_packetbuffer + P, pi_Command->GetData(), pi_Command->GetCount());

    Serial.print(String("pi_Command->GetData(): "));
    for (int i = 0; i < pi_Command->GetCount(); i++) {
        Serial.print(String(pi_Command->GetData()[i], HEX) + " ");
    }
    Serial.println();

    P += pi_Command->GetCount();

    memcpy(pn532_packetbuffer + P, pi_Params->GetData(), pi_Params->GetCount());

    Serial.print(String("pi_Params->GetData(): "));
    for (int i = 0; i < pi_Params->GetCount(); i++) {
        Serial.print(String(pi_Params->GetData()[i], HEX) + " ");
    }
    Serial.println();

    P += pi_Params->GetCount();

    Serial.print(String("pn532_packetbuffer 4: "));
    for (int i = 0; i < P; i++) {
        Serial.print(String(pn532_packetbuffer[i], HEX));
        Serial.print(" ");
    }
    Serial.println();

    // ORIGINALLY:
    if (!SendCommandCheckAck(pn532_packetbuffer, P)) {
        PRINT_DEBUG("SendCommandCheckAck failed");
        return -1;
    }

    // MODIFIED:
    //if (HAL(writeCommand)(pn532_packetbuffer, P)) {
    //    return -1;
    //}

    /*
    Serial.print(String("2 pn532_packetbuffer: "));
    for (int i = 0; i < 16; i++) {
        Serial.print(String(pn532_packetbuffer[i], HEX));
        Serial.print(" ");
    }
    Serial.println();
*/

    // read data packet
    //Modified
    //int16_t status = HAL(readResponse)(PN532_COMMAND_INDATAEXCHANGE, pn532_packetbuffer, sizeof(pn532_packetbuffer));
    //if (0 > status) {
    //    return 0;
    //}

    // Originally
    byte status = ReadData(pn532_packetbuffer, s32_RecvSize + s32_Overhead);

    //Serial.println(status);
    //Serial.println(s32_RecvSize);

    PRINT_DEBUG(String("pn532_packetbuffer 5: "))
    for (int i = 0; i < sizeof(pn532_packetbuffer); i++) {
        if (i % 8 == 0) Serial.println();
        Serial.print(pn532_packetbuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // ReadData() returns 3 byte if status error from the PN532
    // ReadData() returns 4 bytes if status error from the Desfire card
    if (status < 3 || pn532_packetbuffer[1] != PN532_COMMAND_INDATAEXCHANGE + 1) {
        Serial.println(String("status: ") + String(status));
        PRINT_DEBUG(String("pn532_packetbuffer 6: "))
        for (int i = 0; i < 16; i++) Serial.print(String(pn532_packetbuffer[i], HEX) + " ");
        Serial.println();
        PRINT_DEBUG("DataExchange() failed")
        return -1;
    }

    // Here we get two status bytes that must be checked
    uint8_t u8_PN532Status = pn532_packetbuffer[2]; // contains errors from the PN532
    uint8_t u8_CardStatus = pn532_packetbuffer[3]; // contains errors from the Desfire card

    mu8_LastPN532Error = u8_PN532Status;

    if (u8_PN532Status != ST_Success || status < 4) {
        Serial.print(String("pn532_packetbuffer 7: "));
        for (int i = 0; i < 16; i++) Serial.print(String(pn532_packetbuffer[i], HEX) + " ");
        Serial.println();
        Serial.println(String("status: ") + String(status));
        Serial.println(String("u8_PN532Status: ") + String(u8_PN532Status));
        Serial.println(String("u8_CardStatus: ") + String(u8_CardStatus));
        return -1;
    }

    // After any error that the card has returned the authentication is invalidated.
    // The card does not send any CMAC anymore until authenticated anew.
    if (u8_CardStatus != ST_Success && u8_CardStatus != ST_MoreFrames) {
        PRINT_DEBUG(String("pn532_packetbuffer 8: "));
        for (int i = 0; i < 16; i++) Serial.print(String(pn532_packetbuffer[i], HEX) + " ");
        Serial.println();

        mu8_LastAuthKeyNo = NOT_AUTHENTICATED; // A new authentication is required now
    }

    if (!CheckCardStatus((DESFireStatus) u8_CardStatus)) {
        Serial.println(String("(DESFireStatus)u8_CardStatus: ") + String((DESFireStatus) u8_CardStatus, HEX));
        return -1;
    }

    if (pe_Status)
        *pe_Status = (DESFireStatus) u8_CardStatus;

    status -= 4; // 3 bytes for INDATAEXCHANGE response + 1 byte card status

    // A CMAC may be appended to the end of the frame.
    // The CMAC calculation is important because it maintains the IV of the session key up to date.
    // If the IV is out of sync with the IV in the card, the next encryption with the session key will result in an Integrity Error.
    //Serial.println(status);
    //Serial.println(s32_RecvSize);

    if ((e_Mac & MAC_Rmac) &&// Calculate RX CMAC only if the caller requests it
        (u8_CardStatus == ST_Success || u8_CardStatus == ST_MoreFrames) &&
        // In case of an error there is no CMAC in the response
        (mu8_LastAuthKeyNo !=
         NOT_AUTHENTICATED))                          // No session key -> no CMAC calculation possible
    {
        // For example GetCardVersion() calls DataExchange() 3 times:
        // 1. u8_Command = DF_INS_GET_VERSION      -> clear CMAC buffer + append received data
        // 2. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        // 3. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        if (u8_Command != DF_INS_ADDITIONAL_FRAME) {
            mi_CmacBuffer.Clear();
        }

        // This is an intermediate frame. More frames will follow. There is no CMAC in the response yet.
        if (u8_CardStatus == ST_MoreFrames) {
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, status))
                return -1;
        }

        //Serial.println(status);
        //Serial.println(s32_RecvSize);

        if ((status >= 8) &&             // If the response is shorter than 8 bytes it surely does not contain a CMAC
            (u8_CardStatus == ST_Success)) // Response contains CMAC only in case of success
        {
            status -= 8; // Do not return the received CMAC to the caller and do not include it into the CMAC calculation

            byte *u8_RxMac = pn532_packetbuffer + 4 + status;

            // The CMAC is calculated over the RX data + the status byte appended to the END of the RX data!
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, status) ||
                !mi_CmacBuffer.AppendUint8(u8_CardStatus))
                return -1;

            if (!mpi_SessionKey->CalculateCmac(mi_CmacBuffer, u8_CalcMac))
                return -1;

            if (mu8_DebugLevel > 1) {
                Utils::Print("RX CMAC:  ");
                Utils::PrintHexBuf(u8_CalcMac, mpi_SessionKey->GetBlockSize(), LF);
            }

            // For AES the CMAC is 16 byte, but only 8 are transmitted
            /*
            if (memcmp(u8_RxMac, u8_CalcMac, 8) != 0)
            {
                PRINT_DEBUG("CMAC Mismatch\r\n");

                Serial.print("u8_RxMac: ");
                Utils::PrintHexBuf(u8_RxMac, mpi_SessionKey->GetBlockSize(), LF);

                return -1;
            }
             */
        }
    }

    if (status > s32_RecvSize) {
        Serial.println(String("status: ") + String(status));
        Serial.println(String("s32_RecvSize: ") + String(s32_RecvSize));
        Utils::Print("DataExchange() Buffer overflow\r\n");
        return -1;
    }
    //Serial.println(String("status: ") + String(status));

    if (u8_RecvBuf && status) {
        memcpy(u8_RecvBuf, pn532_packetbuffer + 4, status);
        PRINT_DEBUG("Decrypting\n")
        Serial.println(String("e_Mac: ") + String(e_Mac));
        Serial.println(String("MAC_Rcrypt: ") + String(MAC_Rcrypt));

        if (e_Mac & MAC_Rcrypt) // decrypt received data with session key
        {
            PRINT_DEBUG("Decrypting 2\n")
            if (!mpi_SessionKey->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RecvBuf, u8_RecvBuf, status)) {
                PRINT_DEBUG("Decrypting failed\n")
                return -1;
            }

            if (mu8_DebugLevel > 1) {
                Utils::Print("Decrypt:  ");
                Utils::PrintHexBuf(u8_RecvBuf, status, LF);
            }
        }
    }
    return status;
}

/**************************************************************************
    Return true if the PN532 is ready with a response.
**************************************************************************/
bool PN532::IsReady() {
#if PROTOCOL == PROT_I2C
    {
        // After reading this byte, the bus must be released with a Stop condition
        HAL(RequestFrom)(1);
        // PN532 Manual chapter 6.2.4: Before the data bytes the chip sends a Ready byte.
        byte u8_Ready = HAL(Read)();
        if (mu8_DebugLevel > 1) {
            //Serial.print("IsReady(): read ");
            //Serial.println(u8_Ready, HEX);
            //Serial.println(String(u8_Ready == PN532_I2C_READY));
            //Serial.println(String(PN532_I2C_READY, HEX));
        }
        return u8_Ready == PN532_I2C_READY; // 0x01
    }
#endif
}

/**************************************************************************
    Waits until the PN532 is ready.
**************************************************************************/
bool PN532::WaitReady() {
    uint16_t timer = 0;
    while (!IsReady()) {
        if (timer >= PN532_I2C_TIMEOUT) {
            Utils::Print("WaitReady() -> TIMEOUT\r\n");
            return false;
        }
        Utils::DelayMilli(10);
        timer += 10;
    }
    return true;
}

bool PN532::SendCommandCheckAck(byte *cmd, byte cmdlen) {
#if PROTOCOL == PROT_HSU
    Serial.println("\nHSU");
    return HAL(writeCommand)(cmd, cmdlen) == 0;
#else
    Serial.println("\nNOT HSU");
    WriteCommand(cmd, cmdlen);
    return ReadAck();
#endif
}

void PN532::WriteCommand(byte *cmd, byte cmdlen) {
    byte TxBuffer[PN532_PACKBUFFSIZE + 10];
    int P = 0;
    TxBuffer[P++] = PN532_PREAMBLE;    // 00
    TxBuffer[P++] = PN532_STARTCODE1;  // 00
    TxBuffer[P++] = PN532_STARTCODE2;  // FF
    TxBuffer[P++] = cmdlen + 1;
    TxBuffer[P++] = 0xFF - cmdlen;
    TxBuffer[P++] = PN532_HOSTTOPN532; // D4
    for (byte i = 0; i < cmdlen; i++) {
        TxBuffer[P++] = cmd[i];
    }
    byte checksum = 0;
    for (byte i = 0; i < P; i++) {
        checksum += TxBuffer[i];
    }
    TxBuffer[P++] = ~checksum;
    TxBuffer[P++] = PN532_POSTAMBLE; // 00
    SendPacket(TxBuffer, P);
    if (mu8_DebugLevel > 1) {
        Utils::Print("Sending:  ");
        Utils::PrintHexBuf(TxBuffer, P, LF, 5, cmdlen + 6);
        DMSG("\n");
    }
}

/**************************************************************************
    Send a data packet
**************************************************************************/
void PN532::SendPacket(byte *buff, byte len) {
#if PROTOCOL == PROT_I2C
    {
        Utils::DelayMilli(2); // delay is for waking up the board
        HAL(BeginTransmission)(PN532_I2C_ADDRESS);
        for (byte i = 0; i < len; i++) {
            HAL(Write(buff[i]));
        }
        HAL(EndTransmission());
    }
#endif
}

/**************************************************************************
    Read the ACK packet (acknowledge)
**************************************************************************/
bool PN532::ReadAck() {
    const byte Ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    byte ackbuff[sizeof(Ack)];
    // ATTENTION: Never read more than 6 bytes here!
    // The PN532 has a bug in SPI mode which results in the first byte of the response missing if more than 6 bytes are read here!
    if (!ReadPacket(ackbuff, sizeof(ackbuff))) {
        Serial.println("Did not read any ACK packet");
        return false; // Timeout
    }

    if (mu8_DebugLevel > 2) {
        Utils::Print("Read ACK: ");
        Utils::PrintHexBuf(ackbuff, sizeof(ackbuff), LF);
    }
    if (memcmp(ackbuff, Ack, sizeof(Ack)) != 0) {
        Utils::Print("*** No ACK frame received\r\n");
        return false;
    }
    return true;
}

/**************************************************************************
    Reads n bytes of data from the PN532 via SPI or I2C and checks for valid data.
    param  buff      Pointer to the buffer where data will be written
    param  len       Number of bytes to read
    returns the number of bytes that have been copied to buff (< len) or 0 on error
**************************************************************************/
byte PN532::ReadData(byte *buff, byte len) {
    byte RxBuffer[PN532_PACKBUFFSIZE];
    const byte MIN_PACK_LEN = 2 /*start bytes*/ + 2 /*length + length checksum */ + 1 /*checksum*/;
    if (len < MIN_PACK_LEN || len > PN532_PACKBUFFSIZE) {
        Utils::Print("ReadData(): len is invalid\r\n");
        return 0;
    }
    if (!ReadPacket(RxBuffer, len))
        return 0; // timeout
    // The following important validity check was completely missing in Adafruit code (added by Elmü)
    // PN532 documentation says (chapter 6.2.1.6):
    // Before the start code (0x00 0xFF) there may be any number of additional bytes that must be ignored.
    // After the checksum there may be any number of additional bytes that must be ignored.
    // This function returns ONLY the pure data bytes:
    // any leading bytes -> skipped (never seen, but documentation says to ignore them)
    // preamble   0x00   -> skipped (optional, the PN532 does not send it always!!!!!)
    // start code 0x00   -> skipped
    // start code 0xFF   -> skipped
    // length            -> skipped
    // length checksum   -> skipped
    // data[0...n]       -> returned to the caller (first byte is always 0xD5)
    // checksum          -> skipped
    // postamble         -> skipped (optional, the PN532 may not send it!)
    // any bytes behind  -> skipped (never seen, but documentation says to ignore them)
    const char *Error = NULL;
    int Brace1 = -1;
    int Brace2 = -1;
    int dataLength = 0;
    do {
        int startCode = -1;
        for (int i = 0; i <= len - MIN_PACK_LEN; i++) {
            if (RxBuffer[i] == PN532_STARTCODE1 &&
                RxBuffer[i + 1] == PN532_STARTCODE2) {
                startCode = i;
                break;
            }
        }
        if (startCode < 0) {
            Error = "ReadData() -> No Start Code\r\n";
            break;
        }
        int pos = startCode + 2;
        dataLength = RxBuffer[pos++];
        int lengthCheck = RxBuffer[pos++];
        if ((dataLength + lengthCheck) != 0x100) {
            Error = "ReadData() -> Invalid length checksum\r\n";
            break;
        }
        if (len < startCode + MIN_PACK_LEN + dataLength) {
            Error = "ReadData() -> Packet is longer than requested length\r\n";
            break;
        }
        Brace1 = pos;
        for (int i = 0; i < dataLength; i++) {
            buff[i] = RxBuffer[pos++]; // copy the pure data bytes in the packet
        }
        Brace2 = pos;
        // All returned data blocks must start with PN532TOHOST (0xD5)
        if (dataLength < 1 || buff[0] != PN532_PN532TOHOST) {
            Error = "ReadData() -> Invalid data (no PN532TOHOST)\r\n";
            break;
        }
        byte checkSum = 0;
        for (int i = startCode; i < pos; i++) {
            checkSum += RxBuffer[i];
        }
        if (checkSum != (byte)(~RxBuffer[pos])) {
            Error = "ReadData() -> Invalid checksum\r\n";
            break;
        }
    } while (false); // This is not a loop. Avoids using goto by using break.
    // Always print the package, even if it was invalid.
    if (mu8_DebugLevel > 1) {
        PRINT_DEBUG("Response: ")
        Utils::PrintHexBuf(RxBuffer, len, LF, Brace1, Brace2);
    }
    if (Error) {
        Utils::Print(Error);
        return 0;
    }
    return dataLength;
}

/**************************************************************************
   Reads n bytes of data from the PN532 via SPI or I2C and does NOT check for valid data.
    param  buff      Pointer to the buffer where data will be written
    param  len       Number of bytes to read
**************************************************************************/
bool PN532::ReadPacket(byte *buff, byte len) {
#if PROTOCOL == PROT_HSU
    // TODO PN532_ACK_WAIT_TIME*3 is empirically shown to work here, lowet than that times out
    if (HAL(receive)(buff, len, PN532_ACK_WAIT_TIME*3) <= 0) {
        PRINT_DEBUG("Timeout\n");
        return false;
    }
    return true;
#else
    if (!WaitReady())
        return false;
#if PROTOCOL == PROT_I2C
        {
        Utils::DelayMilli(2);
        // read (n+1 to take into account leading Ready byte)
        HAL(RequestFrom)(len + 1);
        // PN532 Manual chapter 6.2.4: Before the data bytes the chip sends a Ready byte.
        // It is ignored here because it has been checked already in isready()
        byte u8_Ready = HAL(Read)();
        if (mu8_DebugLevel > 2) {
            Utils::Print("ReadPacket(): read ");
            Utils::PrintHex8(u8_Ready, LF);
        }
        for (byte i = 0; i < len; i++) {
            Utils::DelayMilli(1);
            buff[i] = HAL(Read)();
        }
        return true;
    }
#endif // if I2C
#endif // if HSU
}

// Checks the status byte that is returned from the card
bool PN532::CheckCardStatus(DESFireStatus e_Status) {
    switch (e_Status) {
        case ST_Success:    // Success
        case ST_NoChanges:  // No changes made
        case ST_MoreFrames: // Another frame will follow
            return true;

        default:
            break; // This is just to avoid stupid gcc compiler warnings
    }

    Utils::Print("Desfire Error: ");
    switch (e_Status) {
        case ST_OutOfMemory:
            Utils::Print("Not enough EEPROM memory.\r\n");
            return false;
        case ST_IllegalCommand:
            Utils::Print("Illegal command.\r\n");
            return false;
        case ST_IntegrityError:
            Utils::Print("Integrity error.\r\n");
            return false;
        case ST_KeyDoesNotExist:
            Utils::Print("Key does not exist.\r\n");
            return false;
        case ST_WrongCommandLen:
            Utils::Print("Wrong command length.\r\n");
            return false;
        case ST_PermissionDenied:
            Utils::Print("Permission denied.\r\n");
            return false;
        case ST_IncorrectParam:
            Utils::Print("Incorrect parameter.\r\n");
            return false;
        case ST_AppNotFound:
            Utils::Print("Application not found.\r\n");
            return false;
        case ST_AppIntegrityError:
            Utils::Print("Application integrity error.\r\n");
            return false;
        case ST_AuthentError:
            Utils::Print("Authentication error.\r\n");
            return false;
        case ST_LimitExceeded:
            Utils::Print("Limit exceeded.\r\n");
            return false;
        case ST_CardIntegrityError:
            Utils::Print("Card integrity error.\r\n");
            return false;
        case ST_CommandAborted:
            Utils::Print("Command aborted.\r\n");
            return false;
        case ST_CardDisabled:
            Utils::Print("Card disabled.\r\n");
            return false;
        case ST_InvalidApp:
            Utils::Print("Invalid application.\r\n");
            return false;
        case ST_DuplicateAidFiles:
            Utils::Print("Duplicate AIDs or files.\r\n");
            return false;
        case ST_EepromError:
            Utils::Print("EEPROM error.\r\n");
            return false;
        case ST_FileNotFound:
            Utils::Print("File not found.\r\n");
            return false;
        case ST_FileIntegrityError:
            Utils::Print("File integrity error.\r\n");
            return false;
        default:
            Utils::Print("0x");
            Utils::PrintHex8((byte) e_Status, LF);
            return false;
    }
}

// Generate multi byte random
void GenerateRandom(byte *u8_Random, int s32_Length) {
    uint32_t u32_Now = millis();
    for (int i = 0; i < s32_Length; i++) {
        u8_Random[i] = (byte) u32_Now;
        u32_Now *= 127773;
        u32_Now += 16807;
    }
}


// Whenever the RF field is switched off, these variables must be reset
bool PN532::SwitchOffRfField() {
    mu8_LastAuthKeyNo = NOT_AUTHENTICATED;
    mu32_LastApplication = 0x000000; // No application selected

    return PN532::SwitchOffRfField();
}

/**************************************************************************
    Does an ISO authentication with a 2K3DES key or an AES authentication with an AES key.
    pi_Key must be an instance of DES or AES.
    The authentication is a 3-pass process where both sides prove that they use the same master key
    without ever exposing that key. Only random values are exchanged.
    Not all commands require authentication.
    If you want to authenticate for an application you must call SelectApplication() first.
    If you select application 0x000000 pi_Key must be the PICC master key (set u8_KeyNo = 0),
    otherwise one of the up to 14 application keys is chosen with u8_KeyNo.
    IMPORTANT: If the card expects the 3K3DES default key you must pass a 3K3DES key full of 24 zeroes,
    although this is in reality a simple DES key (K1 == K2 == K3). Otherwise the session key is calculated wrong.
**************************************************************************/
bool PN532::Authenticate(byte u8_KeyNo, DESFireKey *pi_Key) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** Authenticate(KeyNo= %d, Key= ", u8_KeyNo);
        Utils::Print(s8_Buf);
        pi_Key->PrintKey();
        Utils::Print(")\r\n");
    }

    byte u8_Command;
    switch (pi_Key->GetKeyType()) {
        case DF_KEY_AES:
            u8_Command = DFEV1_INS_AUTHENTICATE_AES;
            break;
        case DF_KEY_2K3DES:
        case DF_KEY_3K3DES:
            u8_Command = DFEV1_INS_AUTHENTICATE_ISO;
            break;
        default:
            Utils::Print("Invalid key\r\n");
            return false;
    }

    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_KeyNo);

    // Request a random of 16 byte, but depending of the key the PICC may also return an 8 byte random
    DESFireStatus e_Status;
    byte u8_RndB_enc[16]; // encrypted random B
    int s32_Read = DataExchange(u8_Command, &i_Params, u8_RndB_enc, 16, &e_Status, MAC_None);
    if (e_Status != ST_MoreFrames || (s32_Read != 8 && s32_Read != 16)) {
        Utils::Print("Authentication failed (1)\r\n");
        return false;
    }

    int s32_RandomSize = s32_Read;

    byte u8_RndB[16];  // decrypted random B
    pi_Key->ClearIV(); // Fill IV with zeroes !ONLY ONCE HERE!
    if (!pi_Key->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RndB, u8_RndB_enc, s32_RandomSize))
        return false;  // key not set

    byte u8_RndB_rot[16]; // rotated random B
    Utils::RotateBlockLeft(u8_RndB_rot, u8_RndB, s32_RandomSize);

    byte u8_RndA[16];
    Utils::GenerateRandom(u8_RndA, s32_RandomSize);

    TX_BUFFER(i_RndAB, 32); // (randomA + rotated randomB)
    i_RndAB.AppendBuf(u8_RndA, s32_RandomSize);
    i_RndAB.AppendBuf(u8_RndB_rot, s32_RandomSize);

    TX_BUFFER(i_RndAB_enc, 32); // encrypted (randomA + rotated randomB)
    i_RndAB_enc.SetCount(2 * s32_RandomSize);
    if (!pi_Key->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, i_RndAB_enc, i_RndAB, 2 * s32_RandomSize))
        return false;

    if (mu8_DebugLevel > 0) {
        Utils::Print("* RndB_enc:  ");
        Utils::PrintHexBuf(u8_RndB_enc, s32_RandomSize, LF);
        Utils::Print("* RndB:      ");
        Utils::PrintHexBuf(u8_RndB, s32_RandomSize, LF);
        Utils::Print("* RndB_rot:  ");
        Utils::PrintHexBuf(u8_RndB_rot, s32_RandomSize, LF);
        Utils::Print("* RndA:      ");
        Utils::PrintHexBuf(u8_RndA, s32_RandomSize, LF);
        Utils::Print("* RndAB:     ");
        Utils::PrintHexBuf(i_RndAB, 2 * s32_RandomSize, LF);
        Utils::Print("* RndAB_enc: ");
        Utils::PrintHexBuf(i_RndAB_enc, 2 * s32_RandomSize, LF);
    }

    byte u8_RndA_enc[16]; // encrypted random A
    s32_Read = DataExchange(DF_INS_ADDITIONAL_FRAME, &i_RndAB_enc, u8_RndA_enc, s32_RandomSize, &e_Status, MAC_None);
    if (e_Status != ST_Success || s32_Read != s32_RandomSize) {
        Utils::Print("Authentication failed (2)\r\n");
        return false;
    }

    byte u8_RndA_dec[16]; // decrypted random A
    if (!pi_Key->CryptDataCBC(CBC_RECEIVE, KEY_DECIPHER, u8_RndA_dec, u8_RndA_enc, s32_RandomSize))
        return false;

    byte u8_RndA_rot[16]; // rotated random A
    Utils::RotateBlockLeft(u8_RndA_rot, u8_RndA, s32_RandomSize);

    if (mu8_DebugLevel > 0) {
        Utils::Print("* RndA_enc:  ");
        Utils::PrintHexBuf(u8_RndA_enc, s32_RandomSize, LF);
        Utils::Print("* RndA_dec:  ");
        Utils::PrintHexBuf(u8_RndA_dec, s32_RandomSize, LF);
        Utils::Print("* RndA_rot:  ");
        Utils::PrintHexBuf(u8_RndA_rot, s32_RandomSize, LF);
    }

    // Last step: Check if the received random A is equal to the sent random A.
    if (memcmp(u8_RndA_dec, u8_RndA_rot, s32_RandomSize) != 0) {
        Utils::Print("Authentication failed (3)\r\n");
        return false;
    }

    // The session key is composed from RandA and RndB
    TX_BUFFER(i_SessKey, 24);
    i_SessKey.AppendBuf(u8_RndA, 4);
    i_SessKey.AppendBuf(u8_RndB, 4);

    if (pi_Key->GetKeySize() > 8) // the following block is not required for simple DES
    {
        switch (pi_Key->GetKeyType()) {
            case DF_KEY_2K3DES:
                i_SessKey.AppendBuf(u8_RndA + 4, 4);
                i_SessKey.AppendBuf(u8_RndB + 4, 4);
                break;

            case DF_KEY_3K3DES:
                i_SessKey.AppendBuf(u8_RndA + 6, 4);
                i_SessKey.AppendBuf(u8_RndB + 6, 4);
                i_SessKey.AppendBuf(u8_RndA + 12, 4);
                i_SessKey.AppendBuf(u8_RndB + 12, 4);
                break;

            case DF_KEY_AES:
                i_SessKey.AppendBuf(u8_RndA + 12, 4);
                i_SessKey.AppendBuf(u8_RndB + 12, 4);
                break;

            default: // avoid stupid gcc compiler warning
                break;
        }
    }

    if (pi_Key->GetKeyType() == DF_KEY_AES) mpi_SessionKey = &mi_AesSessionKey;
    else mpi_SessionKey = &mi_DesSessionKey;

    if (!mpi_SessionKey->SetKeyData(i_SessKey, i_SessKey.GetCount(), 0))
        return false;

    if (mu8_DebugLevel > 0) {
        PRINT_DEBUG("* SessKey:   ");
        mpi_SessionKey->PrintKey(LF);
    }

    mu8_LastAuthKeyNo = u8_KeyNo;
    return true;
}

/**************************************************************************
    ATTENTION:
    Be very careful when you change the PICC master key (for application {0x000000})!
    If you don't know what you are doing you may have to throw the card into the dustbin!
    There is NO way to reanimate the card when you lost the master key.
    -----------------------------------------------------------------------
    Does a key change. You must first call Authenticate().
    To change an application key you must also call SelectApplication().
    To make it complicated NXP defines two different procedures:
    Changing the same key number that was used for authentication and changing another key.
    After changing a key you have to authenticate again with the new key.
    pi_CurKey must be the old key (currently stored in u8_KeyNo) that you want to change into pi_NewKey.
    pi_CurKey may be NULL if you change the same key number that was used for authetication
    or if the current key is the factory default key.
**************************************************************************/
bool PN532::ChangeKey(byte u8_KeyNo, DESFireKey *pi_NewKey, DESFireKey *pi_CurKey) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** ChangeKey(KeyNo= %d)\r\n", u8_KeyNo);
        Utils::Print(s8_Buf);
    }

    if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED) {
        Utils::Print("Not authenticated\r\n");
        return false;
    }

    if (mu8_DebugLevel > 0) {
        Utils::Print("* SessKey IV:  ");
        mpi_SessionKey->PrintIV(LF);
        Utils::Print("* New Key:     ");
        pi_NewKey->PrintKey(LF);
    }

    if (!DESFireKey::CheckValid(pi_NewKey))
        return false;

    TX_BUFFER(i_Cryptogram, 40);
    i_Cryptogram.AppendBuf(pi_NewKey->Data(), pi_NewKey->GetKeySize(16));

    bool b_SameKey = (u8_KeyNo ==
                      mu8_LastAuthKeyNo);  // false -> change another key than the one that was used for authentication

    // The type of key can only be changed for the PICC master key.
    // Applications must define their key type in CreateApplication().
    if (mu32_LastApplication == 0x000000)
        u8_KeyNo |= pi_NewKey->GetKeyType();

    // The following if() applies only to application keys.
    // For the PICC master key b_SameKey is always true because there is only ONE key (#0) at the PICC level.
    if (!b_SameKey) {
        if (!DESFireKey::CheckValid(pi_CurKey))
            return false;

        if (mu8_DebugLevel > 0) {
            Utils::Print("* Cur Key:     ");
            pi_CurKey->PrintKey(LF);
        }

        // The current key and the new key must be XORed
        Utils::XorDataBlock(i_Cryptogram, pi_CurKey->Data(), pi_CurKey->GetKeySize(16));
    }

    // While DES stores the key version in bit 0 of the key bytes, AES transmits the version separately
    if (pi_NewKey->GetKeyType() == DF_KEY_AES) {
        i_Cryptogram.AppendUint8(pi_NewKey->GetKeyVersion());
    }

    byte u8_Command[] = {DF_INS_CHANGE_KEY, u8_KeyNo};
    uint32_t u32_Crc = Utils::CalcCrc32(u8_Command, 2, i_Cryptogram, i_Cryptogram.GetCount());
    i_Cryptogram.AppendUint32(u32_Crc);

    if (mu8_DebugLevel > 0) {
        Utils::Print("* CRC Crypto:  0x");
        Utils::PrintHex32(u32_Crc, LF);
    }

    if (!b_SameKey) {
        uint32_t u32_CrcNew = Utils::CalcCrc32(pi_NewKey->Data(), pi_NewKey->GetKeySize(16));
        i_Cryptogram.AppendUint32(u32_CrcNew);

        if (mu8_DebugLevel > 0) {
            Utils::Print("* CRC New Key: 0x");
            Utils::PrintHex32(u32_CrcNew, LF);
        }
    }

    // Get the padded length of the Cryptogram to be encrypted
    int s32_CryptoLen = 24;
    if (i_Cryptogram.GetCount() > 24) s32_CryptoLen = 32;
    if (i_Cryptogram.GetCount() > 32) s32_CryptoLen = 40;

    // For a blocksize of 16 byte (AES) the data length 24 is not valid -> increase to 32
    s32_CryptoLen = mpi_SessionKey->CalcPaddedBlockSize(s32_CryptoLen);

    byte u8_Cryptogram_enc[40] = {0}; // encrypted cryptogram
    if (!mpi_SessionKey->CryptDataCBC(CBC_SEND, KEY_ENCIPHER, u8_Cryptogram_enc, i_Cryptogram, s32_CryptoLen))
        return false;

    if (mu8_DebugLevel > 0) {
        Utils::Print("* Cryptogram:  ");
        Utils::PrintHexBuf(i_Cryptogram, s32_CryptoLen, LF);
        Utils::Print("* Cryptog_enc: ");
        Utils::PrintHexBuf(u8_Cryptogram_enc, s32_CryptoLen, LF);
    }

    TX_BUFFER(i_Params, 41);
    i_Params.AppendUint8(u8_KeyNo);
    i_Params.AppendBuf(u8_Cryptogram_enc, s32_CryptoLen);

    // If the same key has been changed the session key is no longer valid. (Authentication required)
    if (b_SameKey) mu8_LastAuthKeyNo = NOT_AUTHENTICATED;

    return (0 == DataExchange(DF_INS_CHANGE_KEY, &i_Params, NULL, 0, NULL, MAC_Rmac));
}

/**************************************************************************
    Get the version of the key (optional)
    To store a version number in the key use DES::SetKeyVersion()
    before calling PN532::ChangeKey()
**************************************************************************/
bool PN532::GetKeyVersion(byte u8_KeyNo, byte *pu8_Version) {
    char s8_Buf[80];
    if (mu8_DebugLevel > 0) {
        sprintf(s8_Buf, "\r\n*** GetKeyVersion(KeyNo= %d)\r\n", u8_KeyNo);
        Utils::Print(s8_Buf);
    }

    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_KeyNo);

    if (1 != DataExchange(DF_INS_GET_KEY_VERSION, &i_Params, pu8_Version, 1, NULL, MAC_TmacRmac))
        return false;

    if (mu8_DebugLevel > 0) {
        Utils::Print("Version: 0x");
        Utils::PrintHex8(*pu8_Version, LF);
    }
    return true;
}

/**************************************************************************
    Reads several production details of the Desfire card
    If RandomID mode is active, the UID will be returned as 00 00 00 00 00 00 00
**************************************************************************/
bool PN532::GetCardVersion(DESFireCardVersion *pk_Version) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetCardVersion()\r\n");

    byte *pu8_Ptr = (byte *) pk_Version;

    DESFireStatus e_Status;
    int s32_Read = DataExchange(DF_INS_GET_VERSION, NULL, pu8_Ptr, 7, &e_Status, MAC_TmacRmac);
    if (s32_Read != 7 || e_Status != ST_MoreFrames) {
        if (mu8_DebugLevel > 0 && e_Status != ST_MoreFrames)
            Serial.println(String("Failed (e_Status == ") + String(e_Status, HEX) + ")");
        if (mu8_DebugLevel > 0 && s32_Read != 7)
            Serial.println(String("Failed (s32_Read == ") + String(s32_Read, DEC) + ")");
        return false;
    }

    pu8_Ptr += 7;
    s32_Read = DataExchange(DF_INS_ADDITIONAL_FRAME, NULL, pu8_Ptr, 7, &e_Status, MAC_Rmac);
    if (s32_Read != 7 || e_Status != ST_MoreFrames) {
        if (mu8_DebugLevel > 0 && e_Status != ST_MoreFrames)
            Serial.println(String("2 Failed (e_Status == ") + String(e_Status, HEX) + ")");
        if (mu8_DebugLevel > 0 && s32_Read != 7)
            Serial.println(String("2 Failed (s32_Read == ") + String(s32_Read, DEC) + ")");
        return false;
    }

    pu8_Ptr += 7;
    s32_Read = DataExchange(DF_INS_ADDITIONAL_FRAME, NULL, pu8_Ptr, 14, &e_Status, MAC_Rmac);
    if (s32_Read != 14 || e_Status != ST_Success) {
        if (mu8_DebugLevel > 0 && e_Status != ST_Success)
            Serial.println(String("3 Failed (e_Status == ") + String(e_Status, HEX) + ")");
        if (mu8_DebugLevel > 0 && s32_Read != 14)
            Serial.println(String("3 Failed (s32_Read == ") + String(s32_Read, DEC) + ")");
        return false;
    }

    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        Utils::Print("--- Desfire Card Details ---\r\n");
        sprintf(s8_Buf, "Hardware Version: %d.%d\r\n", pk_Version->hardwareMajVersion, pk_Version->hardwareMinVersion);
        Utils::Print(s8_Buf);
        sprintf(s8_Buf, "Software Version: %d.%d\r\n", pk_Version->softwareMajVersion, pk_Version->softwareMinVersion);
        Utils::Print(s8_Buf);
        sprintf(s8_Buf, "EEPROM size:      %d byte\r\n", 1 << (pk_Version->hardwareStorageSize / 2));
        Utils::Print(s8_Buf);
        sprintf(s8_Buf, "Production:       week %X, year 20%02X\r\n", pk_Version->cwProd, pk_Version->yearProd);
        Utils::Print(s8_Buf);
        Utils::Print("UID no:           ");
        Utils::PrintHexBuf(pk_Version->uid, 7, LF);
        Utils::Print("Batch no:         ");
        Utils::PrintHexBuf(pk_Version->batchNo, 5, LF);
    }
    return true;
}

/**************************************************************************
    Erases all content from the card (all files and all applications)
**************************************************************************/
bool PN532::FormatCard() {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** FormatCard()\r\n");

    return (0 == DataExchange(DF_INS_FORMAT_PICC, NULL, NULL, 0, NULL, MAC_TmacRmac));
}

/**************************************************************************
    Gets the settings of the master key.
    First you must call SelectApplication()
    After selecting the application 0x000000 the settings of the PICC master key
    will be returned, otherwise the settings of the selected application master key.
    pu8_KeyCount will contain the max number of keys for an application.
    pu8_KeyCount will be = 1 for the application ID 0x000000.
    pe_KeyType returns the type of key for the application (2K3DES / AES)
**************************************************************************/
bool PN532::GetKeySettings(DESFireKeySettings *pe_Settg, byte *pu8_KeyCount, DESFireKeyType *pe_KeyType) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetKeySettings()\r\n");

    byte u8_RetData[2];
    if (2 != DataExchange(DF_INS_GET_KEY_SETTINGS, NULL, u8_RetData, 2, NULL, MAC_TmacRmac))
        return false;

    *pe_Settg = (DESFireKeySettings) u8_RetData[0];
    *pu8_KeyCount = u8_RetData[1] & 0x0F;
    *pe_KeyType = (DESFireKeyType) (u8_RetData[1] & 0xF0);

    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "Settings: 0x%02X, KeyCount: %d, KeyType: %s\r\n", *pe_Settg, *pu8_KeyCount,
                DESFireKey::GetKeyTypeAsString(*pe_KeyType));
        Utils::Print(s8_Buf);
    }
    return true;
}

/**************************************************************************
    Changes the settings of the PICC or application master key.
    First you must call SelectApplication() and authenticate with the master key.
**************************************************************************/
bool PN532::ChangeKeySettings(DESFireKeySettings e_NewSettg) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** ChangeKeySettings(0x%02X)\r\n", e_NewSettg);
        Utils::Print(s8_Buf);
    }

    TX_BUFFER(i_Params, 16);
    i_Params.AppendUint8(e_NewSettg);

    // The TX CMAC must not be calculated here because a CBC encryption operation has already been executed
    return (0 == DataExchange(DF_INS_CHANGE_KEY_SETTINGS, &i_Params, NULL, 0, NULL, MAC_TcryptRmac));
}

/**************************************************************************
    Get the remaining free memory on the card.
    NOTE: This function gives stranges results:
    8k Card formatted: EPPROM size: 8192 bytes > Free memory: 7936 bytes.
    4k Card formatted: EPPROM size: 4096 bytes < Free memory: 4864 bytes!
**************************************************************************/
bool PN532::GetFreeMemory(uint32_t *pu32_Memory) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetFreeMemory()\r\n");

    *pu32_Memory = 0;

    RX_BUFFER(i_Data, 3);
    if (3 != DataExchange(DFEV1_INS_FREE_MEM, NULL, i_Data, 3, NULL, MAC_TmacRmac))
        return false;

    *pu32_Memory = i_Data.ReadUint24();

    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "Free memory: %d bytes\r\n", (int) *pu32_Memory);
        Utils::Print(s8_Buf);
    }
    return true;
}

/**************************************************************************
    returns all Application ID's (AID) stored on the card. (maximum = 28 / card)
    Each application ID is 3 bytes.
    pu32_IDlist:   Must point to an uint32_t[28] array
    ps32_AppCount: The count of DESFireAppId's that have been stored in pk_IDlist.
**************************************************************************/
bool PN532::GetApplicationIDs(uint32_t u32_IDlist[28], byte *pu8_AppCount) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetApplicationIDs()\r\n");

    memset(u32_IDlist, 0, 28 * sizeof(uint32_t));

    RX_BUFFER(i_RxBuf, 28 * 3); // 3 byte per application
    byte *pu8_Ptr = i_RxBuf;

    DESFireStatus e_Status;
    int s32_Read1 = DataExchange(DF_INS_GET_APPLICATION_IDS, NULL, pu8_Ptr, MAX_FRAME_SIZE, &e_Status, MAC_TmacRmac);
    if (s32_Read1 < 0) {
        if (mu8_DebugLevel > 0) Serial.println("no response in get application ids");
        return false;
    }

    // If there are more than 19 applications, they will be sent in two frames
    int s32_Read2 = 0;
    if (e_Status == ST_MoreFrames) {
        pu8_Ptr += s32_Read1;
        s32_Read2 = DataExchange(DF_INS_ADDITIONAL_FRAME, NULL, pu8_Ptr, 28 * 3 - s32_Read1, NULL, MAC_Rmac);
        if (s32_Read2 < 0)
            return false;
    }

    i_RxBuf.SetSize(s32_Read1 + s32_Read2);
    *pu8_AppCount = (s32_Read1 + s32_Read2) / 3;

    // Convert 3 byte array -> 4 byte array
    for (byte i = 0; i < *pu8_AppCount; i++) {
        u32_IDlist[i] = i_RxBuf.ReadUint24();
    }

    if (mu8_DebugLevel > 0) {
        if (*pu8_AppCount == 0) {
            Utils::Print("No Application ID's.\r\n");
        } else
            for (byte i = 0; i < *pu8_AppCount; i++) {
                char s8_Buf[80];
                sprintf(s8_Buf, "Application %2d: 0x%06X\r\n", i, (unsigned int) u32_IDlist[i]);
                Utils::Print(s8_Buf);
            }
    }
    return true;
}

/**************************************************************************
    Creates a new application
    You must call SelectApplication(0x000000) before and authenticate with the PICC master key!
    u32_AppID:   The unique ID of the application
    e_Settg:     The application master key settings
    u8_KeyCount: The count of keys to be stored in the application
    e_KeyType:   Defines the key type for the application (2K3DES / AES)
**************************************************************************/
bool
PN532::CreateApplication(uint32_t u32_AppID, DESFireKeySettings e_Settg, byte u8_KeyCount, DESFireKeyType e_KeyType) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** CreateApplication(App= 0x%06X, KeyCount= %d, Type= %s)\r\n", (unsigned int) u32_AppID,
                u8_KeyCount, DESFireKey::GetKeyTypeAsString(e_KeyType));
        Utils::Print(s8_Buf);
    }

    if (e_KeyType == DF_KEY_INVALID) {
        Utils::Print("Invalid key type\r\n");
        return false;
    }

    TX_BUFFER(i_Params, 5);
    i_Params.AppendUint24(u32_AppID);
    i_Params.AppendUint8(e_Settg);
    i_Params.AppendUint8(u8_KeyCount | e_KeyType);

    return (0 == DataExchange(DF_INS_CREATE_APPLICATION, &i_Params, NULL, 0, NULL, MAC_TmacRmac));
}

/**************************************************************************
    Deletes an application after checking that it exists.
    When you call DeleteApplication() and the application does not exist,
    an ST_AppNotFound error is returned and the authentication is invalidated.
    To avoid this error, this function calls first GetApplicationIDs()
**************************************************************************/
bool PN532::DeleteApplicationIfExists(uint32_t u32_AppID) {
    uint32_t u32_IDlist[28];
    byte u8_AppCount;
    if (!GetApplicationIDs(u32_IDlist, &u8_AppCount))
        return false;

    bool b_Found = false;
    for (byte i = 0; i < u8_AppCount; i++) {
        if (u32_IDlist[i] == u32_AppID)
            b_Found = true;
    }
    if (!b_Found)
        return true;

    return DeleteApplication(u32_AppID);
}

/**************************************************************************
    Deletes an application
    You must call SelectApplication(0x000000) before and authenticate with the PICC master key!
**************************************************************************/
bool PN532::DeleteApplication(uint32_t u32_AppID) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** DeleteApplication(0x%06X)\r\n", (unsigned int) u32_AppID);
        Utils::Print(s8_Buf);
    }

    TX_BUFFER(i_Params, 3);
    i_Params.AppendUint24(u32_AppID);

    return (0 == DataExchange(DF_INS_DELETE_APPLICATION, &i_Params, NULL, 0, NULL, MAC_TmacRmac));
}

/**************************************************************************
    Selects an application
    If u8_AppID is 0x000000 the PICC level is selected
**************************************************************************/
bool PN532::SelectApplication(uint32_t u32_AppID) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** SelectApplication(0x%06X)\r\n", (unsigned int) u32_AppID);
        Utils::Print(s8_Buf);
    }

    TX_BUFFER(i_Params, 3);
    i_Params.AppendUint24(u32_AppID);

    // This command does not return a CMAC because after selecting another application the session key is no longer valid. (Authentication required)
    if (0 != DataExchange(DF_INS_SELECT_APPLICATION, &i_Params, NULL, 0, NULL, MAC_None))
        return false;

    mu8_LastAuthKeyNo = NOT_AUTHENTICATED; // set to invalid value (the selected app requires authentication)
    mu32_LastApplication = u32_AppID;
    return true;
}

/**************************************************************************
    returns all File ID's for the selected application.
    Desfire EV1: maximum = 32 files per application.
    u8_FileIDs:     Buffer of 32 bytes
    ps32_FileCount: The count of file Id's that have been written to u8_FileIDs.
**************************************************************************/
bool PN532::GetFileIDs(byte *u8_FileIDs, byte *pu8_FileCount) {
    if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetFileIDs()\r\n");

    int s32_Read = DataExchange(DF_INS_GET_FILE_IDS, NULL, u8_FileIDs, 32, NULL, MAC_TmacRmac);
    if (s32_Read < 0)
        return false;

    *pu8_FileCount = s32_Read;

    if (mu8_DebugLevel > 0) {
        if (*pu8_FileCount == 0) {
            Utils::Print("No files.\r\n");
        } else {
            Utils::Print("File ID's: ");
            Utils::PrintHexBuf(u8_FileIDs, s32_Read, LF);
        }
    }
    return true;
}

/**************************************************************************
    Gets the settings of a file.
**************************************************************************/
bool PN532::GetFileSettings(byte u8_FileID, DESFireFileSettings *pk_Settings) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** GetFileSettings(ID= %d)\r\n", u8_FileID);
        Utils::Print(s8_Buf);
    }

    memset(pk_Settings, 0, sizeof(DESFireFileSettings));

    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_FileID);

    RX_BUFFER(i_RetData, 20);
    int s32_Read = DataExchange(DF_INS_GET_FILE_SETTINGS, &i_Params, i_RetData, 20, NULL, MAC_TmacRmac);
    if (s32_Read < 7)
        return false;

    i_RetData.SetSize(s32_Read);

    pk_Settings->e_FileType = (DESFireFileType) i_RetData.ReadUint8();
    pk_Settings->e_Encrypt = (DESFireFileEncryption) i_RetData.ReadUint8();
    pk_Settings->k_Permis.Unpack(i_RetData.ReadUint16());

    char s8_Buf[150];
    if (mu8_DebugLevel > 0) {
        sprintf(s8_Buf, "Type: %d, Encrypt: %d, Access Read: 0x%X, Write: 0x%X, Rd+Wr: 0x%X, Change: 0x%X\r\n",
                pk_Settings->e_FileType, pk_Settings->e_Encrypt,
                pk_Settings->k_Permis.e_ReadAccess, pk_Settings->k_Permis.e_WriteAccess,
                pk_Settings->k_Permis.e_ReadAndWriteAccess, pk_Settings->k_Permis.e_ChangeAccess);
        Utils::Print(s8_Buf);
    }

    switch (pk_Settings->e_FileType) {
        case MDFT_STANDARD_DATA_FILE:
        case MDFT_BACKUP_DATA_FILE:
            pk_Settings->u32_FileSize = i_RetData.ReadUint24();

            if (mu8_DebugLevel > 0) {
                sprintf(s8_Buf, "FileSize: %d\r\n", (int) pk_Settings->u32_FileSize);
                Utils::Print(s8_Buf);
            }
            return true;

        case MDFT_VALUE_FILE_WITH_BACKUP:
            pk_Settings->u32_LowerLimit = i_RetData.ReadUint32();
            pk_Settings->u32_UpperLimit = i_RetData.ReadUint32();
            pk_Settings->u32_LimitedCreditValue = i_RetData.ReadUint32();
            pk_Settings->b_LimitedCreditEnabled = i_RetData.ReadUint8() == 0x01;

            if (mu8_DebugLevel > 0) {
                sprintf(s8_Buf, "LowerLimit: %d, UpperLimit: %d, CreditValue: %d, LimitEnabled: %d\r\n",
                        (int) pk_Settings->u32_LowerLimit, (int) pk_Settings->u32_UpperLimit,
                        (int) pk_Settings->u32_LimitedCreditValue, (int) pk_Settings->b_LimitedCreditEnabled);
                Utils::Print(s8_Buf);
            }
            return true;

        case MDFT_LINEAR_RECORD_FILE_WITH_BACKUP:
        case MDFT_CYCLIC_RECORD_FILE_WITH_BACKUP:
            pk_Settings->u32_RecordSize = i_RetData.ReadUint24();
            pk_Settings->u32_MaxNumberRecords = i_RetData.ReadUint24();
            pk_Settings->u32_CurrentNumberRecords = i_RetData.ReadUint24();

            if (mu8_DebugLevel > 0) {
                sprintf(s8_Buf, "RecordSize: %d, MaxRecords: %d, CurrentRecords: %d\r\n",
                        (int) pk_Settings->u32_RecordSize, (int) pk_Settings->u32_MaxNumberRecords,
                        (int) pk_Settings->u32_CurrentNumberRecords);
                Utils::Print(s8_Buf);
            }
            return true;

        default:
            return false; // unknown file type
    }
}

/**************************************************************************
    Creates a standard data file (a simple binary file) of a fixed size in the selected application.
**************************************************************************/
bool PN532::CreateStdDataFile(byte u8_FileID, DESFireFilePermissions *pk_Permis, int s32_FileSize) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** CreateStdDataFile(ID= %d, Size= %d)\r\n", u8_FileID, s32_FileSize);
        Utils::Print(s8_Buf);
    }

    uint16_t u16_Permis = pk_Permis->Pack();

    TX_BUFFER(i_Params, 7);
    i_Params.AppendUint8(u8_FileID);
    i_Params.AppendUint8(CM_PLAIN);
    i_Params.AppendUint16(u16_Permis);
    i_Params.AppendUint24(s32_FileSize); // only the low 3 bytes are used

    return (0 == DataExchange(DF_INS_CREATE_STD_DATA_FILE, &i_Params, NULL, 0, NULL, MAC_TmacRmac));

}

/**************************************************************************
    Deletes a file in the selected application
**************************************************************************/
bool PN532::DeleteFile(byte u8_FileID) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** DeleteFile(ID= %d)\r\n", u8_FileID);
        Utils::Print(s8_Buf);
    }

    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_FileID);

    return (0 == DataExchange(DF_INS_DELETE_FILE, &i_Params, NULL, 0, NULL, MAC_TmacRmac));
}

/**************************************************************************
    Reads a block of data from a Standard Data File or a Backup Data File.
    If (s32_Offset + s32_Length > file length) you will get a LimitExceeded error.
    If the file permissions are not set to AR_FREE you must authenticate either
    with the key in e_ReadAccess or the key in e_ReadAndWriteAccess.
**************************************************************************/
bool PN532::ReadFileData(byte u8_FileID, int s32_Offset, int s32_Length, byte *u8_DataBuffer) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** ReadFileData(ID= %d, Offset= %d, Length= %d)\r\n", u8_FileID, s32_Offset, s32_Length);
    }

    // With intention this command does not use DF_INS_ADDITIONAL_FRAME because the CMAC must be calculated over all frames received.
    // When reading a lot of data this could lead to a buffer overflow in mi_CmacBuffer.
    while (s32_Length > 0) {
        int s32_Count = min(s32_Length,
                            48); // the maximum that can be transferred in one frame (must be a multiple of 16 if encryption is used)

        TX_BUFFER(i_Params, 7);
        i_Params.AppendUint8(u8_FileID);
        i_Params.AppendUint24(s32_Offset); // only the low 3 bytes are used
        i_Params.AppendUint24(s32_Count);  // only the low 3 bytes are used

        DESFireStatus e_Status;
        int s32_Read = DataExchangeReadFile(DF_INS_READ_DATA, &i_Params, u8_DataBuffer, s32_Count, &e_Status, MAC_TmacRmac)+2;

        if (e_Status != ST_Success || s32_Read <= 0)
            return false; // ST_MoreFrames is not allowed here!

        s32_Length -= s32_Read;
        s32_Offset += s32_Read;
        u8_DataBuffer += s32_Read;
    }
    return true;
}

/**************************************************************************
    Writes data to a Standard Data File or a Backup Data File.
    If the file permissins are not set to AR_FREE you must authenticate either
    with the key in e_WriteAccess or the key in e_ReadAndWriteAccess.
**************************************************************************/
bool PN532::WriteFileData(byte u8_FileID, int s32_Offset, int s32_Length, const byte *u8_DataBuffer) {
    if (mu8_DebugLevel > 0) {
        char s8_Buf[80];
        sprintf(s8_Buf, "\r\n*** WriteFileData(ID= %d, Offset= %d, Length= %d)\r\n", u8_FileID, s32_Offset, s32_Length);
        Utils::Print(s8_Buf);
    }

    // With intention this command does not use DF_INS_ADDITIONAL_FRAME because the CMAC must be calculated over all frames sent.
    // When writing a lot of data this could lead to a buffer overflow in mi_CmacBuffer.
    while (s32_Length > 0) {
        int s32_Count = min(s32_Length,
                            MAX_FRAME_SIZE - 8); // DF_INS_WRITE_DATA + u8_FileID + s32_Offset + s32_Count = 8 bytes

        TX_BUFFER(i_Params, MAX_FRAME_SIZE);
        i_Params.AppendUint8(u8_FileID);
        i_Params.AppendUint24(s32_Offset); // only the low 3 bytes are used
        i_Params.AppendUint24(s32_Count);  // only the low 3 bytes are used
        i_Params.AppendBuf(u8_DataBuffer, s32_Count);

        DESFireStatus e_Status;
        int s32_Read = DataExchange(DF_INS_WRITE_DATA, &i_Params, NULL, 0, &e_Status, MAC_TmacRmac);
        if (e_Status != ST_Success || s32_Read != 0)
            return false; // ST_MoreFrames is not allowed here!

        s32_Length -= s32_Count;
        s32_Offset += s32_Count;
        u8_DataBuffer += s32_Count;
    }
    return true;
}

/**************************************************************************
    Reads the value of a Value File
**************************************************************************/
bool PN532::ReadFileValue(byte u8_FileID, uint32_t *pu32_Value) {
    TX_BUFFER(i_Params, 1);
    i_Params.AppendUint8(u8_FileID);

    RX_BUFFER(i_RetData, 4);
    if (4 != DataExchange(DF_INS_GET_VALUE, &i_Params, i_RetData, 4, NULL, MAC_TmacRmac))
        return false;

    *pu32_Value = i_RetData.ReadUint32();
    return true;
}

// ########################################################################
// ####                      LOW LEVEL FUNCTIONS                      #####
// ########################################################################

// Safe self test
bool PN532::SAFE_TEST() {
    uint8_t u8_UID[] = {0, 0, 0, 0, 0, 0, 0};  // Buffer to store the returned UID
    uint8_t u8_Length;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    eCardType e_CardType;
    if (!readPassiveTargetID(PN532_MIFARE_ISO14443A, u8_UID, &u8_Length)) {
        Serial.println("readPassiveTargetID failed");
        return false;
    }

    // Get a list of all applications
    uint32_t u32_IDlist[28];
    byte u8_AppCount;
    if (!GetApplicationIDs(u32_IDlist, &u8_AppCount)) {
        Serial.println("GetApplicationIDs failed");
        return false;
    }

    PRINT_DEBUG("GetApplicationIDs ok");
    delay(1000);

    if (!SelectApplication(0xF1CC57)) {
        Serial.println("SelectApplication failed");
        return false;
    }

    PRINT_DEBUG("SelectApplication ok");
    delay(1000);

    const int FILE_LENGTH = 10; // this exceeds the frame size -> requires two frames for write / read
    DESFireFilePermissions k_Permis;
    k_Permis.e_ReadAccess = AR_KEY1; // u8_MasterKeyB
    k_Permis.e_WriteAccess = AR_KEY0; // u8_MasterKeyB
    k_Permis.e_ReadAndWriteAccess = AR_KEY0; // u8_SecondKeyB
    k_Permis.e_ChangeAccess = AR_KEY0; // u8_SecondKeyB

    // Get a list of all files in the application
    byte u8_FileIDs[32];
    byte u8_FileCount;
    if (!GetFileIDs(u8_FileIDs, &u8_FileCount)) {
        Serial.println("GetFileIDs failed");
        return false;
    }
    bool file_exists = false;
    for (int i = 0; i < u8_FileCount; i++) {
        if (u8_FileIDs[i] == 0x01) {
            file_exists = true;
            break;
        }
    }
    if (!file_exists) {
        Utils::Print("file 0x01 not found\r\n");
        return false;
    }

    PRINT_DEBUG("GetFileIDs ok");
    delay(1000);

    // Get the file settings
    DESFireFileSettings k_Settings;
    if (!GetFileSettings(0x01, &k_Settings)) {
        Serial.println("GetFileSettings failed");
        return false;
    }

    if (k_Settings.e_FileType != MDFT_STANDARD_DATA_FILE ||
        k_Settings.e_Encrypt != CM_PLAIN ||
        k_Settings.k_Permis.Pack() != k_Permis.Pack() ||
        k_Settings.u32_FileSize != FILE_LENGTH) {
        if (k_Settings.e_FileType != MDFT_STANDARD_DATA_FILE)
            Serial.println(String("e_FileType failed: ") + String(k_Settings.e_FileType, HEX));
        if (k_Settings.e_Encrypt != CM_ENCRYPT)
            Serial.println(String("e_Encrypt failed: ") + String(k_Settings.e_Encrypt, HEX));
        if (k_Settings.k_Permis.Pack() != k_Permis.Pack())
            Serial.println(String("Pack failed: ") + String(k_Settings.k_Permis.Pack(), HEX));
        if (k_Settings.u32_FileSize != FILE_LENGTH)
            Serial.println(String("u32_FileSize failed: ") + String(k_Settings.u32_FileSize));
        //Utils::Print("GetFileSettings() failed\r\n");
        //return false;
    }

    // Authenticate with the factory default PICC master key
    if (!Authenticate(0x01, &AES_DEFAULT_KEY)) {
        Serial.println("Authenticate failed");
        return false;
    }

    PRINT_DEBUG("Authenticate ok");
    delay(1000);

    byte u8_RxData[16];
    //bool PN532::ReadFileData(byte u8_FileID, int s32_Offset, int s32_Length, byte* u8_DataBuffer)
    if (!ReadFileData(1, 0, FILE_LENGTH, u8_RxData)) {
        Utils::Print("ReadFileData failed\r\n");
        return false;
    }

    PRINT_DEBUG("ReadFileData ok, contents: ")
    Utils::PrintHexBuf(u8_RxData, 4, LF);
    delay(1000);

    byte u8_Version;
    if (!PN532::GetKeyVersion(0, &u8_Version)) {
        Serial.println("GetKeyVersion failed");
        return false;
    }

    PRINT_DEBUG("GetKeyVersion ok");
    delay(1000);

    if (u8_Version != 0) {
        Utils::Print("The selftest requires an empty Desfire card (factory default DES key)\r\n");
        return false;
    }

    // Get the Desfire card version
    DESFireCardVersion k_Version;
    if (!GetCardVersion(&k_Version)) {
        Serial.println("GetCardVersion failed");
        return false;
    }

    PRINT_DEBUG("GetCardVersion ok");
    delay(1000);

    // Print the free memory on the card
    uint32_t u32_FreeMem;
    if (!GetFreeMemory(&u32_FreeMem)) {
        Serial.println("GetFreeMemory failed");
        return false;
    }

    PRINT_DEBUG("GetFreeMemory ok");
    delay(1000);

    // Get the key settings, key count and key type of the application
    DESFireKeySettings e_Settg;
    DESFireKeyType e_KeyType;
    byte u8_KeyCount;
    if (!GetKeySettings(&e_Settg, &u8_KeyCount, &e_KeyType))
        return false;

    if (e_Settg != KS_FACTORY_DEFAULT || u8_KeyCount != 4 || e_KeyType != DF_KEY_AES) {
        Utils::Print("GetKeySettings() failed\r\n");
        return false;
    }

    PRINT_DEBUG("GetKeySettings ok, restarting test in 10 secs");
    delay(10000);

    // ----------------------------------------------------------------------

//    const byte u8_KeyA[24] = { 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xB0, 0xA0, 0x90, 0x80, 0x70, 0x60, 0x50, 0x40, 0x30, 0x20, 0x10, 0x00 };
//    const byte u8_KeyB[24] = { 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8 };

//    if (!CreateStdDataFile(5, &k_Permis, FILE_LENGTH))
//        return false;

    return true;
}


// ########################################################################
// ####                          SELFTEST                             #####
// ########################################################################

// To execute this function set COMPILE_SELFTEST to 'true' in DoorOpenerSketch.ino
// This function tests all the other functions in this class.
// You need an empty Desfire card with the factory default PICC master key.
// The PICC master key will not be changed, but everyting else will be erased.
// If any error occurres the test is aborted and the function returns false.
bool PN532::Selftest() {
    // Activate the RF field and start communication with the card
    byte u8_Length; // 4 or 7
    byte u8_UID[8];
    eCardType e_CardType;
    if (!ReadPassiveTargetID(u8_UID, &u8_Length, &e_CardType))
        return false;

    if ((e_CardType & CARD_Desfire) == 0) {
        Utils::Print("The selftest requires a Desfire card.\r\n");
        return false;
    }

    // Switch to PICC level
    if (!SelectApplication(0x000000))
        return false;

    byte u8_Version;
    if (!PN532::GetKeyVersion(0, &u8_Version))
        return false;

    if (u8_Version != 0) {
        Utils::Print("The selftest requires an empty Desfire card (factory default DES key)\r\n");
        return false;
    }

    // Authenticate with the factory default PICC master key (always DES)
    if (!Authenticate(0, &DES2_DEFAULT_KEY))
        return false;

    // Get the Desfire card version
    DESFireCardVersion k_Version;
    if (!GetCardVersion(&k_Version))
        return false;

    // Delete all applications and all their files
    if (!FormatCard())
        return false;

    // Print the free memory on the card
    uint32_t u32_FreeMem;
    if (!GetFreeMemory(&u32_FreeMem))
        return false;

    // ----------------------------------------------------------------------

    // Create an application with two 2K3DES keys
    uint32_t u32_App2KDES = 0x00DE16;
    if (!CreateApplication(u32_App2KDES, KS_FACTORY_DEFAULT, 2, DF_KEY_2K3DES))
        return false;

    // Create an application with two 3K3DES keys
    uint32_t u32_App3KDES = 0x00DE24;
    if (!CreateApplication(u32_App3KDES, KS_FACTORY_DEFAULT, 2, DF_KEY_3K3DES))
        return false;

    // Create an application with two AES keys
    uint32_t u32_AppAES = 0x00AE16;
    if (!CreateApplication(u32_AppAES, KS_FACTORY_DEFAULT, 2, DF_KEY_AES))
        return false;

    // Create another application that will later be deleted
    uint32_t u32_AppDel = 0xAABBCC;
    if (!CreateApplication(u32_AppDel, KS_FACTORY_DEFAULT, 1, DF_KEY_2K3DES))
        return false;

    // Get a list of all applications
    uint32_t u32_IDlist[28];
    byte u8_AppCount;
    if (!GetApplicationIDs(u32_IDlist, &u8_AppCount))
        return false;

    if (u8_AppCount != 4 || u32_IDlist[0] != u32_App2KDES || u32_IDlist[1] != u32_App3KDES ||
        u32_IDlist[2] != u32_AppAES || u32_IDlist[3] != u32_AppDel) {
        Utils::Print("GetApplicationIDs() failed\r\n");
        return false;
    }

    // Delete the last application
    if (!DeleteApplication(u32_AppDel))
        return false;

    // Get the list of all applications again
    if (!GetApplicationIDs(u32_IDlist, &u8_AppCount))
        return false;

    if (u8_AppCount != 3) {
        Utils::Print("DeleteApplication() failed\r\n");
        return false;
    }

    // ----------------------------------------------------------------------

    // Select the 2K3DES application
    if (!SelectApplication(u32_App2KDES))
        return false;

    // Authenticate access to the new application with the default key
    if (!Authenticate(0, &DES2_DEFAULT_KEY))
        return false;

    // Get the key settings, key count and key type of the application
    DESFireKeySettings e_Settg;
    DESFireKeyType e_KeyType;
    byte u8_KeyCount;
    if (!GetKeySettings(&e_Settg, &u8_KeyCount, &e_KeyType))
        return false;

    if (e_Settg != KS_FACTORY_DEFAULT || u8_KeyCount != 2 || e_KeyType != DF_KEY_2K3DES) {
        Utils::Print("GetKeySettings() failed\r\n");
        return false;
    }

    // ----------------------------------------------------------------------

    const byte u8_KeyA[24] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xB0, 0xA0, 0x90,
                              0x80, 0x70, 0x60, 0x50, 0x40, 0x30, 0x20, 0x10, 0x00};
    const byte u8_KeyB[24] = {0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80,
                              0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8};

    DES i_Des2KeyA, i_Des2KeyB;
    DES i_Des3KeyA, i_Des3KeyB;
    AES i_AesKeyA, i_AesKeyB;

    if (!i_Des2KeyA.SetKeyData(u8_KeyA, 16, CARD_KEY_VERSION) ||
        !i_Des2KeyB.SetKeyData(u8_KeyB, 16, CARD_KEY_VERSION) ||
        !i_Des3KeyA.SetKeyData(u8_KeyA, 24, CARD_KEY_VERSION) ||
        !i_Des3KeyB.SetKeyData(u8_KeyB, 24, CARD_KEY_VERSION) ||
        !i_AesKeyA.SetKeyData(u8_KeyA, 16, CARD_KEY_VERSION) ||
        !i_AesKeyB.SetKeyData(u8_KeyB, 16, CARD_KEY_VERSION))
        return false;

    if (!SelftestKeyChange(u32_App2KDES, &DES2_DEFAULT_KEY, &i_Des2KeyA, &i_Des2KeyB) ||
        !SelftestKeyChange(u32_App3KDES, &DES3_DEFAULT_KEY, &i_Des3KeyA, &i_Des3KeyB) ||
        !SelftestKeyChange(u32_AppAES, &AES_DEFAULT_KEY, &i_AesKeyA, &i_AesKeyB))
        return false;

    Utils::Print("--------------------------------------------------------------\r\n");

    const int FILE_LENGTH = 80; // this exceeds the frame size -> requires two frames for write / read

    // Create Standard Data File no 5 with 80 bytes length
    DESFireFilePermissions k_Permis;
    k_Permis.e_ReadAccess = AR_KEY0; // u8_MasterKeyB
    k_Permis.e_WriteAccess = AR_KEY0; // u8_MasterKeyB
    k_Permis.e_ReadAndWriteAccess = AR_KEY1; // u8_SecondKeyB
    k_Permis.e_ChangeAccess = AR_KEY1; // u8_SecondKeyB
    if (!CreateStdDataFile(5, &k_Permis, FILE_LENGTH))
        return false;

    // Get a list of all files in the application
    byte u8_FileIDs[32];
    byte u8_FileCount;
    if (!GetFileIDs(u8_FileIDs, &u8_FileCount))
        return false;

    if (u8_FileCount != 1 || u8_FileIDs[0] != 5) {
        Utils::Print("GetFileIDs() failed\r\n");
        return false;
    }

    // Get the file settings
    DESFireFileSettings k_Settings;
    if (!GetFileSettings(5, &k_Settings))
        return false;

    if (k_Settings.e_FileType != MDFT_STANDARD_DATA_FILE ||
        k_Settings.e_Encrypt != CM_PLAIN ||
        k_Settings.k_Permis.Pack() != k_Permis.Pack() ||
        k_Settings.u32_FileSize != FILE_LENGTH) {
        Utils::Print("GetFileSettings() failed\r\n");
        return false;
    }

    // ----------------

    // Write 80 bytes to the file
    byte u8_TxData[FILE_LENGTH];
    for (int i = 0; i < FILE_LENGTH; i++) {
        u8_TxData[i] = i;
    }

    if (!WriteFileData(5, 0, FILE_LENGTH, u8_TxData))
        return false;

    // Read 80 bytes from the file
    byte u8_RxData[FILE_LENGTH];
    if (!ReadFileData(5, 0, FILE_LENGTH, u8_RxData))
        return false;

    if (memcmp(u8_TxData, u8_RxData, FILE_LENGTH) != 0) {
        Utils::Print("Read/Write file failed\r\n");
        return false;
    }

    // ----------------

    if (!DeleteFile(5))
        return false;

    if (!GetFileIDs(u8_FileIDs, &u8_FileCount))
        return false;

    if (u8_FileCount != 0) {
        Utils::Print("DeleteFile() failed\r\n");
        return false;
    }

    // ----------------

    // Switch to PICC level
    if (!SelectApplication(0x000000))
        return false;

    // Authenticate with the factory default PICC master key
    if (!Authenticate(0, &DES2_DEFAULT_KEY))
        return false;

    // Leave a clean card
    if (!FormatCard())
        return false;

    return true;
}

/**************************************************************************
    This function is private
    It checks the status byte that is returned by some commands.
    See chapter 7.1 in the manual.
    u8_Status = the status byte
**************************************************************************/
bool PN532::CheckPN532Status(byte u8_Status) {
    // Bits 0...5 contain the error code.
    u8_Status &= 0x3F;
    if (u8_Status == 0)
        return true;
    char s8_Buf[50];
    sprintf(s8_Buf, "PN532 Error 0x%02X: ", u8_Status);
    Utils::Print(s8_Buf);
    switch (u8_Status) {
        case 0x01:
            Utils::Print("Timeout\r\n");
            return false;
        case 0x02:
            Utils::Print("CRC error\r\n");
            return false;
        case 0x03:
            Utils::Print("Parity error\r\n");
            return false;
        case 0x04:
            Utils::Print("Wrong bit count during anti-collision\r\n");
            return false;
        case 0x05:
            Utils::Print("Framing error\r\n");
            return false;
        case 0x06:
            Utils::Print("Abnormal bit collision\r\n");
            return false;
        case 0x07:
            Utils::Print("Insufficient communication buffer\r\n");
            return false;
        case 0x09:
            Utils::Print("RF buffer overflow\r\n");
            return false;
        case 0x0A:
            Utils::Print("RF field has not been switched on\r\n");
            return false;
        case 0x0B:
            Utils::Print("RF protocol error\r\n");
            return false;
        case 0x0D:
            Utils::Print("Overheating\r\n");
            return false;
        case 0x0E:
            Utils::Print("Internal buffer overflow\r\n");
            return false;
        case 0x10:
            Utils::Print("Invalid parameter\r\n");
            return false;
        case 0x12:
            Utils::Print("Command not supported\r\n");
            return false;
        case 0x13:
            Utils::Print("Wrong data format\r\n");
            return false;
        case 0x14:
            Utils::Print("Authentication error\r\n");
            return false;
        case 0x23:
            Utils::Print("Wrong UID check byte\r\n");
            return false;
        case 0x25:
            Utils::Print("Invalid device state\r\n");
            return false;
        case 0x26:
            Utils::Print("Operation not allowed\r\n");
            return false;
        case 0x27:
            Utils::Print("Command not acceptable\r\n");
            return false;
        case 0x29:
            Utils::Print("Target has been released\r\n");
            return false;
        case 0x2A:
            Utils::Print("Card has been exchanged\r\n");
            return false;
        case 0x2B:
            Utils::Print("Card has disappeared\r\n");
            return false;
        case 0x2C:
            Utils::Print("NFCID3 initiator/target mismatch\r\n");
            return false;
        case 0x2D:
            Utils::Print("Over-current\r\n");
            return false;
        case 0x2E:
            Utils::Print("NAD msssing\r\n");
            return false;
        default:
            Utils::Print("Undocumented error\r\n");
            return false;
    }
}

// Changing the application key #0 is a completely different procedure from changing the key #1
// So both must be tested thoroughly.
// If there should be any bug in ChangeKey() the consequence may be a card that you cannot authenticate anymore!
bool PN532::SelftestKeyChange(uint32_t u32_Application, DESFireKey *pi_DefaultKey, DESFireKey *pi_NewKeyA,
                              DESFireKey *pi_NewKeyB) {
    Utils::Print("--------------------------------------------------------------\r\n");

    // Never change the PICC master key in the Selftest!
    if (u32_Application == 0x000000)
        return false;

    if (!SelectApplication(u32_Application))
        return false;

    // Authenticate access to the new application with the default key
    if (!Authenticate(0, pi_DefaultKey))
        return false;

    // As this command uses encryption it must be tested with all key types.
    byte u8_UID[7];
    if (!GetRealCardID(u8_UID))
        return false;

    // ---------- key #0 -> A ----------

    // Change the application key #0
    if (!ChangeKey(0, pi_NewKeyA, NULL))
        return false;

    // Authenticate with the new application master key
    if (!Authenticate(0, pi_NewKeyA))
        return false;

    // ---------- Key Settings ----------

    DESFireKeySettings e_Settg;
    DESFireKeyType e_KeyType;
    byte u8_KeyCount;

    // As this command uses encryption it must be tested with all key types.
    // Change key settings from 0x0F (KS_FACTORY_DEFAULT) -> 0x0D
    if (!ChangeKeySettings((DESFireKeySettings) 0x0D))
        return false;

    if (!GetKeySettings(&e_Settg, &u8_KeyCount, &e_KeyType))
        return false;

    if (e_Settg != 0x0D || u8_KeyCount != 2 || e_KeyType != pi_NewKeyA->GetKeyType()) {
        Utils::Print("ChangeKeySettings() failed\r\n");
        return false;
    }

    // Change key settings back to 0x0F (KS_FACTORY_DEFAULT)
    if (!ChangeKeySettings(KS_FACTORY_DEFAULT))
        return false;

    // ---------- key #0 -> B ----------

    // Change the application key #0 again
    if (!ChangeKey(0, pi_NewKeyB, NULL))
        return false;

    // Authenticate with the new application master key
    if (!Authenticate(0, pi_NewKeyB))
        return false;

    // ---------- key version ----------

    byte u8_KeyVersion;
    if (!PN532::GetKeyVersion(0, &u8_KeyVersion))
        return false;

    if (u8_KeyVersion != CARD_KEY_VERSION) {
        Utils::Print("GetKeyVersion() failed\r\n");
        return false;
    }

    // ---------- key #0 -> 0 ----------

    // Restore key #0 back to the default key
    if (!ChangeKey(0, pi_DefaultKey, NULL))
        return false;

    if (!Authenticate(0, pi_DefaultKey))
        return false;

    // ---------- key #1 ----------

    // Change the application key #1
    if (!ChangeKey(1, pi_NewKeyA, pi_DefaultKey))
        return false;

    // Change the application key #1 again
    if (!ChangeKey(1, pi_NewKeyB, pi_NewKeyA))
        return false;

    if (!Authenticate(1, pi_NewKeyB))
        return false;

    return true;
}
