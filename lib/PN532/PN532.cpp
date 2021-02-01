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
#include <string.h>

#define HAL(func)   (_interface->func)

PN532::PN532(PN532Interface &interface) : mi_CmacBuffer(mu8_CmacBuffer_Data, sizeof(mu8_CmacBuffer_Data))
{
    _interface = &interface;
    mu8_LastAuthKeyNo    = NOT_AUTHENTICATED;
    mu8_LastPN532Error   = 0;
    mu32_LastApplication = 0x000000; // No application selected

    // The PICC master key on an empty card is a simple DES key filled with 8 zeros
    const uint8_t ZERO_KEY[24] = {0};
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void PN532::begin()
{
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
void PN532::PrintHex(const uint8_t *data, const uint32_t numBytes)
{
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
void PN532::PrintHexChar(const uint8_t *data, const uint32_t numBytes)
{
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
uint32_t PN532::getFirmwareVersion(void)
{
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    if (HAL(writeCommand)(pn532_packetbuffer, 1)) {
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (0 > status) {
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
uint32_t PN532::readRegister(uint16_t reg)
{
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_READREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
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
uint32_t PN532::writeRegister(uint16_t reg, uint8_t val)
{
    uint32_t response;

    pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;
    pn532_packetbuffer[3] = val;


    if (HAL(writeCommand)(pn532_packetbuffer, 4)) {
        return 0;
    }

    // read data packet
    int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
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
bool PN532::writeGPIO(uint8_t pinstate)
{
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

    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
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
uint8_t PN532::readGPIO(void)
{
    pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

    // Send the READGPIO command (0x0C)
    if (HAL(writeCommand)(pn532_packetbuffer, 1))
        return 0x0;

    HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));

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
bool PN532::SAMConfig(void)
{
    pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode;
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x01; // use IRQ pin!

    DMSG("SAMConfig\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 4))
        return false;

    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    @brief  Turn the module into power mode  will wake up on I2C or SPI request 
*/
/**************************************************************************/
bool PN532::powerDownMode()
{
    pn532_packetbuffer[0] = PN532_COMMAND_POWERDOWN; 
    pn532_packetbuffer[1] = 0xC0; // I2C or SPI Wakeup
    pn532_packetbuffer[2] = 0x00; // no IRQ

    DMSG("POWERDOWN\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 4))
        return false;

    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool PN532::setPassiveActivationRetries(uint8_t maxRetries)
{
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
    pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
    pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
    pn532_packetbuffer[4] = maxRetries;

    if (HAL(writeCommand)(pn532_packetbuffer, 5))
        return 0x0;  // no ACK

    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
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

bool PN532::setRFField(uint8_t autoRFCA, uint8_t rFOnOff)
{
    if (rFOnOff == 0) { // if RF field is switched off
        mu8_LastAuthKeyNo    = NOT_AUTHENTICATED;
        mu32_LastApplication = 0x000000; // No application selected
    }

    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0x00 | autoRFCA | rFOnOff;

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return 0x0;  // command failed
    }

    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
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
bool PN532::readPassiveTargetID(uint8_t cardbaudrate, uint8_t *uid, uint8_t *uidLength, uint16_t timeout, bool inlist)
{
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    pn532_packetbuffer[2] = cardbaudrate;

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return false;  // command failed
    }

    // read data packet
    if (HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), timeout) < 0) {
        return false;
    }

    // check some basic stuff
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
    if (pn532_packetbuffer[0] != 1) {
        return false;
    }

    int uidLen = pn532_packetbuffer[5];
    *uidLength = uidLen;
    memcpy(uid, uid+6, uidLen);

    // ATQA and SAK codes can be found here: https://www.nxp.com/docs/en/application-note/AN10833.pdf
    uint16_t ATQA = ((uint16_t)pn532_packetbuffer[2] << 8) | pn532_packetbuffer[3];
    byte     SAK   = pn532_packetbuffer[4];

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
    delay(2000);
#endif

    if (inlist) {
        inListedTag = pn532_packetbuffer[1];
    }

    return 1;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool PN532::mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
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
bool PN532::mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
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
uint8_t PN532::mifareclassic_AuthenticateBlock (uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData)
{
    uint8_t i;

    // Hang on to the key and uid data
    memcpy (_key, keyData, 6);
    memcpy (_uid, uid, uidLen);
    _uidLen = uidLen;

    // Prepare the authentication command //
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
    pn532_packetbuffer[1] = 1;                              /* Max card numbers */
    pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
    pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
    memcpy (pn532_packetbuffer + 4, _key, 6);
    for (i = 0; i < _uidLen; i++) {
        pn532_packetbuffer[10 + i] = _uid[i];              /* 4 bytes card ID */
    }

    if (HAL(writeCommand)(pn532_packetbuffer, 10 + _uidLen))
        return 0;

    // Read the response packet
    HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));

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
uint8_t PN532::mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t *data)
{
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
    HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* If byte 8 isn't 0x00 we probably have an error */
    if (pn532_packetbuffer[0] != 0x00) {
        return 0;
    }

    /* Copy the 16 data bytes to the output buffer        */
    /* Block content starts at byte 9 of a valid response */
    memcpy (data, pn532_packetbuffer + 1, 16);

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
uint8_t PN532::mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t *data)
{
    /* Prepare the first command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                      /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
    pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
    memcpy (pn532_packetbuffer + 4, data, 16);        /* Data Payload */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 20)) {
        return 0;
    }

    /* Read the response packet */
    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
}

/**************************************************************************/
/*!
    Formats a Mifare Classic card to store NDEF Records

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t PN532::mifareclassic_FormatNDEF (void)
{
    uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
    // for the MAD sector in NDEF records (sector 0)

    // Write block 1 and 2 to the card
    if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
        return 0;
    // Write key A and access rights card
    if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
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
uint8_t PN532::mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char *url)
{
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
    uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len + 5, 0xD1, 0x01, len + 1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (len <= 6) {
        // Unlikely we'll get a url this short, but why not ...
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer1[len + 9] = 0xFE;
    } else if (len == 7) {
        // 0xFE needs to be wrapped around to next block
        memcpy (sectorbuffer1 + 9, url, len);
        sectorbuffer2[0] = 0xFE;
    } else if ((len > 7) && (len <= 22)) {
        // Url fits in two blocks
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer2[len - 7] = 0xFE;
    } else if (len == 23) {
        // 0xFE needs to be wrapped around to final block
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, len - 7);
        sectorbuffer3[0] = 0xFE;
    } else {
        // Url fits in three blocks
        memcpy (sectorbuffer1 + 9, url, 7);
        memcpy (sectorbuffer2, url + 7, 16);
        memcpy (sectorbuffer3, url + 23, len - 23);
        sectorbuffer3[len - 23] = 0xFE;
    }

    // Now write all three blocks back to the card
    if (!(mifareclassic_WriteDataBlock (sectorNumber * 4, sectorbuffer1)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 1, sectorbuffer2)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 2, sectorbuffer3)))
        return 0;
    if (!(mifareclassic_WriteDataBlock ((sectorNumber * 4) + 3, sectorbuffer4)))
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
uint8_t PN532::mifareultralight_ReadPage (uint8_t page, uint8_t *buffer)
{
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
    HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));

    /* If byte 8 isn't 0x00 we probably have an error */
    if (pn532_packetbuffer[0] == 0x00) {
        /* Copy the 4 data bytes to the output buffer         */
        /* Block content starts at byte 9 of a valid response */
        /* Note that the command actually reads 16 bytes or 4  */
        /* pages at a time ... we simply discard the last 12  */
        /* bytes                                              */
        memcpy (buffer, pn532_packetbuffer + 1, 4);
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
uint8_t PN532::mifareultralight_WritePage (uint8_t page, uint8_t *buffer)
{
    /* Prepare the first command */
    pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = 1;                           /* Card number */
    pn532_packetbuffer[2] = MIFARE_CMD_WRITE_ULTRALIGHT; /* Mifare UL Write cmd = 0xA2 */
    pn532_packetbuffer[3] = page;                        /* page Number (0..63) */
    memcpy (pn532_packetbuffer + 4, buffer, 4);          /* Data Payload */

    /* Send the command */
    if (HAL(writeCommand)(pn532_packetbuffer, 8)) {
        return 0;
    }

    /* Read the response packet */
    return (0 < HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer)));
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
bool PN532::inDataExchange(uint8_t *send, uint8_t sendLength, uint8_t *response, uint8_t *responseLength)
{
    uint8_t i;

    pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[1] = inListedTag;

    if (HAL(writeCommand)(pn532_packetbuffer, 2, send, sendLength)) {
        return false;
    }

    int16_t status = HAL(readResponse)(response, *responseLength, 1000);
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
bool PN532::inListPassiveTarget()
{
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0;

    DMSG("inList passive target\n");

    if (HAL(writeCommand)(pn532_packetbuffer, 3)) {
        return false;
    }

    int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), 30000);
    if (status < 0) {
        return false;
    }

    if (pn532_packetbuffer[0] != 1) {
        return false;
    }

    inListedTag = pn532_packetbuffer[1];

    return true;
}

int8_t PN532::tgInitAsTarget(const uint8_t* command, const uint8_t len, const uint16_t timeout){
  
  int8_t status = HAL(writeCommand)(command, len);
    if (status < 0) {
        return -1;
    }

    status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), timeout);
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
int8_t PN532::tgInitAsTarget(uint16_t timeout)
{
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

        0x0a, 0x46,  0x66, 0x6D, 0x01, 0x01, 0x10, 0x02, 0x02, 0x00, 0x80, // LLCP magic number, version parameter and MIUX
        0x00
    };
    return tgInitAsTarget(command, sizeof(command), timeout);
}

int16_t PN532::tgGetData(uint8_t *buf, uint8_t len)
{
    buf[0] = PN532_COMMAND_TGGETDATA;

    if (HAL(writeCommand)(buf, 1)) {
        return -1;
    }

    int16_t status = HAL(readResponse)(buf, len, 3000);
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

bool PN532::tgSetData(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
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
        for (int8_t i = hlen - 1; i >= 0; i--){
            pn532_packetbuffer[i + 1] = header[i];
        }
        pn532_packetbuffer[0] = PN532_COMMAND_TGSETDATA;

        if (HAL(writeCommand)(pn532_packetbuffer, hlen + 1, body, blen)) {
            return false;
        }
    }

    if (0 > HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), 3000)) {
        return false;
    }

    if (0 != pn532_packetbuffer[0]) {
        return false;
    }

    return true;
}

int16_t PN532::inSelectCard(const uint8_t relevantTarget){

    pn532_packetbuffer[0] = PN532_COMMAND_INSELECT;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

int16_t PN532::inDeselectCard(const uint8_t relevantTarget){

    pn532_packetbuffer[0] = PN532_COMMAND_INDESELECT;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

int16_t PN532::inRelease(const uint8_t relevantTarget){

    pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
    pn532_packetbuffer[1] = relevantTarget;

    if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
        return 0;
    }

    // read data packet
    return HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}


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
int8_t PN532::felica_Polling(uint16_t systemCode, uint8_t requestCode, uint8_t * idm, uint8_t * pmm, uint16_t *systemCodeResponse, uint16_t timeout)
{
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

  int16_t status = HAL(readResponse)(pn532_packetbuffer, 22, timeout);
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
  for (i=0; i<8; ++i) {
    idm[i] = pn532_packetbuffer[4+i];
    _felicaIDm[i] = pn532_packetbuffer[4+i];
    pmm[i] = pn532_packetbuffer[12+i];
    _felicaPMm[i] = pn532_packetbuffer[12+i];
  }

  if ( responseLength == 20 ) {
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
int8_t PN532::felica_SendCommand (const uint8_t *command, uint8_t commandlength, uint8_t *response, uint8_t *responseLength)
{
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
  int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), 200);
  if (status < 0) {
    DMSG("Could not receive response\n");
    return -3;
  }

  // Check status (pn532_packetbuffer[0])
  if ((pn532_packetbuffer[0] & 0x3F)!=0) {
    DMSG("Status code indicates an error: ");
    DMSG_HEX(pn532_packetbuffer[0]);
    DMSG("\n");
    return -4;
  }

  // length check
  *responseLength = pn532_packetbuffer[1] - 1;
  if ( (status - 2) != *responseLength) {
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
int8_t PN532::felica_RequestService(uint8_t numNode, uint16_t *nodeCodeList, uint16_t *keyVersions)
{
  if (numNode > FELICA_REQ_SERVICE_MAX_NODE_NUM) {
    DMSG("numNode is too large\n");
    return -1;
  }

  uint8_t i, j=0;
  uint8_t cmdLen = 1 + 8 + 1 + 2*numNode;
  uint8_t cmd[cmdLen];
  cmd[j++] = FELICA_CMD_REQUEST_SERVICE;
  for (i=0; i<8; ++i) {
    cmd[j++] = _felicaIDm[i];
  }
  cmd[j++] = numNode;
  for (i=0; i<numNode; ++i) {
    cmd[j++] = nodeCodeList[i] & 0xFF;
    cmd[j++] = (nodeCodeList[i] >> 8) & 0xff;
  }

  uint8_t response[10+2*numNode];
  uint8_t responseLength;

  if (felica_SendCommand(cmd, cmdLen, response, &responseLength) != 1) {
    DMSG("Request Service command failed\n");
    return -2;
  }

  // length check
  if ( responseLength != 10+2*numNode ) {
    DMSG("Request Service command failed (wrong response length)\n");
    return -3;
  }

  for(i=0; i<numNode; i++) {
    keyVersions[i] = (uint16_t)(response[10+i*2] + (response[10+i*2+1] << 8));
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
int8_t PN532::felica_RequestResponse(uint8_t * mode)
{
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
  if ( responseLength != 10) {
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
int8_t PN532::felica_ReadWithoutEncryption (uint8_t numService, const uint16_t *serviceCodeList, uint8_t numBlock, const uint16_t *blockList, uint8_t blockData[][16])
{
  if (numService > FELICA_READ_MAX_SERVICE_NUM) {
    DMSG("numService is too large\n");
    return -1;
  }
  if (numBlock > FELICA_READ_MAX_BLOCK_NUM) {
    DMSG("numBlock is too large\n");
    return -2;
  }

  uint8_t i, j=0, k;
  uint8_t cmdLen = 1 + 8 + 1 + 2*numService + 1 + 2*numBlock;
  uint8_t cmd[cmdLen];
  cmd[j++] = FELICA_CMD_READ_WITHOUT_ENCRYPTION;
  for (i=0; i<8; ++i) {
    cmd[j++] = _felicaIDm[i];
  }
  cmd[j++] = numService;
  for (i=0; i<numService; ++i) {
    cmd[j++] = serviceCodeList[i] & 0xFF;
    cmd[j++] = (serviceCodeList[i] >> 8) & 0xff;
  }
  cmd[j++] = numBlock;
  for (i=0; i<numBlock; ++i) {
    cmd[j++] = (blockList[i] >> 8) & 0xFF;
    cmd[j++] = blockList[i] & 0xff;
  }

  uint8_t response[12+16*numBlock];
  uint8_t responseLength;
  if (felica_SendCommand(cmd, cmdLen, response, &responseLength) != 1) {
    DMSG("Read Without Encryption command failed\n");
    return -3;
  }

  // length check
  if ( responseLength != 12+16*numBlock ) {
    DMSG("Read Without Encryption command failed (wrong response length)\n");
    return -4;
  }

  // status flag check
  if ( response[9] != 0 || response[10] != 0 ) {
    DMSG("Read Without Encryption command failed (Status Flag: ");
    DMSG_HEX(pn532_packetbuffer[9]);
    DMSG_HEX(pn532_packetbuffer[10]);
    DMSG(")\n");
    return -5;
  }

  k = 12;
  for(i=0; i<numBlock; i++ ) {
    for(j=0; j<16; j++ ) {
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
int8_t PN532::felica_WriteWithoutEncryption (uint8_t numService, const uint16_t *serviceCodeList, uint8_t numBlock, const uint16_t *blockList, uint8_t blockData[][16])
{
  if (numService > FELICA_WRITE_MAX_SERVICE_NUM) {
    DMSG("numService is too large\n");
    return -1;
  }
  if (numBlock > FELICA_WRITE_MAX_BLOCK_NUM) {
    DMSG("numBlock is too large\n");
    return -2;
  }

  uint8_t i, j=0, k;
  uint8_t cmdLen = 1 + 8 + 1 + 2*numService + 1 + 2*numBlock + 16 * numBlock;
  uint8_t cmd[cmdLen];
  cmd[j++] = FELICA_CMD_WRITE_WITHOUT_ENCRYPTION;
  for (i=0; i<8; ++i) {
    cmd[j++] = _felicaIDm[i];
  }
  cmd[j++] = numService;
  for (i=0; i<numService; ++i) {
    cmd[j++] = serviceCodeList[i] & 0xFF;
    cmd[j++] = (serviceCodeList[i] >> 8) & 0xff;
  }
  cmd[j++] = numBlock;
  for (i=0; i<numBlock; ++i) {
    cmd[j++] = (blockList[i] >> 8) & 0xFF;
    cmd[j++] = blockList[i] & 0xff;
  }
  for (i=0; i<numBlock; ++i) {
    for(k=0; k<16; k++) {
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
  if ( responseLength != 11 ) {
    DMSG("Write Without Encryption command failed (wrong response length)\n");
    return -4;
  }

  // status flag check
  if ( response[9] != 0 || response[10] != 0 ) {
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
int8_t PN532::felica_RequestSystemCode(uint8_t * numSystemCode, uint16_t *systemCodeList)
{
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
  if ( responseLength < 10 + 2 * *numSystemCode ) {
    DMSG("Request System Code command failed (wrong response length)\n");
    return -2;
  }

  uint8_t i;
  for(i=0; i<*numSystemCode; i++) {
    systemCodeList[i] = (uint16_t)((response[10+i*2]<< 8) + response[10+i*2+1]);
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
int8_t PN532::felica_Release()
{
  // InRelease
  pn532_packetbuffer[0] = PN532_COMMAND_INRELEASE;
  pn532_packetbuffer[1] = 0x00;   // All target
  DMSG("Release all FeliCa target\n");

  if (HAL(writeCommand)(pn532_packetbuffer, 2)) {
    DMSG("No ACK\n");
    return -1;  // no ACK
  }

  // Wait card response
  int16_t frameLength = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer), 1000);
  if (frameLength < 0) {
    DMSG("Could not receive response\n");
    return -2;
  }

  // Check status (pn532_packetbuffer[0])
  if ((pn532_packetbuffer[0] & 0x3F)!=0) {
    DMSG("Status code indicates an error: ");
    DMSG_HEX(pn532_packetbuffer[7]);
    DMSG("\n");
    return -3;
  }

  return 1;
}

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
bool PN532::EnableRandomIDForever()
{
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
bool PN532::GetRealCardID(byte u8_UID[7])
{
    //if (mu8_DebugLevel > 0) Utils::Print("\r\n*** GetRealCardID()\r\n");

    if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED)
    {
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
    uint32_t u32_Crc2 = CalcCrc32(u8_UID, 7, &u8_Status, 1);

#ifdef DEBUG
        //Utils::Print("* CRC:       0x");
        //Utils::PrintHex32(u32_Crc2, LF);
#endif

    if (u32_Crc1 != u32_Crc2)
    {
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
byte PN532::GetLastPN532Error()
{
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
int PN532::DataExchange(byte u8_Command, TxBuffer* pi_Params, byte* u8_RecvBuf, int s32_RecvSize, DESFireStatus* pe_Status, DESFireCmac e_Mac)
{
    TX_BUFFER(i_Command, 1);
    i_Command.AppendUint8(u8_Command);

    return DataExchange(&i_Command, pi_Params, u8_RecvBuf, s32_RecvSize, pe_Status, e_Mac);
}
int PN532::DataExchange(TxBuffer* pi_Command,               // in (command + params that are not encrypted)
                          TxBuffer* pi_Params,                // in (parameters that may be encrypted)
                          byte* u8_RecvBuf, int s32_RecvSize, // out
                          DESFireStatus* pe_Status,
                          DESFireCmac    e_Mac)               // in
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
    if (2 + pi_Command->GetCount() + pi_Params->GetCount() > PN532_PACKBUFFSIZE || s32_Overhead + s32_RecvSize > PN532_PACKBUFFSIZE)
    {
        //Utils::Print("DataExchange(): Invalid parameters\r\n");
        return -1;
    }

    if (e_Mac & (MAC_Tcrypt | MAC_Rcrypt))
    {
        if (mu8_LastAuthKeyNo == NOT_AUTHENTICATED)
        {
            //Utils::Print("Not authenticated\r\n");
            return -1;
        }
    }

    byte u8_Command = pi_Command->GetData()[0];

    int P=0;
    pn532_packetbuffer[P++] = PN532_COMMAND_INDATAEXCHANGE;
    pn532_packetbuffer[P++] = 1; // Card number (Logical target number)

    memcpy(pn532_packetbuffer + P, pi_Command->GetData(), pi_Command->GetCount());
    P += pi_Command->GetCount();

    memcpy(pn532_packetbuffer + P, pi_Params->GetData(),  pi_Params->GetCount());
    P += pi_Params->GetCount();

    // ORIGINALLY:
    // if (!SendCommandCheckAck(pn532_packetbuffer, P))
    //    return -1;

    // MODIFIED:
    if (HAL(writeCommand)(pn532_packetbuffer, P)) {
        return -1;
    }

    // read data packet
    int16_t status = HAL(readResponse)(pn532_packetbuffer, sizeof(pn532_packetbuffer));
    if (0 > status) {
        return 0;
    }

    // ReadData() returns 3 byte if status error from the PN532
    // ReadData() returns 4 byte if status error from the Desfire card
    if (status < 3 || pn532_packetbuffer[1] != PN532_COMMAND_INDATAEXCHANGE + 1)
    {
        //Utils::Print("DataExchange() failed\r\n");
        return -1;
    }

    // Here we get two status bytes that must be checked
    uint8_t u8_PN532Status = pn532_packetbuffer[2]; // contains errors from the PN532
    uint8_t u8_CardStatus  = pn532_packetbuffer[3]; // contains errors from the Desfire card

    mu8_LastPN532Error = u8_PN532Status;

    if (u8_PN532Status != ST_Success || status < 4)
        return -1;

    // After any error that the card has returned the authentication is invalidated.
    // The card does not send any CMAC anymore until authenticated anew.
    if (u8_CardStatus != ST_Success && u8_CardStatus != ST_MoreFrames)
    {
        mu8_LastAuthKeyNo = NOT_AUTHENTICATED; // A new authentication is required now
    }

    if (!CheckCardStatus((DESFireStatus)u8_CardStatus))
        return -1;
    if (pe_Status)
        *pe_Status = (DESFireStatus)u8_CardStatus;

    status -= 4; // 3 bytes for INDATAEXCHANGE response + 1 byte card status

    // A CMAC may be appended to the end of the frame.
    // The CMAC calculation is important because it maintains the IV of the session key up to date.
    // If the IV is out of sync with the IV in the card, the next encryption with the session key will result in an Integrity Error.
    if ((e_Mac & MAC_Rmac) &&                                              // Calculate RX CMAC only if the caller requests it
        (u8_CardStatus == ST_Success || u8_CardStatus == ST_MoreFrames) && // In case of an error there is no CMAC in the response
        (mu8_LastAuthKeyNo != NOT_AUTHENTICATED))                          // No session key -> no CMAC calculation possible
    {
        // For example GetCardVersion() calls DataExchange() 3 times:
        // 1. u8_Command = DF_INS_GET_VERSION      -> clear CMAC buffer + append received data
        // 2. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        // 3. u8_Command = DF_INS_ADDITIONAL_FRAME -> append received data
        if (u8_Command != DF_INS_ADDITIONAL_FRAME)
        {
            mi_CmacBuffer.Clear();
        }

        // This is an intermediate frame. More frames will follow. There is no CMAC in the response yet.
        if (u8_CardStatus == ST_MoreFrames)
        {
            if (!mi_CmacBuffer.AppendBuf(pn532_packetbuffer + 4, status))
                return -1;
        }

        if ((status >= 8) &&             // If the response is shorter than 8 bytes it surely does not contain a CMAC
            (u8_CardStatus == ST_Success)) // Response contains CMAC only in case of success
        {
            status -= 8; // Do not return the received CMAC to the caller and do not include it into the CMAC calculation

            byte* u8_RxMac = pn532_packetbuffer + 4 + status;

        }
    }

    if (status > s32_RecvSize)
    {
        //Utils::Print("DataExchange() Buffer overflow\r\n");
        return -1;
    }

    if (u8_RecvBuf && status)
    {
        memcpy(u8_RecvBuf, pn532_packetbuffer + 4, status);

        if (e_Mac & MAC_Rcrypt) // decrypt received data with session key
        {

#ifdef DEBUG
                //Utils::Print("Decrypt:  ");
                //Utils::PrintHexBuf(u8_RecvBuf, status, LF);
#endif
        }
    }
    return status;
}

// Checks the status byte that is returned from the card
bool PN532::CheckCardStatus(DESFireStatus e_Status)
{
/*
    char s8_Buf[24];
    sprintf(s8_Buf, "Desfire status 0x%02X: ", e_Status);
    Serial.println(s8_Buf);
  */
    switch (e_Status)
    {
        case ST_Success:    // Success
        case ST_NoChanges:  // No changes made
        case ST_MoreFrames: // Another frame will follow
            return true;

        default: return false; // This is just to avoid stupid gcc compiler warnings
    }
}

// Generate multi byte random
void GenerateRandom(byte* u8_Random, int s32_Length)
{
    uint32_t u32_Now = millis();
    for (int i=0; i<s32_Length; i++)
    {
        u8_Random[i] = (byte)u32_Now;
        u32_Now *= 127773;
        u32_Now += 16807;
    }
}

// ITU-V.41 (ISO 14443A)
// This CRC is used only for legacy authentication. (not implemented anymore)
uint16_t CalcCrc16(const byte* u8_Data, int s32_Length)
{
    uint16_t u16_Crc = 0x6363;
    for (int i=0; i<s32_Length; i++)
    {
        uint8_t ch = u8_Data[i];
        ch = ch ^ (byte)u16_Crc;
        ch = ch ^ (ch << 4);
        u16_Crc = (u16_Crc >> 8) ^ ((uint16_t)ch << 8) ^ ((uint16_t)ch << 3) ^ ((uint16_t)ch >> 4);
    }
    return u16_Crc;
}

// This CRC is used for ISO and AES authentication.
// The new Desfire EV1 authentication calculates the CRC32 also over the command, but the command is not encrypted later.
// This function allows to include the command into the calculation without the need to add the command to the same buffer that is later encrypted.
uint32_t CalcCrc32(const byte* u8_Data1, int s32_Length1, // data to process
                          const byte* u8_Data2, int s32_Length2) // optional additional data to process (these parameters may be omitted)
{
    uint32_t u32_Crc = 0xFFFFFFFF;
    u32_Crc = CalcCrc32(u8_Data1, s32_Length1, u32_Crc);
    u32_Crc = CalcCrc32(u8_Data2, s32_Length2, u32_Crc);
    return u32_Crc;
}

// private
uint32_t CalcCrc32(const byte* u8_Data, int s32_Length, uint32_t u32_Crc)
{
    for (int i=0; i<s32_Length; i++)
    {
        u32_Crc ^= u8_Data[i];
        for (int b=0; b<8; b++)
        {
            bool b_Bit = (u32_Crc & 0x01) > 0;
            u32_Crc >>= 1;
            if (b_Bit) u32_Crc ^= 0xEDB88320;
        }
    }
    return u32_Crc;
}

// We need a special time counter that does not roll over after 49 days (as millis() does)
uint64_t GetMillis64()
{
    static uint32_t u32_High = 0;
    static uint32_t u32_Last = 0;

    uint32_t u32_Now = millis(); // starts at zero after CPU reset

    // Check for roll-over
    if (u32_Now < u32_Last) u32_High ++;
    u32_Last = u32_Now;

    uint64_t u64_Time = u32_High;
    u64_Time <<= 32;
    u64_Time |= u32_Now;
    return u64_Time;
}
