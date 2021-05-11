// Test file for BlueBox, modify pins etc for V4 board testing

#include "Utils.h"

#define BOARD_V4 1
#define BOARD_BLUEBOX 2
#define BOARD BOARD_BLUEBOX

#if PROTOCOL == PROT_HSU
    #include "PN532_HSU.h"
    #include "PN532.h"

#if BOARD == BOARD_V4
    PN532_HSU pn532(13, 14);
    int piezo_pin = 12;
#else
    PN532_HSU pn532(13, 32);
    int piezo_pin = 33;
#endif


PN532 nfc(pn532);
#define USING "HSU"

#elif PROTOCOL == PROT_I2C
    #include <Wire.h>
    #include <PN532_I2C/PN532_I2C.h>
    #include <PN532.h>

    #define USING "I2C"

    PN532_I2C pn532i2c(Wire);
    PN532 nfc(pn532i2c);
#endif

AES mi_AesSessionKey;

int freq = 2000;
int channel = 0;
int resolution = 8;

void setup(void) {
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(piezo_pin, channel);

    Serial.begin(115200);
  while (!Serial) {
      Serial.print(".");
      delay(500);
  }

  delay(1000);
  Serial.println(String("Using ") + String(USING)) ;

  nfc.AES_DEFAULT_KEY.SetKeyData(NEW_KEY, 16, 0);
  nfc.DES2_DEFAULT_KEY.SetKeyData(NEW_KEY, 8, 0); // simple DES

#if BOARD == BOARD_V4
    digitalWrite(33, HIGH);
    delay(100);
    digitalWrite(33, LOW);
#endif

  nfc.begin();
  uint32_t versiondata = 0;
  while (true) {
      versiondata = nfc.getFirmwareVersion();
      if (versiondata) break;
      Serial.println("Didn't find PN53x board");
      delay(10000);
  }

  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  nfc.SAMConfig();
  Serial.println("Waiting for an ISO14443A Card ...");
}

void loop(void) {
  uint8_t success = 0;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight, DesFire)
  nfc.SAFE_TEST();
//  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");

    nfc.SAFE_TEST();

    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ...
      Serial.println("Seems to be a Mifare Classic card (4 byte UID)");

      // Now we need to try to authenticate it for read/write access
      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
      Serial.println("Trying to authenticate block 4 with default KEYA value");
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	  // Start with block 4 (the first block of sector 1) since sector 0
	  // contains the manufacturer data and it's probably better just
	  // to leave it alone unless you know what you're doing
      success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);

      if (success)
      {
        Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
        uint8_t data[16];

        // If you want to write something to block 4 to test with, uncomment
		// the following line and this text should be read back in a minute
        // data = { 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0};
        // success = nfc.mifareclassic_WriteDataBlock (4, data);

        // Try to read the contents of block 4
        success = nfc.mifareclassic_ReadDataBlock(4, data);

        if (success)
        {
          // Data seems to have been read ... spit it out
          Serial.println("Reading Block 4:");
          nfc.PrintHexChar(data, 16);
          Serial.println("");

          // Wait a bit before reading the card again
          delay(1000);
        }
        else
        {
          Serial.println("Ooops ... unable to read the requested block.  Try another key?");
        }
      }
      else
      {
        Serial.println("Ooops ... authentication failed: Try another key?");
      }
    }

    if (uidLength == 7)
    {
      // We probably have a Mifare Ultralight card ...
      Serial.println("Seems to be a Mifare Ultralight tag (7 byte UID)");

      // Try to read the first general-purpose user page (#4)
      Serial.println("Reading page 4");
      uint8_t data[32];
      success = nfc.mifareultralight_ReadPage (4, data);
      if (success)
      {
        // Data seems to have been read ... spit it out
        nfc.PrintHexChar(data, 4);
        Serial.println("");
		
        // Wait a bit before reading the card again
        delay(1000);
      }
      else
      {
        Serial.println("Ooops ... unable to read the requested page!?");
      }
    }
  }
    delay(1);
}

