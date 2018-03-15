// --------------------------------------
// Modified i2c_scanner to find, dump and update a SOR/T SOR/R  Dosimeter 24C16 EEPROM (and probably related devices).
// to hack in missing features.
//
// See http://stm32duino.com/viewtopic.php?f=45&t=3322 
// for more about hacking these devices
// Latest code should be checked out with git sith something on the lones of... 
//  git clone https://github.com/pingumacpenguin/SOR-T-SOR-R-Dosimeter-24C16-EEROM-dumper-and-updater..git
//
// The i2c scanner code came with the following comments.
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address will not be seen properly.
//
// Additional i2c hints from http://playground.arduino.cc/Code/I2CEEPROM
//

// This code was written for and teseted on and STM32F103C8T6 - the so called BluePill -  see http://wiki.stm32duino.com/index.php?title=Blue_Pill
// but should work pretty much unmodified on most 'duinos.

// Thanks to the many coding giants, who's shoulders this sketch stands upon.
//

// The hexdump of the 24c16 will appear as a set of C++  byte Arrayson the STM32 USB serial device, or UART1, depending on the settings in the IDE.
// See http://stm32duino.com for details of how the serial ports on the STM32F103XX devices are defined.

// For connection to the EEPROM, you should be able to use a SOC8 clamp to connect without the inconvenience of soldering to the board.
// WARNING: Do not leave the battery in the dosimeter and accidentally power it from the STLINK V2 or the USB port at the same time, the battery will pop. 
// Search Ebay for "SOIC8 SOP8 Flash Chip IC Test Clips Socket Adpter BIOS" or similar.

#include <Wire.h>


#define BLINK_PIN PC13   // Unused.. Can be used to assist with debugging. 

#define BOARD_POWER PB9  // This can theoretically be used to power the 24c16 since the board draws a few miliamps. 
// In the end, I just opted to power the board from 3v3 directly, as this makes life easier.

#define BOARD_WP PB8     // Pull down WP pin to allow us to write to the 24c16 - by default this is held high by a pullup resistor on the multimeter. 



void setup()
{



  // Allow the USB to re-enumerate before we attempt to talk. You can probably shorten this delay considerably.
  delay(30000);

  // Using the BluePill, or other STM32F103XX boards, PB6 is SCL => Pin 6 on the 24C02
  //                                                  PB7 is SDA => Pin 5 on the 24C02

  //                                                  PB8 is GPOI used to pull WP (Write Protect) (Pin7) low on the 24C02 to allow writing to the chip.
  //                                                  There is a pullup on WP on the Victor VC921 and probably most other devices, so we need to pull low to write.

  //pinMode(PB6,OUTPUT_OPEN_DRAIN);
  //pinMode(PB7,INPUT);

  // Power up the multimeter if connected to the BOARD_POWER
  //pinMode(BOARD_POWER, OUTPUT);
  //digitalWrite(BOARD_POWER, HIGH);

  //Enable writing to the eerom
  //pinMode(BOARD_WP, OUTPUT);
  //digitalWrite(BOARD_WP, LOW);

  Serial.println("// SOR/T SOR/R Dosimeter 24C16 EEROM dumper and updater.");
  //TwoWire Wire(PB6, PB7, SOFT_STANDARD);
  Wire.begin();
  Serial.println("// Waiting for Device to POST and  i2c Bus to settle.");
  delay(3000);
}



void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("//Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("\n// I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" ");

      // Dump the current rom values, so we have somthing to fall back on if it all goes pear shaped.
      // NOTE: The 24c16 also contains calibration values, so should be unique to the dosimeter, simply pasting the hexdump from
      //       one dosimeter to another will almost certainly screw up the calibration.

      dump24c02(address);

      delay(1000);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("//Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("//No I2C devices found\n");
  else

    Serial.println("\n//Done.\n");

  // Put any changes to bytes immediately after the dump, since that will ensure they hit the found i2c device.

  delay(5000);           // Wait 5 seconds then re-scan.
}

void dump24c02(byte i2cAddress)
{
  int addrPointer = 0;
  //int romLength = 0xff;    // 24c02 - 256x8 bits (256 bytes)
  int romLength = 0xff;   // 24c04 - 512x8 bits (512 bytes)
  byte b = 0;

  // Spit out each 256 byte page of the eeprom contents as a byte array
  Serial.print("static const byte eeprom");
  Serial.print(i2cAddress, HEX);
  Serial.println("[][0x10] =");
  Serial.println("{");

  while (addrPointer <= romLength)
  {

    if ((addrPointer % 16) == 0 ) {

      if (addrPointer == 0) {
        Serial.print(" {");
      } else {
        Serial.print("} //0x0");
        Serial.println(addrPointer, HEX);
        Serial.print(",{");
      }
    } else {
        Serial.print(",");

    }

    b = i2c_eeprom_read_byte(i2cAddress, addrPointer);  // Read byte
    addrPointer++;                                      // increment address pointer
    Serial.print("0x");
    if (b < 0x10) {
      Serial.print("0");
    }

    Serial.print(b, HEX);                               // Print byte
    //Serial.print(",");
  }
  Serial.print("} //End of page 0x");

  // Serial.print("// ");
  Serial.print((i2cAddress - 0x50), HEX);
  Serial.println();

  if (!(addrPointer % 16)) {
    Serial.println("};");

  } else {
    Serial.println("");
  }

}

byte i2c_eeprom_read_byte( int deviceaddress, int eeaddress ) {

  //Wire.beginTransmission(deviceaddress & ((eeaddress & (int)0x100 ) >> 9));
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress & 0xff));
  // Wire.write(address & 0xFF);
  //Wire.write((int)(eeaddress >> 8)); // MSB
  //Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);

  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0xff;
  }

}


void i2c_eeprom_write_byte( int deviceaddress, int eeaddress, byte data ) {
  digitalWrite(BOARD_WP, LOW);
  delay(100);
  int rdata = data;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress)); //
  Wire.write(rdata);
  Wire.endTransmission();
  //digitalWrite(BOARD_WP, HIGH);
}

