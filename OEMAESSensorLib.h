/*
  OEMAESSensorLib.cpp - Library for OpenEnergyMonitor Sensor Nodes
  Supporting AES Encryption and new packet format
  Copyright Stuart Pittaway, Feb 25, 2016.
*/
#ifndef OEMSENSORLIB_H
#define OEMSENSORLIB_H

#include <AESLib.h>
#include <util/crc16.h>
#include <JeeLib.h>                                                   // https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14
#include <avr/eeprom.h>
#include <avr/wdt.h>

#define clearpacket(x) memset(&x.packet, 0,  sizeof x.packet);

//Byte 1
//  Bits 0-3 = Unused
//  Bits 4-7 = Sensor Type
//                                                  76543210
const uint8_t OEM_Sensor_Type_Unknown            = B00000000;
const uint8_t OEM_Sensor_Type_TemperatureCelsius = B00010000;
const uint8_t OEM_Sensor_Type_RelativeHumidity   = B00100000;
const uint8_t OEM_Sensor_Type_PulseCounter       = B00110000;
const uint8_t OEM_Sensor_Type_KilowattHour       = B01000000;
const uint8_t OEM_Sensor_Type_Volt               = B01010000;
const uint8_t OEM_Sensor_Type_Amps               = B01100000;
const uint8_t OEM_Sensor_Type_Watts              = B01110000;
const uint8_t OEM_Sensor_Type_Ohms               = B10000000;
const uint8_t OEM_Sensor_Type_BooleanSwitch      = B10010000;
const uint8_t OEM_Sensor_Type_Capacitance        = B10100000;
const uint8_t OEM_Sensor_Type_CubicMetre = B10110000;
const uint8_t OEM_Sensor_Type_BatteryVoltage = B11000000; //Internal battery voltage
const uint8_t OEM_Sensor_Type_spare14 = B11010000;
const uint8_t OEM_Sensor_Type_spare15 = B11100000;

//Byte 2
//  Bits 0-1 = Byte length of readings (number of bytes in transmitted packet for a single reading of this sensor, 1=byte, 2=int, 4=long) - assume all values are integers
//  Bits 2-3 = Divisor scale (divide integer value by this scale to get float point reading - if needed)
//  Bit  4   = Label defined
//  Bit  5   = LittleEndian = 0
//  Bit  6   = Unused
//  Bit  7   = Signed value (integer value is signed)
//                                          76543210
const uint8_t OEM_Sensor_ByteLen_byte   = B00000000;
const uint8_t OEM_Sensor_ByteLen_int    = B00000001;
const uint8_t OEM_Sensor_ByteLen_triple = B00000010;
const uint8_t OEM_Sensor_ByteLen_long   = B00000011;

const uint8_t OEM_Sensor_Scale_1       = B00000000;
const uint8_t OEM_Sensor_Scale_10      = B00000100;
const uint8_t OEM_Sensor_Scale_100     = B00001000;
const uint8_t OEM_Sensor_Scale_1000    = B00001100;

const uint8_t OEM_Sensor_SignedValue   = B10000000;
const uint8_t OEM_Sensor_UnsignedValue = B00000000;
const uint8_t OEM_Sensor_Unused        = B00010000;

//First 4 bits
const uint8_t OEM_Packet_LittleEndian  = B00000000;
const uint8_t OEM_Packet_BigEndian     = B10000000;

const uint8_t OEM_Packet_NoReply       = B00000000;
const uint8_t OEM_Packet_ExpectReply   = B01000000;

const uint8_t OEM_Packet_Encrypted     = B00100000;
//Indicates last two bytes of packet contain a calculated CRC16 - useful to ensure decryption was correct
const uint8_t OEM_Packet_WithCRC       = B00010000;

//These are OUTBOUND from Sensor to Base message types
const uint8_t OEM_Packet_Type_Data                    = B00000000;
const uint8_t OEM_Packet_Type_Descriptor              = B00000001;
const uint8_t OEM_Packet_Type_Labels                  = B00000010;
const uint8_t OEM_Packet_Type_LocateBaseAnnouncement  = B00000011;
const uint8_t OEM_Packet_Type_InitializeSensor        = B00000100;

//These are OUTBOUND from Base to Sensor message types
const uint8_t OEM_Packet_Type_Sensor_SetEncryptionKey = B00000101;


struct EEPROMSerialNumber {
  uint16_t serial1;
  uint16_t serial2;
  uint16_t crc;
};

struct EEPROMStructure {
  uint8_t Version; //1
  uint8_t nodeId;
  uint8_t networkGroup;
  uint8_t baseStationNodeId;
  uint8_t key[16];
  //AES IV
  uint8_t iv[2];
  uint16_t packetCounter;
  uint8_t iv_remainder1[4];
  uint8_t iv_remainder2[8];
  uint16_t crc;
};


struct __attribute__((packed)) SensorUnencryptedPacketHeader {
  //7 byte unencrypted header
  uint8_t SendingNodeId;  //This is deliberately 8 bit even though RFM12 can only have 30 nodes - future expansion
  uint8_t PacketType;   //1byte
  uint8_t SensorValues;
  uint8_t IVPrefix[2];   //2byte
  uint16_t SendCounter = 0;   //2byte
};

//Automatically pad the payload to correct length, (having a second structure is wasteful on RAM!)
//however the original structure gets destroyed by the AES encryption.
struct __attribute__((packed)) SensorPacket16_t {
  SensorUnencryptedPacketHeader sph;
  /* All the fields below are encrypted - and padded to 16 bytes in length - it must be a multiple of 16 bytes */
  uint8_t packet[16];
};

//__attribute__((packed))
struct  __attribute__((packed)) SensorPacket32_t {
  SensorUnencryptedPacketHeader sph;
  /* All the fields below are encrypted - and padded to 32 bytes in length - it must be a multiple of 16 bytes */
  uint8_t packet[32];
};

struct  __attribute__((packed)) SensorPacket48_t  {
  SensorUnencryptedPacketHeader sph;
  /* All the fields below are encrypted - and padded to 48 bytes in length - it must be a multiple of 16 bytes */
  uint8_t packet[48];
};

class OEMAESSensorLib
{
  public:
    static bool GetConfigurationFromEEPROM(EEPROMStructure &eepromdata);
    static void DefaultEEPROM (EEPROMStructure &data);
    static void GetSerialNumber(EEPROMSerialNumber &data, uint8_t buttonPin , uint8_t ledPin);
    static void LocateBaseStation(EEPROMSerialNumber &serial, EEPROMStructure &data, uint8_t ledPin);
    static void Encrypt(EEPROMStructure &data, void* dataptr, int len);
    static void Decrypt(EEPROMStructure &data, void* dataptr, int len);
    static void RandomStartupDelay(EEPROMSerialNumber &serial);
    static void IncrementPacketCounter(EEPROMStructure &data, SensorUnencryptedPacketHeader &sph);
    static void SendPacketRFM(SensorUnencryptedPacketHeader &sph, EEPROMStructure &data, uint8_t settings, void* pkt, int sendLength);
    static void ApplyCRCToPacket(SensorUnencryptedPacketHeader &sph, uint8_t pkt[], int packetLength);

  private:
    static void Reboot();
    static void WriteEEPROMStructuretoEEPROM(EEPROMStructure &data);
    static void WriteSerialToEEPROM(EEPROMSerialNumber &data);
    static bool ExtractEEPROMSerialToRAM(EEPROMSerialNumber &eepromdata);
    static void GenerateSerialNumber(EEPROMSerialNumber &data, uint8_t buttonPin , uint8_t ledPin);
    static word calcCrc (const void* ptr, uint8_t len);
    static void LEDPattern(byte ledPin, int loop, int delay1, int delay2);
    static const uint8_t LibraryVersion = 2;

    //Use EEPROM addresses near 512 byte boundary
    static const uint16_t eeprom_location_of_serial = (512 - sizeof(EEPROMSerialNumber));
    static const uint16_t eeprom_location_of_structure = (512 - sizeof(EEPROMSerialNumber) - sizeof(EEPROMStructure));
};

#endif
