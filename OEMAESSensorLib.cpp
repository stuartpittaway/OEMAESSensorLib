/*
  OEMAESSensorLib.cpp - Library for OpenEnergyMonitor Sensor Nodes
  Supporting AES Encryption and new packet format
  Copyright Stuart Pittaway, Feb 25, 2016.
*/

#include "OEMAESSensorLib.h"
#include <avr/eeprom.h>


/*
// eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
  // this uses 166 bytes less flash than eeprom_read_block(), no idea why
  for (byte i = 0; i < sizeof config; ++ i)
    ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
 */

bool OEMAESSensorLib::ExtractEEPROMSerialToRAM(EEPROMSerialNumber &data) {
  eeprom_read_block((void*)&data, (void*)eeprom_location_of_serial, sizeof(data));
  word calculatedCRC = calcCrc(&data, sizeof data - 2);
  return (data.crc == calculatedCRC);
}

void OEMAESSensorLib::WriteSerialToEEPROM(EEPROMSerialNumber &data) {
  data.crc = calcCrc(&data, sizeof data - 2);
  eeprom_write_block((const void*)&data, (void*)eeprom_location_of_serial, sizeof(data));
}

bool OEMAESSensorLib::GetConfigurationFromEEPROM(EEPROMStructure &data) {
  eeprom_read_block((void*)&data, (void*)eeprom_location_of_structure, sizeof(data));
  word calculatedCRC = calcCrc(&data, sizeof data - 2);

  //Seed the IV value using 8 bytes of the secret key
  memcpy(&data.iv_remainder2, data.key + 8, 8);

  return (data.crc == calculatedCRC && data.Version == LibraryVersion);
}

void OEMAESSensorLib::WriteEEPROMStructuretoEEPROM(EEPROMStructure &data) {
  data.crc = calcCrc(&data, sizeof data - 2);
  eeprom_write_block((const void*)&data, (void*)eeprom_location_of_structure, sizeof(data));
}

//CRC-16 (Modbus) calculation
word OEMAESSensorLib::calcCrc (const void* ptr, uint8_t len) {
  word crc = ~0;
  for (uint8_t i = 0; i < len; ++i)
    crc = _crc16_update(crc, ((const uint8_t*) ptr)[i]);
  return crc;
}

void OEMAESSensorLib::DefaultEEPROM (EEPROMStructure &data) {
  //Start with a blank configuration
  memset(&data, 0, sizeof data);
  //Hardcoded defaults to allow discovery to work
  data.Version = LibraryVersion;
  data.nodeId = 29; //Base is 30, we use 29 as a temporary discovery channel
  data.networkGroup = 210; //open energy monitor standard group
  data.baseStationNodeId = 30;
  data.packetCounter = 0;
}


void OEMAESSensorLib::GenerateSerialNumber(EEPROMSerialNumber &data, uint8_t buttonPin , uint8_t ledPin) {
  /*
     This function makes the user generate a random serial number for this sensor which is stored in EEPROM
     it lights up the LED, waits for the user to press a button, then repeats
     The random delay in pressing the button is used to generate a unique(ish) 4 uint8_t serial number
     Once in EEPROM, we use this serial number even between resets to factory defaults
  */

  //DEBUG
  //Serial.println(F("GenerateSerialNumber"));
  
  //Enable pullup, so switched to ground
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  uint16_t* ptr = &data.serial1;

  //Loop twice
  for (int i = 0; i < 2; i++) {

    //LED ON
    digitalWrite(ledPin, HIGH);

    //Loop until button press
    while (digitalRead(buttonPin) == HIGH) {
      delayMicroseconds(35);
    }
    //LED OFF
    digitalWrite(ledPin, LOW);

    //Store 2 bytes - note you can remove the random function if memory gets tight
    ptr[i] = (uint16_t)((micros() ^ random()) & 0x0000FFFF);
    if (i == 0) {
      randomSeed(ptr[i]);
    }

    //Wait for a little so the user can see the acknowledgement of the button press (LED goes out)
    delay(750);
  }
  //Restore state of pin
  pinMode(buttonPin, INPUT);

  data.crc = calcCrc(&data, sizeof data - 2);

  WriteSerialToEEPROM(data);
}

void OEMAESSensorLib::LEDPattern(uint8_t ledPin, int loop, int delay1, int delay2) {
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < loop; i++) {
    digitalWrite(ledPin, HIGH);
    delay(delay1);
    digitalWrite(ledPin, LOW);
    delay(delay1);
    digitalWrite(ledPin, HIGH);
    delay(delay1);
    digitalWrite(ledPin, LOW);
    delay(delay2);
  }
}

void OEMAESSensorLib::GetSerialNumber(EEPROMSerialNumber &serial, uint8_t buttonPin , uint8_t ledPin) {
  //We should check here for a "config clear" pin/button and act accordingly
  bool validSerial = ExtractEEPROMSerialToRAM(serial);

  if (!validSerial) {
    //Constant duration on/off to indicate serial number setup
    LEDPattern(ledPin, 4, 250, 250);
    //First step of the configuration is to create a random serial number
    GenerateSerialNumber(serial, buttonPin, ledPin);
  }
}

void OEMAESSensorLib::RandomStartupDelay(EEPROMSerialNumber &serial) {
  //Apply a random seed based on serial number and then delay between 0 and 8 seconds
  //this could be used to stagger readings from sensors which are all powered up at the same time
  //to prevent wireless collisions blocking each other
  randomSeed(serial.serial1);
  delay(random(8000));
}

void OEMAESSensorLib::LocateBaseStation(EEPROMSerialNumber &serial, EEPROMStructure &data, uint8_t ledPin) {
  //When this is called, we expect the RFM module to already be initialised
  //rf12_sleep(RF12_WAKEUP);
  //delay(100);

  rf12_recvDone();

  SensorPacket16_t pkt;

  memset(&pkt, 0, sizeof pkt);

  //Copy serial number into packet
  memcpy(&pkt.packet, &serial.serial1,  4);

  //Node ID 0 and 31 are special, so there can be 30 different nodes in the same net group.
  //We use the default 29 node id here during discovery modes
  pkt.sph.SendingNodeId = data.nodeId;
  pkt.sph.PacketType = OEM_Packet_Type_LocateBaseAnnouncement | OEM_Packet_LittleEndian | OEM_Packet_ExpectReply;

  bool allGood = false;

  while (!allGood) {
    //Fast on/off to indicate base station search
    LEDPattern(ledPin, 2, 75, 600);

    //Clear buffer if something is sitting in it
    //while (!rf12_canSend()) {      rf12_recvDone();    }

    //Serial.print(F("Sending:"));
    //Serial.println(sizeof pkt);
    //Send our packet directly to the base and ask for an ACK
    //rf12_sendStart(RF12_HDR_ACK | RF12_HDR_DST | data.baseStationNodeId, &pkt, sizeof pkt);
    //rf12_sendWait(1);

    SendPacketRFM(pkt.sph, data, RF12_HDR_ACK | RF12_HDR_DST | data.baseStationNodeId, &pkt, sizeof pkt);

    // wait up to 500 ms to get an ack back
    MilliTimer t1;
    while (!t1.poll(500) && !allGood) {
      if (rf12_recvDone() && (rf12_crc == 0) && rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | data.nodeId)) {
        //Now we can wait for our initialize packet to be received OEM_Packet_Type_InitializeSensor
        MilliTimer t2;
        while (!t2.poll(1000) && !allGood) {
          if (rf12_recvDone()) {
            //‭XX1 XXXXX‬=RF12_ACK_REPLY - The A bit (ACK) indicates whether this packet wants to get an ACK back.
            //‭X1X XXXXX‬=RF12_HDR_DST - The D bit (DST) indicates whether the node ID specifies the destination node or the source node. For packets sent to a specific node, DST = 1
            //1XX XXXXX=RF12_HDR_CTL - The C bit (CTL) is used to send ACKs, and in turn must be combined with the A bit set to zero.

            if ( rf12_crc == 0 && ((rf12_hdr & RF12_HDR_DST) == RF12_HDR_DST)  &&  ((rf12_hdr & RF12_HDR_MASK) == data.nodeId)) {
              //Send ACK
              if (RF12_WANTS_ACK) {
                rf12_sendStart(RF12_ACK_REPLY | RF12_HDR_DST | (rf12_data[0] & RF12_HDR_MASK), 0, 0);
                rf12_sendWait(1);
              }

              if (rf12_data[1] == OEM_Packet_Type_Sensor_SetEncryptionKey) {

                allGood = true;
                //Process the packet - populate
                data.nodeId = rf12_data[2];
                data.networkGroup = rf12_data[3];
                data.baseStationNodeId = rf12_data[4];
                
                memcpy(&data.key, (void*)&rf12_data[5], sizeof data.key);

                //Seed the IV value using 8 bytes of the secret key
                memcpy(&data.iv_remainder2, data.key + 8, 8);

                WriteEEPROMStructuretoEEPROM(data);
              }
              //Finally everything is good, quit loop(s)
            }
          }
        }
        //end of while t2 loop
        //Serial.println(F("Abort t2"));
      }//end if
    }
    //Serial.println(F("Abort t1"));
  }

  rf12_sleep(RF12_SLEEP);

  LEDPattern(ledPin, 2, 1000, 1000);

  Reboot();
}

void OEMAESSensorLib::Reboot()
{
  wdt_enable(WDTO_15MS);
  noInterrupts();
  // Set the "external reset" flag so the bootloader runs.
  while (true)
    MCUSR |= _BV(EXTRF);
}

void OEMAESSensorLib::Encrypt(EEPROMStructure &data, void* dataptr, int len) {
  //encrypt multiple blocks of 128bit data, len must be mod 16, key and iv are assumed to be both 128bit thus 16 uint8_t's
  aes128_cbc_enc(data.key, data.iv, dataptr, len);
}

void OEMAESSensorLib::Decrypt(EEPROMStructure &data, void* dataptr, int len) {
  //Decrypt
  aes128_cbc_dec(data.key, data.iv, dataptr, len);
}

void OEMAESSensorLib::IncrementPacketCounter(EEPROMStructure &data, SensorUnencryptedPacketHeader &sph) {
  //Increment the internal packet counter, and occasionally write this back to EEPROM
  //by writing this back to EEPROM we could use this counter on the base station
  //as a basic anti-packet replay attack - although its not fool proof!
  data.packetCounter += 1;

  sph.SendCounter = data.packetCounter;

  unsigned long t = millis();
  memcpy(sph.IVPrefix, &t, 2);
  memcpy(&data.iv, &t, 2);

  //Every 100 increments, write to EEPROM, ideally we would only write the bytes that have changed
  //we should add in some sort of EEPROM wear levelling feature here
  if ((data.packetCounter % 100) == 0) {
    WriteEEPROMStructuretoEEPROM(data);
  }
}



void OEMAESSensorLib::SendPacketRFM(SensorUnencryptedPacketHeader &sph, EEPROMStructure &data, uint8_t settings, void* pkt, int sendLength) {
  //Clear receive buffer
  while (!rf12_canSend()) {
    rf12_recvDone();
  }

  //Send message directed at the base node
  rf12_sendStart(settings, pkt, sendLength);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);

  if ((sph.PacketType & OEM_Packet_ExpectReply) == OEM_Packet_ExpectReply) {
    //We asked base to reply to us with a status message which we may act upon
  }
}

void OEMAESSensorLib::ApplyCRCToPacket(SensorUnencryptedPacketHeader &sph, uint8_t pkt[], int packetLength) {
  //Due to the use of AES encryption, we generally have a couple of spare bytes in the end of the packet.  Put these to good use!
  
  //Calculate a CRC16 value of the packet data and puts this into the last two bytes of the
  
  //packet - this can then be used on encrypted packets to ensure that the decryption was successful
  //for error checking purposes.  
  
  //This won't work with a "full" packet - so there is a simple check to ensure the last two bytes of the packet are zeros.
  word calculatedCRC;

  uint8_t *ptr=&pkt[packetLength - 2];
  
  memcpy(&calculatedCRC, ptr, sizeof calculatedCRC);

  //Only apply the CRC to the packet if the bytes are zero (simply safety check)
  if (calculatedCRC == 0) {
    sph.PacketType = (sph.PacketType | OEM_Packet_WithCRC);

    calculatedCRC = calcCrc(pkt, packetLength - 2);
    memcpy(ptr, &calculatedCRC, sizeof calculatedCRC);
  } else {
    //Clear WithCRC flag as we are not using one here
    sph.PacketType = (sph.PacketType & !OEM_Packet_WithCRC);
  }

}


