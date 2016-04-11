/*
   SAMPLE SENSOR NODE FOR OPEN ENERGY MONITOR

   USING SELF DESCRIBING/CONFIGURATION AND ENCRYPTED COMMUNICATIONS
   
   STUART PITTAWAY, FEB 2016

   SENDS MEANINGLESS SENSOR DATA OVER TO OEM RECEIVER FOR TEST PURPOSES

   COMPILES WITH ARDUINO 1.6.7 - 12,112 bytes
*/
//Arduino uses standard HW Serial port
#define HWSERIAL Serial

#define LED_PIN 5
//#define LED_PIN 9

//RFM module chip select pin (SPI bus)
#define RF12_CS_PIN 10

#define BUTTON_PIN 7

const uint8_t debug = 1;


//Up to 16 characters max - include version number if possible
//                             AAAAAAAAAAAAAAAA
static const char myVersion[]="Test Sensor V1.0";

/*
    Include libraries
*/
#include "OEMAESSensorLib.h"

#define RF69_COMPAT 0                                                 // Set to 1 if using RFM69CW or 0 is using RFM12B
#include <JeeLib.h>                                                   // https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14

/* The frequency of the RFM module must be configured here as its hardware dependant
  Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ.
  You should use the one matching the module you have.
*/
#define RF_freq RF12_433MHZ


#include <avr/power.h>
#include <avr/sleep.h>     
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 


//This could be in progmem
static uint8_t payload_descriptor[] = {
  //byte=Number of fields in output
  OEM_Sensor_Type_TemperatureCelsius, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_TemperatureCelsius, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_TemperatureCelsius, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_TemperatureCelsius, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_RelativeHumidity, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_RelativeHumidity, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_Volt, OEM_Sensor_ByteLen_int | OEM_Sensor_Scale_10 | OEM_Sensor_SignedValue
  , OEM_Sensor_Type_PulseCounter, OEM_Sensor_ByteLen_long | OEM_Sensor_Scale_1 | OEM_Sensor_UnsignedValue
};
//This could be in progmem
static const char payload_labels[] = "T1|T2|T-Ext|Humidity|Batt|Count";

static void activityLed (byte on) {
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, on);
#endif
}

static EEPROMSerialNumber eeserial;
static EEPROMStructure eedata;
static OEMAESSensorLib sensorlib;

//We use a 32 byte buffer for the encrypted data (plus header structure)
static SensorPacket32_t sensorpkt;

// These are our fields we will send to the gateway (after encryption)
typedef struct {
  int temp1;
  int temp2;
  int temp3;
  int temp4;
  int humidity1;
  int humidity2;
  int battery;
  unsigned long pulsecount;
} Payload;

static Payload readings;

inline char i2h(uint8_t i)
{ 
  char k = i & 0x0F;
  return  (k <= 9) ? '0' + k : ('A'-10) + k ;   
}

static void dumpHex(uint8_t number)
{
  if (debug == 1)
  {
    HWSERIAL.print(i2h(number >> 4));
    HWSERIAL.print(i2h(number));
  }
}

static void dumpBytes (char *ptr, byte len) {
  if (debug == 1)
  {
    for (uint8_t i = 0; i < len; i++) {
      dumpHex( ptr[i] );
    }
    HWSERIAL.print('\n');
    //Flush here otherwise we get garbled characters in Serial because we use the Sleepy library - see this problem http://jeelabs.net/boards/7/topics/1760
    HWSERIAL.flush();
  }
}

//CHECKOUT http://www.rotwang.co.uk/projects/bootloader.html
void setup() {
  if (debug == 1)
  {
    HWSERIAL.begin(38400);
  }


  //Led on
  activityLed(1);

  //Set chip select pin for RFM module
#if !defined(RF69_COMPAT)
  rf12_set_cs  ( RF12_CS_PIN ) ;
#endif

  sensorlib.GetSerialNumber(eeserial, BUTTON_PIN, LED_PIN);

  //Debug
  //dumpBytes((char *)&eeserial, sizeof eeserial - 2);

  bool validEEPROM = sensorlib.GetConfigurationFromEEPROM(eedata);

  //Debug
  //dumpBytes((char *)&eedata, sizeof eedata);

  if (!validEEPROM) {
    //Configuration is invalid, so enter config mode
    //Start with default settings so we can find base station (sets group and nodeid)
    sensorlib.DefaultEEPROM(eedata);
  }

  // Initialize RFM12B and then put into sleep mode
  rf12_initialize(eedata.nodeId, RF_freq, eedata.networkGroup);

  if (!validEEPROM) {
    //Enter discovery mode for self-configuration - note this function will never return Arduino will reboot after configuration
    sensorlib.LocateBaseStation(eeserial, eedata, LED_PIN);
  }

  sensorlib.RandomStartupDelay(eeserial);

  rf12_sleep(RF12_WAKEUP);
  delay(50);

  SensorPacket48_t pkt1;
  clearpacket(pkt1)  
  pkt1.sph.PacketType = OEM_Packet_Type_Descriptor | OEM_Packet_LittleEndian | OEM_Packet_Encrypted; //| OEM_Packet_ExpectReply
  memcpy(pkt1.packet, &eeserial.serial1, sizeof eeserial.serial1 + sizeof eeserial.serial2);
  memcpy(pkt1.packet + 4, &myVersion, sizeof myVersion);
  memcpy(pkt1.packet + 4 + 16, &payload_descriptor, sizeof payload_descriptor);
  
  sensorlib.IncrementPacketCounter(eedata, pkt1.sph);
  sensorlib.ApplyCRCToPacket(pkt1.sph, pkt1.packet, sizeof pkt1.packet);
  //Debug
  //dumpBytes((char *)&pkt1.packet, sizeof(pkt1.packet));
  sensorlib.Encrypt(eedata, &pkt1.packet, sizeof pkt1.packet);
  sensorlib.SendPacketRFM(pkt1.sph, eedata, RF12_HDR_DST | eedata.baseStationNodeId, &pkt1, sizeof pkt1);
  dodelay(500);


  SensorPacket48_t pkt2;
  clearpacket(pkt2)
  pkt2.sph.PacketType = OEM_Packet_Type_Labels | OEM_Packet_LittleEndian | OEM_Packet_Encrypted ; //| OEM_Packet_ExpectReply
  memcpy(pkt2.packet, &payload_labels, sizeof payload_labels);
  sensorlib.IncrementPacketCounter(eedata,pkt2.sph);
  sensorlib.ApplyCRCToPacket(pkt2.sph, pkt2.packet, sizeof pkt2.packet);  
  //Debug
  //dumpBytes((char *)&pkt2.packet, sizeof(pkt2.packet));
  sensorlib.Encrypt(eedata, &pkt2.packet, sizeof pkt2.packet);
  sensorlib.SendPacketRFM(pkt2.sph, eedata, RF12_HDR_DST | eedata.baseStationNodeId, &pkt2, sizeof pkt2);

  rf12_sleep(RF12_SLEEP);
  delay(50);

  //TODO: Allow a sensor to send more than 2 readings - register a 2nd node id ??

  //Tell receiver we use a LittleEndian storage format (and we will also expect a reply for each packet we send)
  sensorpkt.sph.PacketType = OEM_Packet_Type_Data | OEM_Packet_LittleEndian | OEM_Packet_Encrypted; //| OEM_Packet_ExpectReply
  //We are sending all readings in each packet
  sensorpkt.sph.SensorValues = 0xFF;

  readings.temp1 = 50;
  readings.temp2 = 60;
  readings.temp3 = 70;
  readings.pulsecount = 0x12345678;
  
  activityLed(0);
}


void loop() {
  //HWSERIAL.println("loop()");

  //Clear old packet otherwise we have encrypted bytes left in buffer
  clearpacket(sensorpkt)

  //HWSERIAL.print(F("R1 :"));  dumpBytes((char *)&sensorpkt, sizeof sensorpkt);

  sensorpkt.sph.SensorValues = 0xFF;
  readings.temp1++;
  readings.temp2++;
  readings.temp3++;
  readings.battery = 0; //int(analogRead(BATT_ADC) * 0.03225806);                //read battery voltage, convert ADC to volts x10
  readings.pulsecount--;

  //Copy readings into packet data to be sent
  memcpy(&sensorpkt.packet, &readings, sizeof(readings));

  //Increment packet counter
  sensorlib.IncrementPacketCounter(eedata,sensorpkt.sph);
  //Calculate internal checksum on packet (last 2 bytes of encrypted payload)
  sensorlib.ApplyCRCToPacket(sensorpkt.sph, sensorpkt.packet, sizeof sensorpkt.packet);  
  //Encrypt packet with AES128
  sensorlib.Encrypt(eedata, &sensorpkt.packet, sizeof sensorpkt.packet);

  //Send the packet over RFM chip (with power saving wrapper)
  power_spi_enable();
  rf12_sleep(RF12_WAKEUP);
  dodelay(50);
  sensorlib.SendPacketRFM(sensorpkt.sph, eedata, RF12_HDR_DST | eedata.baseStationNodeId, &sensorpkt, sizeof sensorpkt);
  rf12_sleep(RF12_SLEEP);
  dodelay(50);
  power_spi_disable();  

  //Flash LED
  activityLed(1);
  dodelay(25);
  activityLed(0); 

  //Now decrypt the payload for the next run
  //sensorlib.Decrypt(eedata, &sensorpkt.packet, sizeof sensorpkt.packet);
  //HWSERIAL.print(F("R5 :"));  dumpBytes((char *)&sensorpkt, sizeof sensorpkt);

  //Sleep 2 seconds
  dodelay(2000);
}

void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
      
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

