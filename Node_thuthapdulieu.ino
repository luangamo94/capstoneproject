/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include "STM32LowPower.h"
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
#define relayPin 8
SoftwareSerial sSerial (6, 7);
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x15, 0x12 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xE6, 0xCC, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xE6, 0x09, 0xC0, 0x99, 0x85, 0xD8, 0xD9, 0x33, 0xCC, 0x8B, 0xD5, 0x26, 0xFA, 0x8E, 0x20, 0x24 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
uint16_t DO_add = 2;
uint16_t ORP_add = 1;
ModbusMaster node;
float nhietdo = 0.0, nhietdo_tb = 0.0;
float nongdo =0.0, nongdo_tb = 0.0;
int ORP_value = 0,ORP_tb = 0;

int flag = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = A0,
    .dio = {2, 3, 4},
};
void DO_init(ModbusMaster _node) {
  float sum1 = 0.0;
  float sum2 = 0.0;
  float sum3 =0.0;
  int counter1 =0, counter2 =0,counter3 =0;
  uint8_t result;
  long int num[3];
  float fvalue[3];  // gia tri dang so thuc
  for(int i = 0; i < 20; i++){
  _node.begin(DO_add, sSerial);
  result = _node.readHoldingRegisters(0x0000, 6);  // doc tu thanh ghi 0x0000, so luong 6
  if (result == node.ku8MBSuccess)                 // nhan thanh cong tra ve 0x00
  {
    num[1] = (((unsigned long)_node.getResponseBuffer(0x02) & 0xFFFF) << 16) | (_node.getResponseBuffer(0x03) & 0xFFFF);
    memcpy(&fvalue[1], &num[1], 4);
    nongdo = fvalue[1];
    if(nongdo != 0){
      sum2 +=nongdo;
      counter2 ++;
    }
    // Nhiet do (don vi: do C
    num[2] = (((unsigned long)_node.getResponseBuffer(0x04) & 0xFFFF) << 16) | (_node.getResponseBuffer(0x05) & 0xFFFF);
    memcpy(&fvalue[2], &num[2], 4);
    nhietdo = fvalue[2];
    if(nhietdo != 0){
    sum3 += nhietdo;
    counter3 ++;}
    
  }
  delay(500);
  }
  Serial.print("counter1 = ");
  Serial.println(counter1);
  Serial.print("counter2 = ");
  Serial.println(counter2);
  Serial.print("counter3 = ");
  Serial.println(counter3);
  nongdo_tb   = sum2/counter2;
  nhietdo_tb  = sum3/counter3;
}

void ORP_init(ModbusMaster _node) {
  uint8_t result;

  _node.begin(ORP_add, sSerial);
  result = _node.readHoldingRegisters(0x0000, 1);  // doc tu thanh ghi 0x0000, so luong 6
  if (result == node.ku8MBSuccess)                 // nhan thanh cong tra ve 0x00
  {
    // gia tri ORP (don vi: mV)
    ORP_value = _node.getResponseBuffer(0x00);
  }
}
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            // LowPower.deepSleep(60000);
            //   Serial.println("hello");
            delay(5000);
             flag = 1;

            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      DO_init(node);
      int T = (int)(nhietdo_tb*10);
      int DO = (int)(nongdo_tb*10);
      ORP_init(node);
      int ORP = ORP_value;
      Serial.println(T);
      Serial.println(DO);
      Serial.println(ORP);
      byte mydata[5];
      mydata[0] = highByte(T);
      mydata[1] = lowByte(T);
      mydata[2] = highByte(DO);
      mydata[3] = lowByte(DO);
      mydata[4] = highByte(ORP);
      mydata[5] = lowByte(ORP);
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(4800);
    sSerial.begin(4800);
    pinMode(relayPin,OUTPUT);
    digitalWrite(relayPin,HIGH);
    Serial.println(F("Starting"));
    LowPower.begin();
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    // do_send(&sendjob);
}

void loop() {
  do_send(&sendjob);
  while(flag == 0){
  os_runloop_once();
  }
  flag = 0;
  digitalWrite(relayPin, LOW);
  LowPower.deepSleep(60000);
  digitalWrite(relayPin,HIGH);

}
