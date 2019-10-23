#include <Arduino.h>
#include <WiFi.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SimpleButton.h>
#include <U8x8lib.h>
#include <gps.h>

// EDIT AND INSERT YOUR ABP KEYS HERE!
#include <config.h>

// Extra Buttons
#define BUTTON_L 13
#define BUTTON_R 2
// Integrated Button
#define BUTTON_B 39
// Pin to read Battery Voltage
#define BAT_PIN  35

// Thread Handle for UI
TaskHandle_t uiThreadTask;

// Delay between Lora Send
uint8_t sendInterval[] = {20, 30, 40, 10};
uint8_t sendIntervalKey = 0;

// TinyGPS Wrapper
gps gps;

// Global Status Flags
float vbat;
String LoraStatus;
boolean hasFix = false;

static osjob_t sendjob;

uint8_t  loraBuffer[9];
uint16_t dispBuffer[6];

// Empty Callbacks since we are not using OTAA
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void do_send(osjob_t* j); // declare for onEvent

// Setup Lora Pins
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, // was "14,"
  .dio = {26, 33, 32},
};

// Lora Event Handling
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      LoraStatus = "SCANTIMEO";
      break;
    case EV_BEACON_FOUND:
      LoraStatus = "BEAC_FOUN";
      break;
    case EV_BEACON_MISSED:
      LoraStatus = "BEAC_MISS";
      break;
    case EV_BEACON_TRACKED:
      LoraStatus = "BEAC_TRAC";
      break;
    case EV_JOINING:
      LoraStatus = "JOINING";
      break;
    case EV_JOINED:
      LoraStatus = "JOINED";
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      LoraStatus = "RFU1";
      break;
    case EV_JOIN_FAILED:
      LoraStatus = "JOIN_FAIL";
      break;
    case EV_REJOIN_FAILED:
      LoraStatus = "REJOIN_FA";
      break;
    case EV_TXCOMPLETE:
      LoraStatus = "TXCOMPL";
      digitalWrite(BUILTIN_LED, LOW);  
      if (LMIC.txrxFlags & TXRX_ACK) {
        LoraStatus = "Recvd Ack";
      }
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks((sendInterval[sendIntervalKey])), do_send);
      break;
    case EV_LOST_TSYNC:
      LoraStatus = "LOST_TSYN";
      break;
    case EV_RESET:
      LoraStatus = "RESET";
      break;
    case EV_RXCOMPLETE:
      LoraStatus = "RXCOMPL";
      break;
    case EV_LINK_DEAD:
      LoraStatus = "LINK_DEAD";
      break;
    case EV_LINK_ALIVE:
      LoraStatus = "LINK_ALIV";
      break;
    default:
      LoraStatus = "UNKNOWN";
      break;
  }
}

void do_send(osjob_t* j) {  

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    LoraStatus = "TXRXPEND";
  }
  else
  { 
    if (gps.checkGpsFix())
    {
      // Prepare upstream data transmission at the next possible time.
      gps.buildPacket(loraBuffer);
      LMIC_setTxData2(1, loraBuffer, sizeof(loraBuffer), 0);
      digitalWrite(BUILTIN_LED, HIGH);
      LoraStatus = "QUEUED";
    }
    else
    {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
}

// UI Thread
void uiThread (void * parameter) {
  // Initialize Buttons
  simplebutton::Button* b1 = new simplebutton::ButtonPullup(BUTTON_R);
  b1->setOnClicked([]() {
    sendIntervalKey++;
    if (sendIntervalKey >= (sizeof(sendInterval)/sizeof(*sendInterval)))
      sendIntervalKey = 0;
  });

  // Initialize Display
  U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

  u8x8.begin();
  u8x8.setPowerSave(0);

  // Endless Loop for Display Thread
  bool lastState = hasFix;
  for(;;) {
    b1->update();
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    if (lastState != hasFix) {
      lastState = hasFix;
      u8x8.clear();
    } else {
      u8x8.home();
    }
    if (hasFix) {
      u8x8.print("HDOP:  ");
      u8x8.println(dispBuffer[4] / 10.0);
      u8x8.print("Sat:   ");
      u8x8.print(dispBuffer[5]);
      u8x8.print("/");
      u8x8.println(dispBuffer[0]);
      u8x8.print("Int:   ");
      u8x8.println(sendInterval[sendIntervalKey]);
      u8x8.print("Speed: ");
      u8x8.println(dispBuffer[1]);
      u8x8.print("Alt:   ");
      u8x8.println(dispBuffer[2]);
      u8x8.print("Lora:  ");
      u8x8.println(LoraStatus);
      u8x8.print("Bat:   ");
      u8x8.println(vbat, 2);
    } else {
      u8x8.println("NO FIX");
      u8x8.print("Sat:   ");
      u8x8.print(dispBuffer[5]);
      u8x8.print("/");
      u8x8.println(dispBuffer[0]);
      u8x8.print("Int:   ");
      u8x8.println(sendInterval[sendIntervalKey]);
      u8x8.print("Bat:   ");
      u8x8.println(vbat, 2);
    }
    delay(50);
  }
}

void setup() {
  // Setup Hardware
  pinMode(BAT_PIN, INPUT);
  
  // Wifi and BT Off
  WiFi.mode(WIFI_OFF);
  btStop();

  // Init GPS
  gps.init();

  // Start UI-Thread on Second Core
  xTaskCreatePinnedToCore(uiThread, "uiThread", 10000, NULL, 1, &uiThreadTask, 0); 

  // Builtin LED will be used to indicate LoRa Activity
  pinMode(BUILTIN_LED, OUTPUT);

  // LoRa Init
  os_init();
  LMIC_reset();
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Setup EU Channels
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Disable Data Rate Adaptation, for Mapping we want static SF7
  LMIC_setAdrMode(0);
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14); 
  // Don't do Link Checks
  LMIC_setLinkCheckMode(0);

  do_send(&sendjob);
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  vbat = (float)(analogRead(BAT_PIN)) / 4095*2*3.3*1.1;
  
  hasFix = gps.checkGpsFix();
  gps.gdisplay(dispBuffer);
  os_runloop_once();
  yield();
}