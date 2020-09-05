#include <Arduino.h>
#include <WiFi.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SimpleButton.h>
#include <U8x8lib.h>
#include <gps.h>

// EDIT AND INSERT YOUR ABP KEYS HERE!
#include <config.h>

#ifdef V1_1
  // Battery management
  #include "axp20x.h"
  #ifndef AXP192_SLAVE_ADDRESS
    #define AXP192_SLAVE_ADDRESS    0x34
  #endif

  #define PMU_IRQ             35

  AXP20X_Class axp;
  bool pmu_irq = false;
  bool axp192_found = true;
  String baChStatus = "No charging";
#endif

// if not set in config.h defaults to 1
#ifndef TTN_PORT
  #define TTN_PORT 1
#endif

// keeps the millis since last "QUEUED" state
uint32_t lastsendjob = 0;

// Thread Handle for UI
TaskHandle_t uiThreadTask;

// Delay between Lora Send
uint8_t sendInterval[] = {20, 30, 40, 10};
uint8_t sendIntervalKey = 0;

// number of packets send to TTN
uint16_t packets_send = 0;

// last packet number !doesn't survive a reset!
u4_t lmicSeqNumber = 0;

// flag to clear display only if was inverse before
bool display_is_inverse = false;

// TinyGPS Wrapper
Gps gps;

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

  // last packet count
  lmicSeqNumber = LMIC.seqnoUp;

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
      packets_send ++;
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
    if (hasFix)
    {
      // Prepare upstream data transmission at the next possible time.
      if(gps.buildPacket(loraBuffer)){

        LMIC_setTxData2(TTN_PORT, loraBuffer, sizeof(loraBuffer), 0);
        digitalWrite(BUILTIN_LED, HIGH);
        LoraStatus = "QUEUED";

        //keep time for timeout
        lastsendjob = millis();
        
      } else {
        LoraStatus = "GPS FAIL";
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
      }
      
    }
    else
    {
      LoraStatus = "NO FIX";
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
  u8x8.begin( );
  u8x8.setPowerSave(0);

#ifdef V1_1

  /* Init AXP192 to get access to Battery management.
     We have to do that here because we have to use the Wire instance which
     is already in use by OLED and not accessible outside of this Thread.
     There are a few flags below which I have commented out. I don't know
     whether these are relly needed.
     The code is almost from https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam/blob/master/src/LilyGO-T-Beam.ino
     reduced to a minimum.
  */
  axp.begin(Wire, AXP192_SLAVE_ADDRESS);

  // if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
  //     Serial.println("AXP192 Begin PASS");
  // } else {
  //     Serial.println("AXP192 Begin FAIL");
  // }

  // Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  // Serial.println("----------------------------------------");

  // axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  // axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  // axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  // axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  // axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  // axp.setDCDC1Voltage(3300);  //esp32 core VDD    3v3
  // axp.setLDO2Voltage(3300);   //LORA VDD set 3v3
  // axp.setLDO3Voltage(3300);   //GPS VDD      3v3

  // Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
  // Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

  // setting the interrupt to handle state changes on AXP
  pinMode(PMU_IRQ, INPUT_PULLUP);
  attachInterrupt(PMU_IRQ, [] {
    pmu_irq = true;
  }, FALLING);

  axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
  axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
  axp.clearIRQ();

  // set initial charge state
  if (axp.isChargeing()) {
      baChStatus = "Charging";
  }
#endif

  // Endless Loop for Display Thread
  bool lastState = hasFix;
  for(;;) {

#ifdef V1_1
    // interrupt is triggered if charge state changes (cable plugged/ unplugged)
    if (axp192_found && pmu_irq) {
      pmu_irq = false;
      axp.readIRQ();
      if (axp.isChargingIRQ()) {
          baChStatus = "Charging";
      } else {
          baChStatus = "No Charging";
      }
      if (axp.isVbusRemoveIRQ()) {
          baChStatus = "No Charging";
      }
      digitalWrite(2, !digitalRead(2));
      axp.clearIRQ();
    }
    // set Bat Voltage for legacy code
    vbat = axp.getBattVoltage() / 1000.0;
#endif

    b1->update();
    //u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    u8x8.setFont(u8x8_font_victoriabold8_r); // <- this font is better readable IMHO

    if (lastState != hasFix) {
      lastState = hasFix;
      u8x8.clear();
    } else {
      u8x8.home();
    }

    if (hasFix) {
      if(LoraStatus == "QUEUED"){
        u8x8.setInverseFont(1);
        display_is_inverse = true;
      }else{
        u8x8.setInverseFont(0);
        // this is to avoid white spaces on display
        if(display_is_inverse){
          u8x8.clear();
          display_is_inverse = false;
        }
      }
      u8x8.print("HDOP:  ");
      u8x8.println(dispBuffer[4] / 10.0);
      u8x8.print("Sat:   ");
      u8x8.print(dispBuffer[5]);
      u8x8.print("/");
      u8x8.println(dispBuffer[0]);
      u8x8.print("Int:   ");
      u8x8.print(sendInterval[sendIntervalKey]);
      u8x8.print("  ");
      u8x8.print(packets_send);
      u8x8.println("x");
      u8x8.print("Speed: ");
      u8x8.print(dispBuffer[1]);
      u8x8.println("km/h");
      u8x8.print("Alt:   ");
      u8x8.print(dispBuffer[3]);
      u8x8.println("m");
      u8x8.print("Lora:  ");
      u8x8.println(LoraStatus);
      u8x8.print("Bat:   ");
      u8x8.print(vbat, 2);
      u8x8.println("V");
#ifdef V1_1
      // Charge status
      u8x8.println(baChStatus);
#endif
    } else {
      u8x8.setInverseFont(0);
      u8x8.println("NO FIX");
      u8x8.print("Sat:   ");
      u8x8.print(dispBuffer[5]);
      u8x8.print("/");
      u8x8.println(dispBuffer[0]);
      u8x8.print("Int:   ");
      u8x8.print(sendInterval[sendIntervalKey]);
      u8x8.print("  ");
      u8x8.print(packets_send);
      u8x8.println("x");
      u8x8.println("");
      u8x8.println("");
      u8x8.println("");
      u8x8.print("Bat:   ");
      u8x8.print(vbat, 2);
      u8x8.println("V");
#ifdef V1_1
      // Charge status
      u8x8.println(baChStatus);
#endif
    }
    delay(50);
  }
}

void init_LMIC(){
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

  // set last packet count after re-init
  LMIC.seqnoUp = lmicSeqNumber;

  delay(500);
  do_send(&sendjob);
}

void setup() {

  Serial.begin(115200);

  // Setup Hardware
#ifdef V1_0
  pinMode(BAT_PIN, INPUT);
#endif

  // Init GPS
  gps.init();

  // Wifi and BT Off
  WiFi.mode(WIFI_OFF);
  btStop();

  // Start UI-Thread on Second Core
  xTaskCreatePinnedToCore(uiThread, "uiThread", 10000, NULL, 1, &uiThreadTask, 0); 

  // Builtin LED will be used to indicate LoRa Activity
  pinMode(BUILTIN_LED, OUTPUT);

  // LoRa Init
  os_init();
  init_LMIC();

  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {

//Bord Rev1.1 cannot measure BAT this way. Measure is done in uiThread
#ifdef V1_0
  vbat = (float)(analogRead(BAT_PIN)) / 4095*2*3.3*1.1;
#endif

  // check whether LMIC hangs and re-init
  if(LoraStatus == "QUEUED" && lastsendjob > 0 && (millis() - lastsendjob > 10000) ){
    init_LMIC();
    packets_send = 0;
    LoraStatus = "LMIC_RST";
  }

  hasFix = gps.checkGpsFix();
  gps.gdisplay(dispBuffer);
  os_runloop_once();
  yield();
}