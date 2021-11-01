
// #define CFG_eu868 1
// #define CFG_sx1276_radio 1

#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>

#include "credentials.h"

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Buffer with data to send to TTNMapper
uint8_t txBuffer[9];

void build_packet()
{
  String toLog = "test-1234";
  // txBuffer = toLog.StringToCharArray(txBuffer, 9);
  for (int i = 0; i < toLog.length(); i++)
  {
    txBuffer[i] = uint8_t(toLog[i]);
  }
  Serial.println(toLog);
}

// NO CHANGES BELOW THIS LINE

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/15, /* data=*/4, /* reset=*/16);

void os_getArtEui(u1_t *buf) { memcpy(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy(buf, APPKEY, 16); }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    //.dio = {26, 33, 32}, // HELTEC V1
    .dio = {26, 34, 35}, // HELTEC V2
};

String logs[7];
void write_log(String log) {
  Serial.println(log);

  for (int i  = 0; i < 6; i++) {
    logs[i] = logs[i+1]; 
  }

  logs[6] = log;

  u8x8.clearDisplay();

  for (int i  = 0; i < 6; i++) {
    char row[16];
    logs[i].toCharArray(row, 16);
    u8x8.drawString(0, i+1, row);
  }
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    build_packet();
    //LMIC_setTxData2(1, (uint8_t*) coords, sizeof(coords), 0);
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
    Serial.println(F("Packet queued"));
    write_log("Packet queued");

    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
  switch (ev)
  {
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
    write_log("EV_JOINING");
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    write_log("EV_JOINED");
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    //break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    write_log("EV_TXCOMPLETE");
    digitalWrite(BUILTIN_LED, LOW);
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      Serial.println(F("Received ack"));
    }
    if (LMIC.dataLen)
    {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
  default:
    Serial.println(F("Unknown event"));
    Serial.print("ev: ");
    Serial.println(ev);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  btStop();

  // start display
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  write_log("LoRa Receiver");


  SPI.begin(5, 19, 27);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

void loop()
{
  os_runloop_once();
}
