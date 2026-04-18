// SPDX-License-Identifier: MIT
// Copyright (c) 2026 netstage.io
// OneFader Firmware — Ethernet only

#include <ETH.h>
#include <WiFiUdp.h>
#include <sACN.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <SPI.h>
#include <USB.h>
#include <USBCDC.h>
#include "mbedtls/md.h"

// Native USB-C port on ESP32-S3 uses USB CDC, not UART
USBCDC USBSerial;

// W5500 Ethernetx pins for Waveshare ESP32-S3-ETH
#define ETH_SPI_HOST    SPI3_HOST
#define ETH_SPI_SCK     13
#define ETH_SPI_MISO    12
#define ETH_SPI_MOSI    11
#define ETH_SPI_CS      14
#define ETH_SPI_INT     10
#define ETH_SPI_RST     9

// Persistent settings storage
Preferences preferences;

// Serial number based on MAC address
String deviceSerial;

// Firmware version
String firmwareVersion = "v1.7.0";
String otaPassword = "netstage";
String webUsername = "admin";
String webPassword = "netstage";
bool webAuthEnabled = false;

// Network state
bool ethConnected = false;

// Network settings
bool useDHCP = true;
IPAddress staticIP(192, 168, 1, 100);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 1);
IPAddress dns(8, 8, 8, 8);

// sACN settings
uint8_t myDeviceCID[16];
String myDeviceName = "OneFader";
uint16_t universe = 1;
uint16_t dmxStartAddress = 1;
uint8_t sacnPriority = 100;
uint8_t dmxData[512] = {0};
bool use16bit = false;

// DMX output range limits (applied after calibration and invert)
uint8_t dmxOutMin = 0;
uint8_t dmxOutMax = 255;

// Per-address priority (0xDD start code)
bool perAddressPriority = true;
uint8_t ddData[512] = {0};

// sACN output enable
bool sacnEnabled = true;

// Unicast settings
bool useUnicast = false;
IPAddress unicastTarget(0, 0, 0, 0);

// OSC output settings
// Format: 0=float 0.0-1.0, 1=int 0-100, 2=int 0-255, 3=int 0-65535
bool oscEnabled = false;
IPAddress oscTargetIP(0, 0, 0, 0);
uint16_t oscPort = 8000;
String oscAddress = "/fader/1";
uint8_t oscFormat = 0;
uint16_t oscInterval = 25; // ms between OSC sends (25=40Hz, 100=10Hz, 1000=1Hz)
unsigned long lastOSCSend = 0;
WiFiUDP oscUdp;

// Sig-Net (Singularity UK) output settings
// Protocol: CoAP/UDP multicast to 239.254.0.X:5683 with HMAC-SHA256 auth
bool signetEnabled = false;
uint16_t signetUniverse = 1;
uint16_t signetDMXAddress = 1;
String signetPassphrase = "";
bool signetUseUnicast = false;
IPAddress signetUnicastTarget(0, 0, 0, 0);
uint16_t signetInterval = 25; // ms (40Hz default)

String signetScope = "local";    // URI scope: /sig-net/v1/<scope>/level/{universe}
uint16_t signetMfgCode = 0x0000; // ESTA Manufacturer Code (2 bytes, big-endian)

// Sig-Net runtime state
WiFiUDP signetUdp;
uint32_t signetSessionID = 1; // persisted in NVS, incremented each boot
uint32_t signetSequence = 0;  // strictly incrementing per session
unsigned long lastSigNetSend = 0;
uint8_t signetSenderKey[32];  // derived HMAC key (from PBKDF2)
volatile bool signetKeyReady = false;

// Fader input pin for ESP32-S3
const int FADER_PIN = 1;
const int ADC_MAX   = 4095;

// Smoothing filter — moving average (pre-fill buffer on boot)
const int SMOOTHING_SAMPLES = 16;
int smoothingBuffer[SMOOTHING_SAMPLES];
int smoothingIndex = 0;
long smoothingTotal = 0;
int lastMovingAvg = 0;

// EMA (Exponential Moving Average) smoothing
// Alpha 0-100: higher = more responsive, lower = smoother
// Stored as integer 0-100, applied as alpha/100.0
bool emaEnabled = true;
uint8_t emaAlpha = 15;  // default: fairly smooth, still responsive
float emaValue = 0.0f;
bool emaInitialised = false;

// Deadband — prevents tiny jitter from generating output changes
int deadband = 30; // ADC counts — increase to reduce noise, decrease for finer response
int lastSmoothedValue = 0;

// Spike rejection — discard single-sample jumps larger than this ADC threshold
// Set to 0 to disable. 200 = ~5% of full range.
uint16_t spikeThreshold = 200;

// ADC calibration — learned from fader calibration routine
int adcCalMin = 10;
int adcCalMax = 4085;

// Calibration state
bool calibrating = false;
int calibLiveMin = 4095;
int calibLiveMax = 0;
unsigned long calibStartTime = 0;
const unsigned long CAL_TIMEOUT_MS = 2 * 60 * 1000; // 2 minutes

// Invert option
bool invertFader = false;

// Lustr snap — detects rapid downward fader movement and forces DMX output to 0
// ETC Lustr lights internally fade if they don't see a clean snap to 0; this helps.
bool lustrSnap = false;
bool lustrSnapActive = false;  // currently holding output at 0
uint8_t lustrSnapPrevDMX = 0; // previous coarse DMX value for velocity tracking

// Simulation mode
bool simulationMode = false;
uint16_t simulatedFaderValue = 0;

// Test mode
bool testModeEnabled = false;
uint8_t testPercent = 0;       // 0-100, cycles up then down
bool testFadeDirection = true; // true = up, false = down

// Web server
AsyncWebServer server(80);
WiFiUDP udp;
Source sacn(udp);

// Serial command buffer
String serialBuffer = "";

// Debug log buffer
String debugLogs = "";
const size_t MAX_DEBUG_LOG_SIZE = 4096;

// Timing
unsigned long lastSACNSend = 0;
const unsigned long SACN_INTERVAL = 25; // 40Hz

// Forward declarations
void addLog(String message);
void handleSerialCommand(String cmd);

// -------------------------------------------------------
// Ethernet event handlers (ETH-native, no WiFi.h required)
// -------------------------------------------------------
void onEthStart(arduino_event_id_t event) {
  addLog("ETH Started");
}

void onEthConnected(arduino_event_id_t event) {
  addLog("ETH Connected");
}

void onEthGotIP(arduino_event_id_t event) {
  addLog("ETH IP: " + ETH.localIP().toString());
  ethConnected = true;
}

void onEthDisconnected(arduino_event_id_t event) {
  addLog("ETH Disconnected");
  ethConnected = false;
}

void onEthStop(arduino_event_id_t event) {
  addLog("ETH Stopped");
  ethConnected = false;
}

// -------------------------------------------------------
// Logging
// -------------------------------------------------------
void addLog(String message) {
  String timestamp = "[" + String(millis() / 1000) + "s] ";
  debugLogs += timestamp + message + "\n";
  if (debugLogs.length() > MAX_DEBUG_LOG_SIZE) {
    debugLogs = debugLogs.substring(debugLogs.length() - MAX_DEBUG_LOG_SIZE);
  }
  USBSerial.print(timestamp);
  USBSerial.println(message);
}

// -------------------------------------------------------
// CID / Serial generation
// -------------------------------------------------------
void generateCID() {
  uint8_t mac[6];
  ETH.macAddress(mac);

  myDeviceCID[0]  = mac[0];
  myDeviceCID[1]  = mac[1];
  myDeviceCID[2]  = mac[2];
  myDeviceCID[3]  = mac[3];
  myDeviceCID[4]  = mac[4];
  myDeviceCID[5]  = mac[5];
  myDeviceCID[6]  = 0x94;
  myDeviceCID[7]  = 0x11;
  myDeviceCID[8]  = 0xE7;
  myDeviceCID[9]  = 0xBB;
  myDeviceCID[10] = 0x31;
  myDeviceCID[11] = 0xBE;
  myDeviceCID[12] = mac[0] ^ 0x2E;
  myDeviceCID[13] = mac[1] ^ 0x44;
  myDeviceCID[14] = mac[2] ^ 0xB0;
  myDeviceCID[15] = mac[3] ^ 0x6B;

  char cidStr[64];
  snprintf(cidStr, sizeof(cidStr),
           "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
           myDeviceCID[0],  myDeviceCID[1],  myDeviceCID[2],  myDeviceCID[3],
           myDeviceCID[4],  myDeviceCID[5],  myDeviceCID[6],  myDeviceCID[7],
           myDeviceCID[8],  myDeviceCID[9],  myDeviceCID[10], myDeviceCID[11],
           myDeviceCID[12], myDeviceCID[13], myDeviceCID[14], myDeviceCID[15]);
  addLog("CID: " + String(cidStr));
}

String generateSerial() {
  uint8_t mac[6];
  ETH.macAddress(mac);
  char serial[13];
  snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(serial);
}

// -------------------------------------------------------
// Analog / fader
// -------------------------------------------------------
int readSmoothedAnalog() {
  int newReading = analogRead(FADER_PIN);

  // Step 0: 3-sample median filter — eliminates single-sample ADC spikes without
  // delaying real fader movement. The median of 3 consecutive samples discards any
  // one outlier cleanly, even if spikeThreshold is set too high or disabled.
  {
    static int medBuf[3] = {0, 0, 0};
    static uint8_t medIdx = 0;
    static bool medInit = false;
    if (!medInit) { medBuf[0] = medBuf[1] = medBuf[2] = newReading; medInit = true; }
    medBuf[medIdx] = newReading;
    medIdx = (medIdx + 1) % 3;
    int a = medBuf[0], b = medBuf[1], c = medBuf[2];
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }
    newReading = b; // median
  }

  // Step 1: spike rejection
  // If a reading jumps beyond the threshold it enters a confirmation window.
  // The candidate target is tracked — if subsequent readings stay within
  // 150 counts of that candidate for 5 consecutive samples it is accepted
  // as a real move (snap). If readings are scattered it is noise and
  // the last good value is held.
  static bool spikeInitialised = false;
  static int lastRaw = 0;
  static int spikeCount = 0;
  static int spikeCandidate = 0;
  if (!spikeInitialised) {
    lastRaw = newReading;
    spikeInitialised = true;
  }
  // If calibration is active, track the live min/max
  if (calibrating) {
    if (newReading < calibLiveMin) calibLiveMin = newReading;
    if (newReading > calibLiveMax) calibLiveMax = newReading;
  }

  if (spikeThreshold > 0 && abs(newReading - lastRaw) > spikeThreshold) {
    if (spikeCount == 0) {
      // First sample over threshold — record candidate target
      spikeCandidate = newReading;
      spikeCount = 1;
    } else if (abs(newReading - spikeCandidate) <= 150) {
      // Reading is consistent with candidate — increment confidence
      spikeCount++;
      if (spikeCount >= 5) {
        // Confirmed real move — accept and reset
        lastRaw = newReading;
        spikeCount = 0;
      }
    } else {
      // Reading is scattered — reset, treat as noise
      spikeCount = 0;
    }
    newReading = lastRaw; // hold until confirmed
  } else {
    lastRaw = newReading;
    spikeCount = 0;
  }

  // Step 2: moving average over 16 samples
  smoothingTotal -= smoothingBuffer[smoothingIndex];
  smoothingBuffer[smoothingIndex] = newReading;
  smoothingTotal += newReading;
  smoothingIndex = (smoothingIndex + 1) % SMOOTHING_SAMPLES;
  int averaged = smoothingTotal / SMOOTHING_SAMPLES;

  // Step 3: optional EMA on top of moving average
  int filtered;
  if (emaEnabled) {
    if (!emaInitialised) {
      emaValue = (float)averaged;
      emaInitialised = true;
    }
    float alpha = emaAlpha / 100.0f;
    emaValue = alpha * (float)averaged + (1.0f - alpha) * emaValue;
    filtered = (int)emaValue;
  } else {
    filtered = averaged;
  }

  // Step 4: deadband — only update output if change exceeds threshold
  if (abs(filtered - lastSmoothedValue) > deadband) {
    lastSmoothedValue = filtered;
  }
  return lastSmoothedValue;
}

uint16_t readFaderValue16bit() {
  uint16_t raw16;
  if (simulationMode) {
    raw16 = (uint16_t)map(simulatedFaderValue, 0, 100, 0, 65535);
  } else {
    int adc = constrain(readSmoothedAnalog(), adcCalMin, adcCalMax);
    raw16 = (uint16_t)min((long)65535, map(adc, adcCalMin, adcCalMax, 0, 65536));
  }
  // Endpoint snap — when the fader is within ~1% of either end, force it to exactly
  // 0 or 65535. This overcomes the final few ADC counts of deadband/smoothing lag
  // that would otherwise prevent the output from reaching true 0 or full.
  if (raw16 < 655) raw16 = 0;
  else if (raw16 > 64880) raw16 = 65535;

  // Apply invert, then scale into output range limits.
  // 257 * 255 = 65535, so dmxOutMin/Max (0-255) map cleanly to 16-bit.
  if (invertFader) raw16 = 65535 - raw16;
  uint16_t outMin16 = (uint16_t)((uint32_t)dmxOutMin * 257);
  uint16_t outMax16 = (uint16_t)((uint32_t)dmxOutMax * 257);
  return (uint16_t)map((long)raw16, 0, 65535, (long)outMin16, (long)outMax16);
}

uint8_t readFaderValue() {
  // All processing (calibration, invert, range) is in readFaderValue16bit().
  return (uint8_t)(readFaderValue16bit() >> 8);
}

void resetDMXOutput() {
  memset(dmxData, 0, sizeof(dmxData));
  addLog("DMX output reset to 0");
}

// -------------------------------------------------------
// Network settings
// -------------------------------------------------------
void saveNetworkSettings() {
  preferences.putBool("useDHCP",   useDHCP);
  preferences.putUInt("staticIP",  (uint32_t)staticIP);
  preferences.putUInt("subnet",    (uint32_t)subnet);
  preferences.putUInt("gateway",   (uint32_t)gateway);
  preferences.putUInt("dns",       (uint32_t)dns);
}

void loadNetworkSettings() {
  useDHCP   = preferences.getBool("useDHCP", true);
  staticIP  = IPAddress(preferences.getUInt("staticIP", (uint32_t)IPAddress(192, 168, 1, 100)));
  subnet    = IPAddress(preferences.getUInt("subnet",   (uint32_t)IPAddress(255, 255, 255, 0)));
  gateway   = IPAddress(preferences.getUInt("gateway",  (uint32_t)IPAddress(192, 168, 1, 1)));
  dns       = IPAddress(preferences.getUInt("dns",      (uint32_t)IPAddress(8, 8, 8, 8)));
}

void applyEthernetSettings() {
  if (!useDHCP) {
    ETH.config(staticIP, gateway, subnet, dns);
    addLog("Static IP: " + staticIP.toString());
  } else {
    addLog("Ethernet using DHCP");
  }
}

String getCurrentIP() {
  return ethConnected ? ETH.localIP().toString() : "Not Connected";
}

String getNetworkStatus() {
  if (!ethConnected) return "Not Connected";
  String dhcpInfo = useDHCP ? "DHCP" : "Static IP";
  return "Ethernet: " + ETH.localIP().toString() +
         "<br><span style=\"font-size:0.78rem;opacity:0.8;font-weight:400;\">" + dhcpInfo +
         " &nbsp;|&nbsp; GW: " + ETH.gatewayIP().toString() + "</span>";
}

// -------------------------------------------------------
// Serial command handler (USB-C tool)
// -------------------------------------------------------
void handleSerialCommand(String cmd) {
  cmd.trim();
  addLog("Serial CMD: " + cmd);

  if (cmd == "FACTORY_RESET") {
    USBSerial.println("[ONEFADER] Factory reset triggered via USB");
    preferences.clear();
    delay(500);
    ESP.restart();

  } else if (cmd == "REBOOT") {
    USBSerial.println("[ONEFADER] Reboot triggered via USB");
    delay(500);
    ESP.restart();

  } else if (cmd == "STATUS") {
    USBSerial.println("[ONEFADER] STATUS");
    USBSerial.println("[ONEFADER] FW:" + firmwareVersion);
    USBSerial.println("[ONEFADER] NAME:" + myDeviceName);
    USBSerial.println("[ONEFADER] UNIVERSE:" + String(universe));
    USBSerial.println("[ONEFADER] ADDR:" + String(dmxStartAddress));
    USBSerial.println("[ONEFADER] IP:" + getCurrentIP());
    USBSerial.println("[ONEFADER] DHCP:" + String(useDHCP ? "1" : "0"));
    USBSerial.println("[ONEFADER] STATIC_IP:" + staticIP.toString());
    USBSerial.println("[ONEFADER] GATEWAY:" + gateway.toString());
    USBSerial.println("[ONEFADER] SUBNET:" + subnet.toString());
    USBSerial.println("[ONEFADER] DNS:" + dns.toString());

  } else if (cmd.startsWith("SET_IP ")) {
    String params = cmd.substring(7);
    int c1 = params.indexOf(',');
    int c2 = params.indexOf(',', c1 + 1);
    int c3 = params.indexOf(',', c2 + 1);

    if (c1 < 0 || c2 < 0 || c3 < 0) {
      USBSerial.println("[ONEFADER] ERROR: Bad SET_IP format");
      return;
    }

    String sIP  = params.substring(0, c1);
    String sGW  = params.substring(c1 + 1, c2);
    String sSN  = params.substring(c2 + 1, c3);
    String sDNS = params.substring(c3 + 1);

    bool changed = false;
    if (sIP  != "_" && staticIP.fromString(sIP))  { changed = true; }
    if (sGW  != "_" && gateway.fromString(sGW))    { changed = true; }
    if (sSN  != "_" && subnet.fromString(sSN))     { changed = true; }
    if (sDNS != "_" && dns.fromString(sDNS))       { changed = true; }

    if (changed) {
      useDHCP = false;
      saveNetworkSettings();
      USBSerial.println("[ONEFADER] IP config saved. Rebooting...");
      delay(500);
      ESP.restart();
    } else {
      USBSerial.println("[ONEFADER] ERROR: No valid IP values parsed");
    }

  } else if (cmd == "LOG_START") {
    USBSerial.println("[ONEFADER] Log streaming active");

  } else if (cmd == "LOG_STOP") {
    USBSerial.println("[ONEFADER] Log streaming paused");

  } else {
    USBSerial.println("[ONEFADER] Unknown command: " + cmd);
  }
}

// -------------------------------------------------------
// sACN begin helper — applies unicast or multicast depending on setting
// -------------------------------------------------------
void sacnBegin() {
  if (useUnicast && unicastTarget != IPAddress(0, 0, 0, 0)) {
    sacn.begin(unicastTarget, universe, sacnPriority, perAddressPriority);
    addLog("sACN Unicast -> " + unicastTarget.toString());
  } else {
    sacn.begin(universe, sacnPriority, perAddressPriority);
    addLog("sACN Multicast");
  }
}

// Build per-address priority buffer — only active channel(s) get sacnPriority,
// everything else is 0 (this source does not claim those channels).
void updateDDData() {
  memset(ddData, 0, sizeof(ddData));
  ddData[dmxStartAddress - 1] = sacnPriority;
  if (use16bit && dmxStartAddress <= 511) {
    ddData[dmxStartAddress] = sacnPriority;
  }
  sacn.dd(ddData);
}

// Send current DMX value as an OSC message. Reads from dmxData so the output
// always mirrors exactly what sACN is sending (including test mode sweeps).
// OSC spec: all fields big-endian, string fields null-terminated and padded to
// 4-byte boundaries. No external library needed — packets are built inline.
void sendOSC() {
  if (!oscEnabled || !ethConnected) return;
  if (oscTargetIP == IPAddress(0, 0, 0, 0)) return;

  // Reconstruct 16-bit value from dmxData
  uint16_t raw16;
  if (use16bit && dmxStartAddress <= 511) {
    raw16 = ((uint16_t)dmxData[dmxStartAddress - 1] << 8) | dmxData[dmxStartAddress];
  } else {
    raw16 = (uint16_t)dmxData[dmxStartAddress - 1] * 257; // 0-255 → 0-65535
  }

  uint8_t buf[128];
  memset(buf, 0, sizeof(buf));
  int offset = 0;

  // Address string: copy bytes, add null, pad to 4-byte boundary
  int addrLen = (int)oscAddress.length();
  if (addrLen > 60) addrLen = 60;
  memcpy(buf + offset, oscAddress.c_str(), addrLen);
  offset += addrLen + 1;            // +1 for null terminator
  offset = (offset + 3) & ~3;       // round up to multiple of 4

  // Type tag string: ",f\0\0" (float) or ",i\0\0" (int) — always 4 bytes
  buf[offset++] = ',';
  buf[offset++] = (oscFormat == 0) ? 'f' : 'i';
  buf[offset++] = 0;
  buf[offset++] = 0;

  // Value — big-endian (required by OSC spec)
  if (oscFormat == 0) {
    float fval = raw16 / 65535.0f;
    uint32_t bits;
    memcpy(&bits, &fval, 4);
    buf[offset++] = (bits >> 24) & 0xFF;
    buf[offset++] = (bits >> 16) & 0xFF;
    buf[offset++] = (bits >>  8) & 0xFF;
    buf[offset++] =  bits        & 0xFF;
  } else {
    int32_t ival;
    switch (oscFormat) {
      case 1:  ival = (int32_t)map((long)raw16, 0, 65535, 0, 100); break;
      case 2:  ival = (int32_t)(raw16 >> 8); break;   // 0-255
      case 3:  ival = (int32_t)raw16; break;           // 0-65535
      default: ival = 0; break;
    }
    buf[offset++] = (ival >> 24) & 0xFF;
    buf[offset++] = (ival >> 16) & 0xFF;
    buf[offset++] = (ival >>  8) & 0xFF;
    buf[offset++] =  ival        & 0xFF;
  }

  oscUdp.beginPacket(oscTargetIP, oscPort);
  oscUdp.write(buf, offset);
  oscUdp.endPacket();
}

// -------------------------------------------------------
// Sig-Net (Singularity UK) output
// Protocol: CoAP NON-POST over UDP with HMAC-SHA256 option auth
// Spec: https://singularity-uk.com Sig-Net Protocol Framework
// -------------------------------------------------------

// Encode a single CoAP option (delta-encoded, ascending order required).
// buf: packet buffer; offset: current write position; optNum: this option number;
// prevOptNum: last option number written; value/len: option payload.
// Returns new offset.
static int signetEncodeOption(uint8_t* buf, int offset, uint16_t optNum,
                              uint16_t prevOptNum, const uint8_t* value, uint16_t len) {
  uint16_t delta = optNum - prevOptNum;
  uint8_t d4, l4;
  uint8_t edBuf[2] = {0, 0};
  uint8_t elBuf[2] = {0, 0};
  int edLen = 0, elLen = 0;

  if (delta < 13) { d4 = (uint8_t)delta; }
  else if (delta < 269) { d4 = 13; edBuf[0] = (uint8_t)(delta - 13); edLen = 1; }
  else { d4 = 14; uint16_t v = delta - 269; edBuf[0] = v >> 8; edBuf[1] = v & 0xFF; edLen = 2; }

  if (len < 13) { l4 = (uint8_t)len; }
  else if (len < 269) { l4 = 13; elBuf[0] = (uint8_t)(len - 13); elLen = 1; }
  else { l4 = 14; uint16_t v = len - 269; elBuf[0] = v >> 8; elBuf[1] = v & 0xFF; elLen = 2; }

  buf[offset++] = (d4 << 4) | l4;
  for (int i = 0; i < edLen; i++) buf[offset++] = edBuf[i];
  for (int i = 0; i < elLen; i++) buf[offset++] = elBuf[i];
  if (value && len > 0) { memcpy(buf + offset, value, len); offset += len; }
  return offset;
}

// PBKDF2-HMAC-SHA256: single 32-byte output block (RFC 2898 §5.2, block index 1).
// Uses the mbedtls context API directly so the HMAC key schedule is set up once
// and reused for all 100,000 U_i computations — no per-iteration allocation.
static void pbkdf2HmacSha256(const uint8_t* pw, size_t pwLen,
                              const uint8_t* salt, size_t saltLen,
                              uint32_t iters, uint8_t* dk) {
  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, md, 1) != 0) {   // 1 = HMAC mode
    mbedtls_md_free(&ctx);
    return;
  }

  // Build salt || INT(1) — block counter 1, big-endian 32-bit
  uint8_t saltBlock[64]; // salt is 18 bytes; 64 is ample
  memcpy(saltBlock, salt, saltLen);
  saltBlock[saltLen]   = 0x00;
  saltBlock[saltLen+1] = 0x00;
  saltBlock[saltLen+2] = 0x00;
  saltBlock[saltLen+3] = 0x01;

  uint8_t U[32], T[32];

  // U1 = HMAC(pw, salt || INT(1))
  mbedtls_md_hmac_starts(&ctx, pw, pwLen);
  mbedtls_md_hmac_update(&ctx, saltBlock, saltLen + 4);
  mbedtls_md_hmac_finish(&ctx, U);
  memcpy(T, U, 32);

  // U2..Uc = HMAC(pw, U_{i-1}), each XOR'd into T
  for (uint32_t i = 1; i < iters; i++) {
    uint8_t Unext[32];
    mbedtls_md_hmac_starts(&ctx, pw, pwLen);
    mbedtls_md_hmac_update(&ctx, U, 32);
    mbedtls_md_hmac_finish(&ctx, Unext);
    memcpy(U, Unext, 32);
    for (int j = 0; j < 32; j++) T[j] ^= U[j];
  }

  mbedtls_md_free(&ctx);
  memcpy(dk, T, 32);
}

// FreeRTOS task: derives Sig-Net sender key in background so loop() isn't blocked.
// K0 = PBKDF2-HMAC-SHA256(passphrase, "Sig-Net-K0-Salt-v1", 100000, 32)
// SenderKey = HMAC-SHA256(K0, "Sig-Net-Sender-v1" || 0x01)
static void signetKeyTask(void* /*unused*/) {
  // Log passphrase length, first/last chars, and a hex dump of the first 8 bytes.
  // Helps detect hidden characters, smart quotes, or wrong passphrase without fully exposing it.
  {
    int ppLen = (int)signetPassphrase.length();
    int dumpLen = (ppLen < 8) ? ppLen : 8;
    char hexBuf[17]; // 8 bytes * 2 chars + NUL
    for (int i = 0; i < dumpLen; i++) {
      snprintf(hexBuf + i*2, 3, "%02x", (uint8_t)signetPassphrase[i]);
    }
    hexBuf[dumpLen * 2] = '\0';
    char ppInfo[64];
    if (ppLen >= 2) {
      snprintf(ppInfo, sizeof(ppInfo), "len=%d first='%c' last='%c' hex[0..%d]=%s",
               ppLen, signetPassphrase[0], signetPassphrase[ppLen - 1], dumpLen - 1, hexBuf);
    } else {
      snprintf(ppInfo, sizeof(ppInfo), "len=%d hex=%s", ppLen, hexBuf);
    }
    addLog("Sig-Net: deriving key (PBKDF2 100k iter) " + String(ppInfo));
  }

  static const uint8_t k0Salt[] = "Sig-Net-K0-Salt-v1"; // 18 bytes
  uint8_t k0[32];
  pbkdf2HmacSha256(
    (const uint8_t*)signetPassphrase.c_str(), signetPassphrase.length(),
    k0Salt, 18, 100000, k0);

  // Sender Key = HKDF-Expand: HMAC-SHA256(K0, "Sig-Net-Sender-v1" || 0x01)
  // Info string matches constants.js HKDF_INFO.KS = Buffer.from('Sig-Net-Sender-v1').
  // Both the one-shot and context HMAC APIs produce wrong output on ESP32 mbedTLS
  // when the key is 32 bytes (high-entropy). Implement HMAC manually with two plain
  // SHA256 calls instead: HMAC(K,M) = SHA256((K^opad) || SHA256((K^ipad) || M))
  {
    uint8_t label[18];
    memcpy(label, "Sig-Net-Sender-v1", 17);
    label[17] = 0x01;

    // Build ipad/opad keys: K0 (32 bytes) padded to 64 with zeros, XOR'd
    uint8_t k_ipad[64], k_opad[64];
    for (int i = 0; i < 64; i++) {
      uint8_t kb = (i < 32) ? k0[i] : 0;
      k_ipad[i] = kb ^ 0x36;
      k_opad[i] = kb ^ 0x5c;
    }

    // Use mbedtls MD in plain hash mode (hmac=0) — SHA256 itself works correctly
    mbedtls_md_context_t sha;
    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    mbedtls_md_init(&sha);
    if (mbedtls_md_setup(&sha, md_info, 0) == 0) {  // 0 = plain hash, not HMAC
      // inner = SHA256(k_ipad || label)
      uint8_t inner[32];
      mbedtls_md_starts(&sha);
      mbedtls_md_update(&sha, k_ipad, 64);
      mbedtls_md_update(&sha, label, 18);
      mbedtls_md_finish(&sha, inner);

      // Ks = SHA256(k_opad || inner)
      mbedtls_md_starts(&sha);
      mbedtls_md_update(&sha, k_opad, 64);
      mbedtls_md_update(&sha, inner, 32);
      mbedtls_md_finish(&sha, signetSenderKey);
    }
    mbedtls_md_free(&sha);
  }

  // Log full K0 and SenderKey (64 hex chars each) for cross-checking.
  // Test vectors (confirmed from working SigNet Tester constants.js):
  //   K0 = 52fcc2e7749f40358ba00b1d557dc11861e89868e139f23014f6a0cfe59cf173
  //   Ks = 78981fe02576b2e9e47d916853d5967f34f8ae8aaae46db0495b178a75620e89
  char k0hex[65], skHex[65];
  for (int i = 0; i < 32; i++) snprintf(k0hex + i*2, 3, "%02x", k0[i]);
  for (int i = 0; i < 32; i++) snprintf(skHex + i*2, 3, "%02x", signetSenderKey[i]);
  k0hex[64] = skHex[64] = '\0';
  addLog("Sig-Net: K0=" + String(k0hex));
  addLog("Sig-Net: SK=" + String(skHex));

  // Self-test: if using the SDK test passphrase, verify K0 and Ks against known-good vectors.
  // This definitively confirms whether PBKDF2 and HKDF are both correct.
  if (signetPassphrase == "Ge2p$E$4*A") {
    static const uint8_t expectedK0[32] = {
      0x52,0xfc,0xc2,0xe7,0x74,0x9f,0x40,0x35,0x8b,0xa0,0x0b,0x1d,0x55,0x7d,0xc1,0x18,
      0x61,0xe8,0x98,0x68,0xe1,0x39,0xf2,0x30,0x14,0xf6,0xa0,0xcf,0xe5,0x9c,0xf1,0x73
    };
    static const uint8_t expectedKs[32] = {
      0x78,0x98,0x1f,0xe0,0x25,0x76,0xb2,0xe9,0xe4,0x7d,0x91,0x68,0x53,0xd5,0x96,0x7f,
      0x34,0xf8,0xae,0x8a,0xaa,0xe4,0x6d,0xb0,0x49,0x5b,0x17,0x8a,0x75,0x62,0x0e,0x89
    };
    bool k0ok = (memcmp(k0, expectedK0, 32) == 0);
    bool ksok = (memcmp(signetSenderKey, expectedKs, 32) == 0);
    addLog(String("Sig-Net: TEST K0 ") + (k0ok ? "PASS" : "FAIL (PBKDF2 wrong)"));
    addLog(String("Sig-Net: TEST Ks ") + (ksok ? "PASS" : "FAIL (HKDF wrong)"));
  }

  memset(k0, 0, sizeof(k0)); // clear root key from RAM
  signetKeyReady = true;
  addLog("Sig-Net: key ready");
  vTaskDelete(nullptr);
}

// Build and transmit a Sig-Net CoAP/UDP level packet.
// Destination: multicast 239.254.0.X:5683 (or unicast override).
// Per Sig-Net spec: level packets always use Security-Mode 0x00 (HMAC_SHA256).
// Unprovisioned beacons are a different packet type — do not send level data
// until the sender key has been derived.
void sendSigNet() {
  static uint32_t snPktCount = 0;
  static bool snFirstSend = true;
  static unsigned long snLastDiag = 0;

  if (!signetEnabled) return;
  if (!ethConnected) return;

  // Periodic diagnostic — log state every 5s so user can see why we're not sending
  unsigned long now = millis();
  if (now - snLastDiag >= 5000) {
    snLastDiag = now;
    if (!signetKeyReady && signetPassphrase.length() >= 10) {
      addLog("Sig-Net: key deriving (wait ~30s)...");
    } else if (signetPassphrase.length() < 10) {
      addLog("Sig-Net: no passphrase — set one in Output > Sig-Net");
    } else {
      // Key ready — no need to log here; packet count logged below
    }
  }

  // Do not send level data until key derivation is complete (PBKDF2 ~30s)
  if (!signetKeyReady || signetPassphrase.length() < 10) return;

  // Build 8-byte Sender-ID: [mfgCode(2 BE) | deviceId(4 BE from MAC) | endpoint(2 BE)]
  // Format matches TUID structure: constants.js _randomSenderId() / SENDER_ID option.
  uint8_t mac[6];
  ETH.macAddress(mac);
  uint8_t senderID[8];
  senderID[0] = uint8_t(signetMfgCode >> 8);
  senderID[1] = uint8_t(signetMfgCode & 0xFF);
  senderID[2] = mac[2]; senderID[3] = mac[3];
  senderID[4] = mac[4]; senderID[5] = mac[5];
  senderID[6] = 0x00; senderID[7] = 0x01;  // endpoint 1

  uint8_t mfgCode[2] = { uint8_t(signetMfgCode >> 8), uint8_t(signetMfgCode & 0xFF) };
  static const uint8_t secMode    = 0x00;           // HMAC_SHA256

  // Sequence: wrap session when sequence exhausted
  signetSequence++;
  if (signetSequence == 0) { signetSequence = 1; signetSessionID++; }

  uint8_t sessBytes[4] = {
    uint8_t(signetSessionID >> 24), uint8_t(signetSessionID >> 16),
    uint8_t(signetSessionID >>  8), uint8_t(signetSessionID)
  };
  uint8_t seqBytes[4] = {
    uint8_t(signetSequence >> 24), uint8_t(signetSequence >> 16),
    uint8_t(signetSequence >>  8), uint8_t(signetSequence)
  };

  // URI string for HMAC input: "/sig-net/v1/<scope>/level/{universe}"
  char uriStr[64];
  snprintf(uriStr, sizeof(uriStr), "/sig-net/v1/%s/level/%u", signetScope.c_str(), (unsigned)signetUniverse);

  // TLV payload: TID_LEVEL (0x0101), 512 slots, big-endian length field
  static uint8_t tlv[516];
  tlv[0] = 0x01; tlv[1] = 0x01;  // Type = TID_LEVEL (0x0101)
  tlv[2] = 0x02; tlv[3] = 0x00;  // Length = 512 (0x0200, big-endian)
  memcpy(tlv + 4, dmxData, 512);
  const uint16_t tlvLen = 516;

  // Compute HMAC-SHA256(SenderKey, URI | secMode | senderID | mfgCode | sessionID | seqNum | TLV)
  uint8_t hmacBuf[32];
  {
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, md, 1);
    mbedtls_md_hmac_starts(&ctx, signetSenderKey, 32);
    mbedtls_md_hmac_update(&ctx, (const uint8_t*)uriStr, strlen(uriStr));
    mbedtls_md_hmac_update(&ctx, &secMode, 1);
    mbedtls_md_hmac_update(&ctx, senderID, 8);
    mbedtls_md_hmac_update(&ctx, mfgCode, 2);
    mbedtls_md_hmac_update(&ctx, sessBytes, 4);
    mbedtls_md_hmac_update(&ctx, seqBytes, 4);
    mbedtls_md_hmac_update(&ctx, tlv, tlvLen);
    mbedtls_md_hmac_finish(&ctx, hmacBuf);
    mbedtls_md_free(&ctx);
  }

  // Build CoAP packet into static buffer (avoids stack pressure)
  static uint8_t pkt[700]; // 4 hdr + ~50 options + 1 marker + 516 TLV ≈ 571 bytes
  int off = 0;

  // CoAP header: Ver=1 Type=NON TKL=0 Code=POST MsgID=low16(seq)
  pkt[off++] = 0x50;                          // 0b01010000
  pkt[off++] = 0x02;                          // POST
  pkt[off++] = (signetSequence >> 8) & 0xFF;
  pkt[off++] = signetSequence & 0xFF;

  // URI-Path options (option 11): "sig-net" / "v1" / <scope> / "level" / "{universe}"
  uint16_t prevOpt = 0;
  const char* segs[] = { "sig-net", "v1", signetScope.c_str(), "level" };
  for (int i = 0; i < 4; i++) {
    off = signetEncodeOption(pkt, off, 11, prevOpt, (const uint8_t*)segs[i], strlen(segs[i]));
    prevOpt = 11;
  }
  char univStr[8];
  snprintf(univStr, sizeof(univStr), "%u", (unsigned)signetUniverse);
  off = signetEncodeOption(pkt, off, 11, prevOpt, (const uint8_t*)univStr, strlen(univStr));
  prevOpt = 11;

  // Sig-Net custom options in ascending order (per §8.3)
  off = signetEncodeOption(pkt, off, 2076, prevOpt, &secMode, 1);        prevOpt = 2076;  // Security-Mode
  off = signetEncodeOption(pkt, off, 2108, prevOpt, senderID, 8);        prevOpt = 2108;  // Sender-ID
  off = signetEncodeOption(pkt, off, 2140, prevOpt, mfgCode, 2);         prevOpt = 2140;  // Mfg-Code
  off = signetEncodeOption(pkt, off, 2172, prevOpt, sessBytes, 4);       prevOpt = 2172;  // Session-ID
  off = signetEncodeOption(pkt, off, 2204, prevOpt, seqBytes, 4);        prevOpt = 2204;  // Seq-Num
  off = signetEncodeOption(pkt, off, 2236, prevOpt, hmacBuf, 32);        prevOpt = 2236;  // HMAC

  // Payload marker + TLV
  pkt[off++] = 0xFF;
  memcpy(pkt + off, tlv, tlvLen);
  off += tlvLen;

  // Destination: multicast 239.254.0.X or unicast override
  IPAddress dest;
  if (signetUseUnicast && signetUnicastTarget != IPAddress(0, 0, 0, 0)) {
    dest = signetUnicastTarget;
  } else {
    uint8_t oct = (uint8_t)(((signetUniverse - 1) % 100) + 1);
    dest = IPAddress(239, 254, 0, oct);
  }

  int beginOk = signetUdp.beginPacket(dest, 5683);
  signetUdp.write(pkt, off);
  int endOk = signetUdp.endPacket();

  snPktCount++;
  if (snFirstSend) {
    snFirstSend = false;
    addLog("Sig-Net: first packet -> " + dest.toString() + ":5683 beginOk=" + String(beginOk) + " endOk=" + String(endOk) + " len=" + String(off));
  } else if (snPktCount % 500 == 0) {
    addLog("Sig-Net: " + String(snPktCount) + " pkts -> " + dest.toString() + " beginOk=" + String(beginOk) + " endOk=" + String(endOk));
  }
  if (!beginOk || !endOk) {
    addLog("Sig-Net: UDP send FAILED beginOk=" + String(beginOk) + " endOk=" + String(endOk));
  }
}

// -------------------------------------------------------
// Web auth helper
// -------------------------------------------------------
bool checkAuth(AsyncWebServerRequest *request) {
  if (!webAuthEnabled) return true;
  if (!request->authenticate(webUsername.c_str(), webPassword.c_str())) {
    request->requestAuthentication("OneFader");
    return false;
  }
  return true;
}

// -------------------------------------------------------
// Web server
// -------------------------------------------------------
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>%STREAM_NAME%</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box;}
    body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:#f0f2f7;min-height:100vh;display:flex;flex-direction:column;color:#333;}
    /* Sticky header */
    .hdr{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);padding:14px 16px;color:white;position:sticky;top:0;z-index:100;box-shadow:0 2px 16px rgba(0,0,0,0.2);}
    .hdr-top{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px;}
    .hdr-title{font-size:1.25rem;font-weight:700;letter-spacing:-0.3px;}
    .hdr-sub{font-size:0.7rem;opacity:0.75;margin-top:1px;}
    .badge{display:inline-block;padding:3px 10px;border-radius:10px;font-size:0.65rem;font-weight:700;text-transform:uppercase;letter-spacing:0.5px;}
    .badge-sim{background:#fbbf24;color:#78350f;}
    .badge-hw{background:#34d399;color:#065f46;}
    .fw{display:flex;align-items:center;gap:12px;}
    .fpct{font-size:1.9rem;font-weight:700;min-width:62px;text-align:right;line-height:1;}
    .fbar-wrap{flex:1;}
    .fbar{width:100%;height:9px;background:rgba(255,255,255,0.3);border-radius:5px;overflow:hidden;}
    .ffill{height:100%;background:linear-gradient(90deg,#34d399,#fbbf24,#f87171);transition:width 0.15s ease-out;border-radius:5px;}
    .fhint{font-size:0.68rem;opacity:0.75;text-align:right;margin-top:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
    /* Tabs */
    .tabs{display:flex;background:white;border-bottom:2px solid #e8e8f0;overflow-x:auto;-webkit-overflow-scrolling:touch;scrollbar-width:none;flex-shrink:0;}
    .tabs::-webkit-scrollbar{display:none;}
    .tab{flex:1;min-width:64px;padding:10px 6px 8px;text-align:center;cursor:pointer;font-size:0.65rem;font-weight:600;color:#999;border-bottom:3px solid transparent;margin-bottom:-2px;transition:color 0.2s,border-color 0.2s;user-select:none;}
    .tab .ti{font-size:1.1rem;display:block;margin-bottom:2px;}
    .tab.active{color:#667eea;border-bottom-color:#667eea;}
    /* Content */
    .content{flex:1;padding:14px;max-width:560px;width:100%;margin:0 auto;}
    .panel{display:none;}
    .panel.active{display:block;}
    #panel-output{max-width:860px;margin:0 auto;}
    /* Card */
    .card{background:white;border-radius:14px;padding:18px;box-shadow:0 2px 10px rgba(0,0,0,0.06);margin-bottom:14px;}
    .ct{font-size:1rem;font-weight:700;color:#667eea;margin-bottom:14px;display:flex;align-items:center;gap:8px;}
    /* Info rows */
    .ir{display:flex;justify-content:space-between;align-items:center;padding:9px 11px;background:#f8f9fa;border-radius:8px;margin-bottom:7px;font-size:0.83rem;}
    .il{font-weight:600;color:#666;}
    .iv{font-family:'Courier New',monospace;font-size:0.78rem;color:#333;}
    /* Forms */
    .fg{margin-bottom:12px;}
    label{display:block;margin-bottom:5px;font-weight:600;color:#555;font-size:0.82rem;}
    input[type="text"],input[type="number"],select{width:100%;padding:9px 12px;border:2px solid #e0e0e0;border-radius:9px;font-size:0.92rem;transition:border-color 0.2s;background:white;}
    input:focus,select:focus{outline:none;border-color:#667eea;box-shadow:0 0 0 3px rgba(102,126,234,0.1);}
    /* Buttons */
    .btn{width:100%;padding:11px;margin-bottom:8px;border:none;border-radius:9px;font-size:0.88rem;font-weight:600;cursor:pointer;transition:all 0.2s;}
    .bp{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;box-shadow:0 3px 10px rgba(102,126,234,0.3);}
    .bp:hover{transform:translateY(-1px);box-shadow:0 5px 14px rgba(102,126,234,0.4);}
    .bs{background:#f1f3f5;color:#555;}
    .bs:hover{background:#e4e6ea;}
    .bd{background:linear-gradient(135deg,#f87171 0%,#dc2626 100%);color:white;}
    .it{font-size:0.77rem;color:#999;margin:-2px 0 10px;line-height:1.4;padding:7px 10px;background:#f8f9fa;border-radius:7px;}
    /* Network status */
    .ns{background:linear-gradient(135deg,#34d399 0%,#059669 100%);color:white;padding:11px 14px;border-radius:9px;margin-bottom:12px;font-weight:500;font-size:0.85rem;text-align:center;}
    /* Slider */
    input[type="range"]{width:100%;height:6px;border-radius:3px;background:#e0e0e0;outline:none;-webkit-appearance:none;margin:10px 0;border:none;padding:0;}
    input[type="range"]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);cursor:pointer;box-shadow:0 2px 7px rgba(102,126,234,0.5);}
    .sv{text-align:center;font-size:1rem;font-weight:700;color:#667eea;}
    /* Logs */
    .logs{background:#1e293b;color:#e2e8f0;padding:12px;border-radius:9px;font-family:'Courier New',monospace;font-size:0.73rem;height:180px;overflow-y:auto;white-space:pre-wrap;line-height:1.5;}
    .logs::-webkit-scrollbar{width:5px;}
    .logs::-webkit-scrollbar-track{background:#334155;border-radius:3px;}
    .logs::-webkit-scrollbar-thumb{background:#667eea;border-radius:3px;}
    hr{border:none;border-top:1px solid #e8e8f0;margin:14px 0;}
    .out-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px;}
  </style>
</head>
<body>

  <div class="hdr">
    <div class="hdr-top">
      <div>
        <div class="hdr-title">OneFader</div>
        <div class="hdr-sub">%STREAM_NAME% &nbsp;&middot;&nbsp; <a href="https://netstage.io" target="_blank" style="color:rgba(255,255,255,0.7);text-decoration:none;">netstage.io</a></div>
      </div>
      <span class="badge badge-hw" id="modeBadge">HARDWARE</span>
    </div>
    <div class="fw">
      <div class="fpct" id="faderPct">0%</div>
      <div class="fbar-wrap">
        <div class="fbar"><div class="ffill" id="faderFill" style="width:0%"></div></div>
        <div class="fhint" id="netHint">Connecting...</div>
      </div>
    </div>
  </div>

  <div class="tabs">
    <div class="tab active" onclick="switchTab('output')"><span class="ti">💡</span>Output</div>
    <div class="tab" onclick="switchTab('control')"><span class="ti">🎚️</span>Control</div>
    <div class="tab" onclick="switchTab('network')"><span class="ti">🌐</span>Network</div>
    <div class="tab" onclick="switchTab('tuning')"><span class="ti">🎛️</span>Tuning</div>
    <div class="tab" onclick="switchTab('system')"><span class="ti">🔧</span>System</div>
  </div>

  <div class="content">

    <!-- OUTPUT -->
    <div class="panel active" id="panel-output">
    <div class="out-grid">
      <div class="card">
        <div class="ct">💡 sACN Output</div>
        <form action="/toggleSACN" method="POST"><button type="submit" class="btn bs">%SACN_ENABLED_TEXT%</button></form>
        <p class="it">%SACN_ENABLED_INFO%</p>
        <form action="/updateUniverse" method="POST">
          <div class="fg"><label>Universe (1&ndash;63999)</label><input type="number" name="universe" value="%UNIVERSE%" min="1" max="63999"></div>
          <button type="submit" class="btn bp">Update Universe</button>
        </form>
        <form action="/updateStartAddress" method="POST">
          <div class="fg"><label>DMX Start Address (1&ndash;512)</label><input type="number" name="dmxStartAddress" value="%DMX_START_ADDRESS%" min="1" max="512"></div>
          <button type="submit" class="btn bp">Update Address</button>
        </form>
        <form action="/updatePriority" method="POST">
          <div class="fg"><label>sACN Priority (0&ndash;200)</label><input type="number" name="sacnPriority" value="%SACN_PRIORITY%" min="0" max="200"></div>
          <button type="submit" class="btn bp">Update Priority</button>
        </form>
        <hr>
        <form action="/toggle16bit" method="POST"><button type="submit" class="btn bs">%BIT_MODE_TEXT%</button></form>
        <p class="it">%BIT_MODE_INFO%</p>
        <form action="/togglePerAddressPriority" method="POST"><button type="submit" class="btn bs">%PER_ADDR_PRI_TEXT%</button></form>
        <p class="it">%PER_ADDR_PRI_INFO%</p>
        <form action="/toggleUnicast" method="POST"><button type="submit" class="btn bs">%UNICAST_MODE_TEXT%</button></form>
        <div style="display:%UNICAST_DISPLAY%;margin-top:10px;">
          <form action="/updateUnicast" method="POST">
            <div class="fg"><label>Unicast Target IP</label><input type="text" name="unicastIP" value="%UNICAST_IP%" placeholder="192.168.1.100"></div>
            <button type="submit" class="btn bp">Set Target IP</button>
          </form>
        </div>
      </div>

      <div class="card" style="min-width:0;">
        <div class="ct">🔊 OSC Output</div>
        <form action="/toggleOSC" method="POST"><button type="submit" class="btn bs">%OSC_ENABLED_TEXT%</button></form>
        <p class="it">%OSC_INFO%</p>
        <form action="/updateOSCSettings" method="POST">
          <div class="fg"><label>Target IP</label><input type="text" name="oscIP" value="%OSC_IP%" placeholder="192.168.1.100"></div>
          <div class="fg"><label>Port</label><input type="number" name="oscPort" value="%OSC_PORT%" min="1" max="65535"></div>
          <div class="fg"><label>OSC Address</label><input type="text" name="oscAddress" value="%OSC_ADDRESS%" placeholder="/fader/1"></div>
          <div class="fg">
            <label>Send Interval (ms)</label>
            <input type="number" name="oscInterval" value="%OSC_INTERVAL%" min="25" max="10000">
            <p style="font-size:0.73rem;color:#aaa;margin-top:3px;">25=40Hz &nbsp;|&nbsp; 100=10Hz &nbsp;|&nbsp; 1000=1Hz</p>
          </div>
          <div class="fg">
            <label>Value Format</label>
            <select name="oscFormat">
              <option value="0" %OSC_FMT_SEL_0%>Float 0.0&ndash;1.0</option>
              <option value="1" %OSC_FMT_SEL_1%>Int 0&ndash;100</option>
              <option value="2" %OSC_FMT_SEL_2%>Int 0&ndash;255</option>
              <option value="3" %OSC_FMT_SEL_3%>Int 0&ndash;65535</option>
            </select>
          </div>
          <button type="submit" class="btn bp">Update OSC Settings</button>
        </form>
      </div>

      <div class="card" style="min-width:0;">
        <div class="ct">🎭 Sig-Net Output</div>
        <form action="/toggleSigNet" method="POST"><button type="submit" class="btn bs">%SIGNET_ENABLED_TEXT%</button></form>
        <p class="it">%SIGNET_INFO%</p>
        <form action="/updateSigNetSettings" method="POST">
          <div class="fg"><label>Universe (1&ndash;63999)</label><input type="number" name="snUniverse" value="%SIGNET_UNIVERSE%" min="1" max="63999"></div>
          <div class="fg"><label>DMX Start Address (1&ndash;512)</label><input type="number" name="snDMXAddr" value="%SIGNET_DMX_ADDR%" min="1" max="512"></div>
          <div class="fg">
            <label>Send Interval (ms)</label>
            <input type="number" name="snInterval" value="%SIGNET_INTERVAL%" min="25" max="10000">
            <p style="font-size:0.73rem;color:#aaa;margin-top:3px;">25=40Hz &nbsp;|&nbsp; 100=10Hz &nbsp;|&nbsp; 1000=1Hz</p>
          </div>
          <div class="fg">
            <label>Scope (1&ndash;32 chars)</label>
            <input type="text" name="snScope" value="%SIGNET_SCOPE%" maxlength="32" placeholder="local">
            <p style="font-size:0.72rem;color:#888;margin-top:2px;">A&ndash;Z, a&ndash;z, 0&ndash;9, <code>-</code> <code>.</code> <code>_</code> <code>~</code> only &mdash; must match receiver configuration</p>
          </div>
          <div class="fg">
            <label>ESTA Manufacturer Code (hex)</label>
            <input type="text" name="snMfgCode" value="%SIGNET_MFG_CODE%" maxlength="4" placeholder="0000" style="font-family:monospace;">
            <p style="font-size:0.72rem;color:#888;margin-top:2px;">4 hex digits &mdash; use 0000 if unregistered</p>
          </div>
          <hr>
          <div class="fg">
            <label>Passphrase (10&ndash;64 chars)</label>
            <input type="text" name="snPassphrase" value="%SIGNET_PASSPHRASE%" placeholder="Required — no passphrase = no output">
            <p style="font-size:0.72rem;color:#888;margin-top:2px;">Must include 3 of: uppercase, lowercase, digits, symbols &mdash; no &gt;2 identical consecutive chars, no &gt;3 sequential chars</p>
            <p style="font-size:0.73rem;color:#aaa;margin-top:3px;">Key derivation runs ~30s after save; no packets sent until complete &mdash; %SIGNET_KEY_STATUS%</p>
          </div>
          <button type="submit" class="btn bp">Update Sig-Net Settings</button>
        </form>
        <hr>
        <form action="/toggleSigNetUnicast" method="POST"><button type="submit" class="btn bs">%SIGNET_UNICAST_TEXT%</button></form>
        <div style="display:%SIGNET_UNICAST_DISPLAY%;margin-top:10px;">
          <form action="/updateSigNetUnicast" method="POST">
            <div class="fg"><label>Unicast Target IP</label><input type="text" name="snUnicastIP" value="%SIGNET_UNICAST_IP%" placeholder="192.168.1.100"></div>
            <button type="submit" class="btn bp">Set Unicast Target</button>
          </form>
        </div>
      </div>
    </div>
    </div>

    <!-- CONTROL -->
    <div class="panel" id="panel-control">
      <div class="card">
        <div class="ct">🎚️ Fader Control</div>
        <form action="/toggleSimulation" method="POST"><button type="submit" class="btn bs">%SIMULATION_MODE_TEXT%</button></form>
        <div id="sliderSection" style="margin-top:12px;">
          <div class="fg">
            <label>Simulate Fader Position</label>
            <input type="range" id="faderSlider" min="0" max="100" value="0">
            <div class="sv" id="sliderVal">0%</div>
          </div>
        </div>
        <hr>
        <form action="/toggleInvert" method="POST"><button type="submit" class="btn bs">%INVERT_MODE_TEXT%</button></form>
        <p class="it">%INVERT_INFO%</p>
        <form action="/toggleLustrSnap" method="POST"><button type="submit" class="btn bs">%LUSTR_SNAP_TEXT%</button></form>
        <p class="it">%LUSTR_SNAP_INFO%</p>
        <form action="/toggleTestMode" method="POST"><button type="submit" class="btn bs">%TEST_MODE_TEXT%</button></form>
      </div>
    </div>

    <!-- NETWORK -->
    <div class="panel" id="panel-network">
      <div class="card">
        <div class="ct">🌐 Network</div>
        <div class="ns" id="netStatus">%NETWORK_STATUS%</div>
        <form action="/dhcpToggle" method="POST"><button type="submit" class="btn bs">%DHCP_MODE_TEXT%</button></form>
        <form action="/updateIPSettings" method="POST" style="margin-top:12px;">
          <div class="fg"><label>Static IP</label><input type="text" name="staticIP" value="%STATIC_IP%"></div>
          <div class="fg"><label>Subnet Mask</label><input type="text" name="subnet" value="%SUBNET%"></div>
          <div class="fg"><label>Gateway</label><input type="text" name="gateway" value="%GATEWAY%"></div>
          <div class="fg"><label>DNS</label><input type="text" name="dns" value="%DNS%"></div>
          <button type="submit" class="btn bp">Update IP Settings</button>
        </form>
      </div>
      <div class="card">
        <div class="ct">⚙️ Device</div>
        <div class="ir"><span class="il">Serial</span><span class="iv">%DEVICE_SERIAL%</span></div>
        <div class="ir"><span class="il">Firmware</span><span class="iv">%FIRMWARE_VERSION%</span></div>
        <div class="ir"><span class="il">CID</span><span class="iv" style="font-size:0.62rem;">%SACN_CID%</span></div>
        <form action="/updateStreamName" method="POST" style="margin-top:12px;">
          <div class="fg">
            <label>Stream Name</label>
            <input type="text" name="streamName" value="%STREAM_NAME%">
            <p style="font-size:0.73rem;color:#aaa;margin-top:3px;">&#9888; Requires reboot to take effect.</p>
          </div>
          <button type="submit" class="btn bp">Update Name &amp; Reboot</button>
        </form>
      </div>
    </div>

    <!-- TUNING -->
    <div class="panel" id="panel-tuning">
      <div class="card">
        <div class="ct">🎛️ Input Tuning</div>
        <form action="/toggleEMA" method="POST"><button type="submit" class="btn bs">%EMA_MODE_TEXT%</button></form>
        <p class="it">%EMA_MODE_INFO%</p>
        <div style="display:%EMA_DISPLAY%;">
          <form action="/updateEMAAlpha" method="POST">
            <div class="fg"><label>Smoothing Amount (1=max, 100=off)</label><input type="number" name="emaAlpha" value="%EMA_ALPHA%" min="1" max="100"></div>
            <button type="submit" class="btn bp">Update Smoothing</button>
          </form>
        </div>
        <hr>
        <form action="/updateSpikeThreshold" method="POST">
          <div class="fg">
            <label>Spike Rejection (0=off, 1&ndash;500)</label>
            <input type="number" name="spikeThreshold" value="%SPIKE_THRESHOLD%" min="0" max="500">
          </div>
          <button type="submit" class="btn bp">Update Spike Rejection</button>
        </form>
        <hr>
        <form action="/updateDeadband" method="POST">
          <div class="fg">
            <label>Deadband (ADC counts, 0&ndash;200)</label>
            <input type="number" name="deadband" value="%DEADBAND%" min="0" max="200">
          </div>
          <button type="submit" class="btn bp">Update Deadband</button>
        </form>
      </div>
      <div class="card">
        <div class="ct">📐 Calibration</div>
        <div class="ir"><span class="il">Cal Min (ADC)</span><span class="iv">%CAL_MIN%</span></div>
        <div class="ir" style="margin-bottom:12px;"><span class="il">Cal Max (ADC)</span><span class="iv">%CAL_MAX%</span></div>
        <a href="/calibrate" style="text-decoration:none;"><button type="button" class="btn bs">Calibrate Fader</button></a>
        <hr>
        <form action="/updateDMXRange" method="POST">
          <label style="margin-bottom:8px;">DMX Output Range</label>
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:8px;">
            <div>
              <label style="font-size:0.73rem;color:#aaa;font-weight:400;">Min (0&ndash;254)</label>
              <input type="number" name="dmxOutMin" value="%DMX_OUT_MIN%" min="0" max="254">
            </div>
            <div>
              <label style="font-size:0.73rem;color:#aaa;font-weight:400;">Max (1&ndash;255)</label>
              <input type="number" name="dmxOutMax" value="%DMX_OUT_MAX%" min="1" max="255">
            </div>
          </div>
          <button type="submit" class="btn bs">Update DMX Range</button>
        </form>
      </div>
    </div>

    <!-- SYSTEM -->
    <div class="panel" id="panel-system">
      <div class="card">
        <div class="ct">🔧 System</div>
        <form action="/upload" method="POST" enctype="multipart/form-data">
          <div class="fg">
            <label>Firmware Update (OTA)</label>
            <input type="file" name="update" accept=".bin">
            <p style="font-size:0.73rem;color:#aaa;margin-top:3px;">&#9888; Use onefader-webversion.bin</p>
          </div>
          <button type="submit" class="btn bp">Upload Firmware</button>
        </form>
        <hr>
        <form action="/toggleWebAuth" method="POST"><button type="submit" class="btn bs">%WEB_AUTH_TOGGLE_TEXT%</button></form>
        <p class="it">%WEB_AUTH_INFO%</p>
        <div style="display:%WEB_AUTH_DISPLAY%;">
          <form action="/updateWebAuth" method="POST">
            <div class="fg"><label>Username</label><input type="text" name="webUsername" value="%WEB_USERNAME%"></div>
            <div class="fg"><label>Password</label><input type="text" name="webPassword" value="%WEB_PASSWORD%"></div>
            <button type="submit" class="btn bp">Update Credentials</button>
          </form>
        </div>
        <hr>
        <form action="/updateOTAPassword" method="POST">
          <div class="fg"><label>OTA Password</label><input type="text" name="otaPassword" value="%OTA_PASSWORD%"></div>
          <button type="submit" class="btn bs">Update OTA Password</button>
        </form>
        <hr>
        <form action="/reboot" method="POST"><button type="submit" class="btn bs">Reboot Device</button></form>
        <form action="/reset" method="POST" onsubmit="return confirm('Erase all settings?');">
          <button type="submit" class="btn bd">Factory Reset</button>
        </form>
        <hr>
        <label style="margin-bottom:8px;">Debug Logs</label>
        <div class="logs" id="logs">%LOGS%</div>
      </div>
    </div>

  </div>

  <script>
    var tabs = ['output','control','network','tuning','system'];
    function switchTab(name) {
      var els = document.querySelectorAll('.tab');
      var pnls = document.querySelectorAll('.panel');
      for (var i=0;i<pnls.length;i++) { pnls[i].classList.remove('active'); els[i].classList.remove('active'); }
      var idx = tabs.indexOf(name);
      if (idx>=0) { els[idx].classList.add('active'); document.getElementById('panel-'+name).classList.add('active'); }
      try { localStorage.setItem('of-tab', name); } catch(e){}
    }
    try { var t=localStorage.getItem('of-tab'); if(t) switchTab(t); } catch(e){}

    var slider = document.getElementById('faderSlider');
    var sliderVal = document.getElementById('sliderVal');
    slider.addEventListener('input', function() {
      sliderVal.innerText = this.value + '%';
      fetch('/setFader?value=' + this.value);
    });

    function updateMode(isSim) {
      document.getElementById('sliderSection').style.display = isSim ? 'block' : 'none';
      var b = document.getElementById('modeBadge');
      b.innerText = isSim ? 'SIMULATION' : 'HARDWARE';
      b.className = 'badge ' + (isSim ? 'badge-sim' : 'badge-hw');
    }

    setInterval(function() {
      fetch('/faderPercent').then(function(r){return r.text();}).then(function(d){
        var p = parseInt(d);
        document.getElementById('faderPct').innerText = p + '%';
        document.getElementById('faderFill').style.width = p + '%';
      });
      fetch('/getMode').then(function(r){return r.text();}).then(function(d){ updateMode(d==='simulation'); });
      fetch('/networkStatus').then(function(r){return r.text();}).then(function(d){
        document.getElementById('netStatus').innerHTML = d;
        document.getElementById('netHint').innerText = d.replace(/<br\s*\/?>/gi,' ').replace(/<[^>]+>/g,'').replace(/&nbsp;/g,' ').replace(/\s+/g,' ').trim();
      });
      fetch('/logs').then(function(r){return r.text();}).then(function(d){ document.getElementById('logs').innerText = d; });
    }, 200);

    fetch('/getMode').then(function(r){return r.text();}).then(function(d){ updateMode(d==='simulation'); });
  </script>
</body>
</html>
    )rawliteral";

    char cidStr[64];
    snprintf(cidStr, sizeof(cidStr),
             "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
             myDeviceCID[0],  myDeviceCID[1],  myDeviceCID[2],  myDeviceCID[3],
             myDeviceCID[4],  myDeviceCID[5],  myDeviceCID[6],  myDeviceCID[7],
             myDeviceCID[8],  myDeviceCID[9],  myDeviceCID[10], myDeviceCID[11],
             myDeviceCID[12], myDeviceCID[13], myDeviceCID[14], myDeviceCID[15]);

    html.replace("%STREAM_NAME%",          myDeviceName);
    html.replace("%DEVICE_SERIAL%",        deviceSerial);
    html.replace("%FIRMWARE_VERSION%",     firmwareVersion);
    html.replace("%SACN_CID%",             String(cidStr));
    html.replace("%UNIVERSE%",             String(universe));
    html.replace("%DMX_START_ADDRESS%",    String(dmxStartAddress));
    html.replace("%SACN_PRIORITY%",        String(sacnPriority));
    html.replace("%BIT_MODE_TEXT%",        use16bit ? "Switch to 8-bit Mode" : "Enable 16-bit Mode");
    html.replace("%BIT_MODE_INFO%",        use16bit ? "Currently using 2 DMX channels (coarse/fine)" : "Currently using 1 DMX channel");
    html.replace("%SACN_ENABLED_TEXT%",    sacnEnabled ? "Disable sACN Output" : "Enable sACN Output");
    html.replace("%SACN_ENABLED_INFO%",    sacnEnabled ? "sACN output active — sending DMX over the network." : "sACN output disabled — no DMX packets being sent.");
    html.replace("%PER_ADDR_PRI_TEXT%",    perAddressPriority ? "Disable Per-Address Priority" : "Enable Per-Address Priority");
    html.replace("%PER_ADDR_PRI_INFO%",    perAddressPriority ? "Per-address priority active — only controlled channel(s) claim priority." : "Per-address priority disabled — universe-level priority only.");
    html.replace("%UNICAST_MODE_TEXT%",    useUnicast ? "Switch to Multicast" : "Enable Unicast Mode");
    html.replace("%UNICAST_DISPLAY%",      useUnicast ? "block" : "none");
    html.replace("%UNICAST_IP%",           unicastTarget.toString());

    html.replace("%OSC_ENABLED_TEXT%",  oscEnabled ? "Disable OSC Output" : "Enable OSC Output");
    html.replace("%OSC_INFO%",          oscEnabled
      ? (oscTargetIP == IPAddress(0,0,0,0)
          ? "OSC enabled — set a target IP below to start sending."
          : "OSC active — sending to " + oscTargetIP.toString() + ":" + String(oscPort))
      : "OSC output disabled — sends fader value as an OSC message over UDP.");
    html.replace("%OSC_DISPLAY%",       oscEnabled ? "block" : "none");
    html.replace("%OSC_IP%",            oscTargetIP.toString());
    html.replace("%OSC_PORT%",          String(oscPort));
    html.replace("%OSC_ADDRESS%",       oscAddress);
    html.replace("%OSC_INTERVAL%",      String(oscInterval));
    html.replace("%OSC_FMT_SEL_0%",     oscFormat == 0 ? "selected" : "");
    html.replace("%OSC_FMT_SEL_1%",     oscFormat == 1 ? "selected" : "");
    html.replace("%OSC_FMT_SEL_2%",     oscFormat == 2 ? "selected" : "");
    html.replace("%OSC_FMT_SEL_3%",     oscFormat == 3 ? "selected" : "");

    html.replace("%SIGNET_ENABLED_TEXT%", signetEnabled ? "Disable Sig-Net Output" : "Enable Sig-Net Output");
    html.replace("%SIGNET_INFO%", signetEnabled
      ? (signetKeyReady
          ? "Sig-Net active — sending authenticated to universe " + String(signetUniverse) + "."
          : "Sig-Net enabled — key derivation in progress (or no passphrase).")
      : "Sig-Net disabled — Singularity UK CoAP/UDP lighting protocol.");
    html.replace("%SIGNET_UNIVERSE%",        String(signetUniverse));
    html.replace("%SIGNET_SCOPE%",           signetScope);
    { char mfgHex[5]; snprintf(mfgHex, sizeof(mfgHex), "%04X", signetMfgCode);
      html.replace("%SIGNET_MFG_CODE%", mfgHex); }
    html.replace("%SIGNET_DMX_ADDR%",        String(signetDMXAddress));
    html.replace("%SIGNET_INTERVAL%",        String(signetInterval));
    html.replace("%SIGNET_PASSPHRASE%",      signetPassphrase);
    html.replace("%SIGNET_KEY_STATUS%",      signetKeyReady ? "key ready" : (signetPassphrase.length() >= 10 ? "deriving..." : "no passphrase"));
    html.replace("%SIGNET_UNICAST_TEXT%",    signetUseUnicast ? "Switch to Multicast" : "Enable Unicast Mode");
    html.replace("%SIGNET_UNICAST_DISPLAY%", signetUseUnicast ? "block" : "none");
    html.replace("%SIGNET_UNICAST_IP%",      signetUnicastTarget.toString());

    html.replace("%TEST_MODE_TEXT%",       testModeEnabled ? "Disable Test Mode" : "Enable Test Mode");
    html.replace("%EMA_MODE_TEXT%",        emaEnabled ? "Disable Input Smoothing" : "Enable Input Smoothing");
    html.replace("%EMA_MODE_INFO%",        emaEnabled ? "Smoothing active — reduces fader noise and jitter." : "Smoothing disabled — raw ADC input.");
    html.replace("%EMA_DISPLAY%",          emaEnabled ? "block" : "none");
    html.replace("%EMA_ALPHA%",            String(emaAlpha));
    html.replace("%SPIKE_THRESHOLD%",      String(spikeThreshold));
    html.replace("%DEADBAND%",              String(deadband));
    html.replace("%CAL_MIN%",               String(adcCalMin));
    html.replace("%CAL_MAX%",               String(adcCalMax));
    html.replace("%DMX_OUT_MIN%",           String(dmxOutMin));
    html.replace("%DMX_OUT_MAX%",           String(dmxOutMax));
    html.replace("%SIMULATION_MODE_TEXT%", simulationMode  ? "Switch to Hardware Mode" : "Switch to Simulation Mode");
    html.replace("%INVERT_MODE_TEXT%",     invertFader ? "Disable Fader Invert" : "Enable Fader Invert");
    html.replace("%INVERT_INFO%",          invertFader ? "Fader is INVERTED (full = 0, off = 255)" : "Fader is normal (off = 0, full = 255)");
    html.replace("%LUSTR_SNAP_TEXT%",     lustrSnap ? "Disable Lustr Snap" : "Enable Lustr Snap");
    html.replace("%LUSTR_SNAP_INFO%",     lustrSnap ? "Lustr Snap ON — fast pull-to-zero snaps DMX output immediately to 0." : "Lustr Snap OFF — DMX output follows fader smoothly.");
    html.replace("%DHCP_MODE_TEXT%",       useDHCP ? "Switch to Static IP" : "Switch to DHCP");
    html.replace("%STATIC_IP%",            staticIP.toString());
    html.replace("%SUBNET%",               subnet.toString());
    html.replace("%GATEWAY%",              gateway.toString());
    html.replace("%DNS%",                  dns.toString());
    html.replace("%NETWORK_STATUS%",       getNetworkStatus());
    html.replace("%WEB_AUTH_TOGGLE_TEXT%",  webAuthEnabled ? "Disable Web Authentication" : "Enable Web Authentication");
    html.replace("%WEB_AUTH_INFO%",          webAuthEnabled ? "Web UI is password protected." : "Web UI is open — no login required.");
    html.replace("%WEB_AUTH_DISPLAY%",       webAuthEnabled ? "block" : "none");
    html.replace("%WEB_USERNAME%",           webUsername);
    html.replace("%WEB_PASSWORD%",           webPassword);
    html.replace("%OTA_PASSWORD%",          otaPassword);
    html.replace("%LOGS%",                 debugLogs);
    request->send(200, "text/html", html);
  });

  server.on("/networkStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", getNetworkStatus());
  });

  server.on("/faderValue", HTTP_GET, [](AsyncWebServerRequest *request) {
    uint8_t value = testModeEnabled ? (uint8_t)map(testPercent, 0, 100, 0, 255) : (use16bit ? (readFaderValue16bit() >> 8) : readFaderValue());
    request->send(200, "text/plain", String(value));
  });

  server.on("/faderPercent", HTTP_GET, [](AsyncWebServerRequest *request) {
    uint8_t pct;
    if (testModeEnabled) {
      pct = invertFader ? (100 - testPercent) : testPercent;
    } else if (simulationMode) {
      uint8_t raw = (uint8_t)constrain(simulatedFaderValue, 0, 100);
      pct = invertFader ? (100 - raw) : raw;
    } else {
      int adc = constrain(readSmoothedAnalog(), adcCalMin, adcCalMax);
      uint8_t raw = (uint8_t)min((long)100, map(adc, adcCalMin, adcCalMax, 0, 101));
      pct = invertFader ? (100 - raw) : raw;
    }
    request->send(200, "text/plain", String(pct));
  });

  server.on("/setFader", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("value")) {
      simulatedFaderValue = (uint16_t)constrain(request->getParam("value")->value().toInt(), 0, 100);
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/getMode", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", simulationMode ? "simulation" : "hardware");
  });

  server.on("/toggleSimulation", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    simulationMode = !simulationMode;
    addLog(simulationMode ? "Switched to Simulation Mode" : "Switched to Hardware Mode");
    if (simulationMode) simulatedFaderValue = 0;
    request->redirect("/");
  });

  server.on("/toggleInvert", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    invertFader = !invertFader;
    preferences.putBool("invertFader", invertFader);
    addLog(invertFader ? "Fader Invert: ON" : "Fader Invert: OFF");
    request->redirect("/");
  });

  server.on("/toggleLustrSnap", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    lustrSnap = !lustrSnap;
    lustrSnapActive = false;
    lustrSnapPrevDMX = 0;
    preferences.putBool("lustrSnap", lustrSnap);
    addLog(lustrSnap ? "Lustr Snap: ON" : "Lustr Snap: OFF");
    request->redirect("/");
  });

  server.on("/updateStreamName", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("streamName", true)) {
      myDeviceName = request->getParam("streamName", true)->value();
      addLog("Stream Name: " + myDeviceName + " — rebooting to apply");
      preferences.putString("streamName", myDeviceName);
      request->send(200, "text/plain", "Name saved. Rebooting...");
      delay(500);
      ESP.restart();
    }
    request->redirect("/");
  });

  server.on("/updateUniverse", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("universe", true)) {
      uint16_t newUniverse = request->getParam("universe", true)->value().toInt();
      if (newUniverse < 1 || newUniverse > 63999) {
        addLog("Invalid Universe: " + String(newUniverse));
        request->redirect("/");
        return;
      }
      universe = newUniverse;
      addLog("Universe: " + String(universe));
      preferences.putUInt("universe", universe);
      sacnBegin();
      resetDMXOutput();
      updateDDData();
    }
    request->redirect("/");
  });

  server.on("/updateStartAddress", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("dmxStartAddress", true)) {
      uint16_t newAddr = request->getParam("dmxStartAddress", true)->value().toInt();
      if (newAddr < 1 || newAddr > 512) {
        addLog("Invalid DMX Address: " + String(newAddr));
        request->redirect("/");
        return;
      }
      dmxStartAddress = newAddr;
      addLog("DMX Address: " + String(dmxStartAddress));
      preferences.putUInt("dmxStartAddress", dmxStartAddress);
      resetDMXOutput();
      updateDDData();
    }
    request->redirect("/");
  });

  server.on("/updatePriority", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("sacnPriority", true)) {
      uint8_t newPriority = request->getParam("sacnPriority", true)->value().toInt();
      if (newPriority > 200) {
        addLog("Invalid Priority: " + String(newPriority));
        request->redirect("/");
        return;
      }
      sacnPriority = newPriority;
      addLog("sACN Priority: " + String(sacnPriority));
      preferences.putUChar("sacnPriority", sacnPriority);
      sacnBegin();
      updateDDData();
    }
    request->redirect("/");
  });

  server.on("/togglePerAddressPriority", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    perAddressPriority = !perAddressPriority;
    preferences.putBool("perAddrPri", perAddressPriority);
    sacnBegin();
    updateDDData();
    addLog(perAddressPriority ? "Per-Address Priority: ON" : "Per-Address Priority: OFF");
    request->redirect("/");
  });

  server.on("/toggle16bit", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    use16bit = !use16bit;
    preferences.putBool("use16bit", use16bit);
    addLog(use16bit ? "16-bit mode" : "8-bit mode");
    resetDMXOutput();
    updateDDData();
    request->redirect("/");
  });

  server.on("/toggleUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    useUnicast = !useUnicast;
    preferences.putBool("useUnicast", useUnicast);
    sacnBegin();
    updateDDData();
    request->redirect("/");
  });

  server.on("/updateUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("unicastIP", true)) {
      String ipStr = request->getParam("unicastIP", true)->value();
      if (unicastTarget.fromString(ipStr)) {
        preferences.putUInt("unicastIP", (uint32_t)unicastTarget);
        if (useUnicast) { sacnBegin(); updateDDData(); }
        addLog("Unicast target: " + unicastTarget.toString());
      } else {
        addLog("Invalid IP: " + ipStr);
      }
    }
    request->redirect("/");
  });

  server.on("/toggleSACN", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    sacnEnabled = !sacnEnabled;
    preferences.putBool("sacnEnabled", sacnEnabled);
    addLog(sacnEnabled ? "sACN: ON" : "sACN: OFF");
    request->redirect("/");
  });

  server.on("/toggleOSC", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    oscEnabled = !oscEnabled;
    preferences.putBool("oscEnabled", oscEnabled);
    addLog(oscEnabled ? "OSC: ON" : "OSC: OFF");
    request->redirect("/");
  });

  server.on("/updateOSCSettings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("oscIP", true)) {
      IPAddress newIP;
      if (newIP.fromString(request->getParam("oscIP", true)->value())) {
        oscTargetIP = newIP;
        preferences.putUInt("oscIP", (uint32_t)oscTargetIP);
      }
    }
    if (request->hasParam("oscPort", true)) {
      int p = request->getParam("oscPort", true)->value().toInt();
      if (p >= 1 && p <= 65535) {
        oscPort = (uint16_t)p;
        preferences.putUInt("oscPort", oscPort);
      }
    }
    if (request->hasParam("oscAddress", true)) {
      String addr = request->getParam("oscAddress", true)->value();
      addr.trim();
      if (addr.length() > 0 && addr.length() <= 60) {
        if (!addr.startsWith("/")) addr = "/" + addr;
        oscAddress = addr;
        preferences.putString("oscAddress", oscAddress);
      }
    }
    if (request->hasParam("oscFormat", true)) {
      oscFormat = (uint8_t)constrain(request->getParam("oscFormat", true)->value().toInt(), 0, 3);
      preferences.putUChar("oscFormat", oscFormat);
    }
    if (request->hasParam("oscInterval", true)) {
      int iv = request->getParam("oscInterval", true)->value().toInt();
      oscInterval = (uint16_t)constrain(iv, 25, 10000);
      preferences.putUInt("oscInterval", oscInterval);
    }
    addLog("OSC: " + oscTargetIP.toString() + ":" + String(oscPort) + " " + oscAddress + " " + String(oscInterval) + "ms");
    request->redirect("/");
  });

  server.on("/toggleSigNet", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    signetEnabled = !signetEnabled;
    preferences.putBool("snEnabled", signetEnabled);
    addLog(signetEnabled ? "Sig-Net: ON" : "Sig-Net: OFF");
    request->redirect("/");
  });

  server.on("/updateSigNetSettings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    bool passphraseChanged = false;
    if (request->hasParam("snUniverse", true)) {
      int u = request->getParam("snUniverse", true)->value().toInt();
      if (u >= 1 && u <= 63999) { signetUniverse = (uint16_t)u; preferences.putUInt("snUniverse", signetUniverse); }
    }
    if (request->hasParam("snDMXAddr", true)) {
      int a = request->getParam("snDMXAddr", true)->value().toInt();
      if (a >= 1 && a <= 512) { signetDMXAddress = (uint16_t)a; preferences.putUInt("snDMXAddr", signetDMXAddress); }
    }
    if (request->hasParam("snInterval", true)) {
      int iv = request->getParam("snInterval", true)->value().toInt();
      signetInterval = (uint16_t)constrain(iv, 25, 10000);
      preferences.putUInt("snInterval", signetInterval);
    }
    if (request->hasParam("snMfgCode", true)) {
      String mh = request->getParam("snMfgCode", true)->value();
      mh.trim();
      if (mh.length() >= 1 && mh.length() <= 4) {
        // Validate all hex chars
        bool hexOk = true;
        for (int i = 0; hexOk && i < (int)mh.length(); i++) {
          char c = mh[i];
          hexOk = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
        }
        if (hexOk) {
          uint16_t code = (uint16_t)strtoul(mh.c_str(), nullptr, 16);
          if (code != signetMfgCode) {
            signetMfgCode = code;
            preferences.putUInt("snMfgCode", signetMfgCode);
          }
        }
      }
    }
    if (request->hasParam("snScope", true)) {
      String sc = request->getParam("snScope", true)->value();
      sc.trim();
      // Validate: 1-32 URL-safe unreserved chars (A-Z, a-z, 0-9, - . _ ~)
      bool valid = sc.length() >= 1 && sc.length() <= 32;
      for (int i = 0; valid && i < (int)sc.length(); i++) {
        char c = sc[i];
        valid = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '-' || c == '.' || c == '_' || c == '~';
      }
      if (valid && sc != signetScope) {
        signetScope = sc;
        preferences.putString("snScope", signetScope);
      }
    }
    if (request->hasParam("snPassphrase", true)) {
      String pp = request->getParam("snPassphrase", true)->value();
      pp.trim();
      if (pp != signetPassphrase) {
        signetPassphrase = pp;
        preferences.putString("snPassphrase", signetPassphrase);
        passphraseChanged = true;
      }
    }
    addLog("Sig-Net: universe=" + String(signetUniverse) + " addr=" + String(signetDMXAddress));
    if (passphraseChanged) {
      signetKeyReady = false;
      if (signetPassphrase.length() >= 10) {
        // Derive key in background FreeRTOS task
        xTaskCreate(signetKeyTask, "sn_key", 8192, nullptr, 1, nullptr);
        addLog("Sig-Net: key derivation started");
      } else {
        addLog("Sig-Net: no passphrase — unprovisioned mode");
      }
    }
    request->redirect("/");
  });

  server.on("/toggleSigNetUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    signetUseUnicast = !signetUseUnicast;
    preferences.putBool("snUseUnicast", signetUseUnicast);
    addLog(signetUseUnicast ? "Sig-Net: unicast" : "Sig-Net: multicast");
    request->redirect("/");
  });

  server.on("/updateSigNetUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("snUnicastIP", true)) {
      IPAddress newIP;
      if (newIP.fromString(request->getParam("snUnicastIP", true)->value())) {
        signetUnicastTarget = newIP;
        preferences.putUInt("snUnicastIP", (uint32_t)signetUnicastTarget);
        addLog("Sig-Net unicast target: " + signetUnicastTarget.toString());
      }
    }
    request->redirect("/");
  });

  server.on("/startCalibration", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    calibrating = true;
    calibLiveMin = 4095;
    calibLiveMax = 0;
    calibStartTime = millis();
    addLog("Calibration started");
    request->redirect("/calibrate");
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    // Start calibration automatically when page loads
    calibrating = true;
    calibLiveMin = 4095;
    calibLiveMax = 0;
    calibStartTime = millis();
    String html = R"rawliteral(
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Fader Calibration</title>
        <style>
          * { margin: 0; padding: 0; box-sizing: border-box; }
          body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; color: #333; display: flex; flex-direction: column; align-items: center; justify-content: center; }
          .card { background: rgba(255,255,255,0.95); border-radius: 20px; padding: 36px; box-shadow: 0 8px 32px rgba(0,0,0,0.15); max-width: 460px; width: 100%; }
          .header { text-align: center; margin-bottom: 28px; }
          .header h1 { font-size: 1.8rem; font-weight: 700; color: #667eea; margin-bottom: 6px; }
          .header p { color: #888; font-size: 0.9rem; line-height: 1.5; }
          .info-grid { display: grid; gap: 12px; margin-bottom: 24px; }
          .info-item { display: flex; justify-content: space-between; align-items: center; padding: 14px 16px; background: #f8f9fa; border-radius: 12px; font-size: 0.9rem; }
          .info-label { font-weight: 600; color: #666; }
          .info-value { color: #333; font-family: 'Courier New', monospace; font-size: 1rem; font-weight: 700; }
          .live-badge { display: inline-block; width: 8px; height: 8px; background: #34d399; border-radius: 50%; margin-right: 8px; animation: pulse 1s infinite; }
          @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.3; } }
          .status { text-align: center; padding: 14px; border-radius: 12px; margin-bottom: 24px; font-weight: 600; font-size: 0.9rem; background: #fef3c7; color: #92400e; }
          button { width: 100%; padding: 14px; margin-bottom: 12px; border: none; border-radius: 12px; font-size: 1rem; font-weight: 600; cursor: pointer; transition: all 0.2s; }
          .btn-save { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; box-shadow: 0 4px 15px rgba(102,126,234,0.4); }
          .btn-save:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(102,126,234,0.6); }
          .btn-save:disabled { opacity: 0.5; cursor: not-allowed; transform: none; }
          .btn-cancel { background: linear-gradient(135deg, #64748b 0%, #475569 100%); color: white; box-shadow: 0 4px 15px rgba(100,116,139,0.3); }
          .btn-cancel:hover { transform: translateY(-2px); }
          .range-bar { width: 100%; height: 16px; background: #e0e0e0; border-radius: 8px; overflow: hidden; margin-bottom: 24px; position: relative; }
          .range-fill { height: 100%; background: linear-gradient(90deg, #667eea, #764ba2); border-radius: 8px; transition: width 0.1s; width: 0%; }
        </style>
      </head>
      <body>
        <div class="card">
          <div class="header">
            <h1>📐 Fader Calibration</h1>
            <p>Move the fader slowly to <strong>both ends</strong> of its travel, then click Save.</p>
          </div>
          <div class="status"><span class="live-badge"></span>Calibrating — move fader to both extremes &nbsp;|&nbsp; <span id="countdown">2:00</span> remaining</div>
          <div class="range-bar"><div class="range-fill" id="rangeFill"></div></div>
          <div class="info-grid">
            <div class="info-item">
              <span class="info-label">Live Min (ADC)</span>
              <span class="info-value" id="liveMin">—</span>
            </div>
            <div class="info-item">
              <span class="info-label">Live Max (ADC)</span>
              <span class="info-value" id="liveMax">—</span>
            </div>
            <div class="info-item">
              <span class="info-label">Range Captured</span>
              <span class="info-value" id="rangeSpan">—</span>
            </div>
          </div>
          <form action="/saveCalibration" method="POST">
            <button type="submit" class="btn-save" id="saveBtn">Save Calibration</button>
          </form>
          <form action="/cancelCalibration" method="POST">
            <button type="submit" class="btn-cancel">Cancel</button>
          </form>
          <div style="margin-top:24px;padding-top:24px;border-top:2px solid #e8e8f0;">
            <div style="text-align:center;font-size:1rem;font-weight:700;color:#667eea;margin-bottom:6px;">Manual Entry</div>
            <p style="text-align:center;font-size:0.82rem;color:#888;margin-bottom:16px;line-height:1.5;">Enter known ADC values directly. Min must be at least 100 counts below Max.</p>
            <form action="/setCalibrationManual" method="POST">
              <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px;">
                <div>
                  <label style="display:block;font-size:0.8rem;font-weight:600;color:#555;margin-bottom:6px;">Min (ADC)</label>
                  <input type="number" name="calMin" id="manualMin" min="0" max="4094" style="width:100%;padding:10px 12px;border:2px solid #e0e0e0;border-radius:10px;font-size:0.95rem;" required>
                </div>
                <div>
                  <label style="display:block;font-size:0.8rem;font-weight:600;color:#555;margin-bottom:6px;">Max (ADC)</label>
                  <input type="number" name="calMax" id="manualMax" min="1" max="4095" style="width:100%;padding:10px 12px;border:2px solid #e0e0e0;border-radius:10px;font-size:0.95rem;" required>
                </div>
              </div>
              <button type="submit" class="btn-save" style="margin-bottom:0;">Apply Manual Values</button>
            </form>
          </div>
        </div>
        <script>
          function fmt(s) {
            var m = Math.floor(s / 60);
            var sec = s % 60;
            return m + ':' + (sec < 10 ? '0' : '') + sec;
          }
          var manualPrefilled = false;
          function poll() {
            fetch('/calStatus').then(r => r.json()).then(d => {
              if (d.liveMin < 4095) document.getElementById('liveMin').textContent = d.liveMin;
              if (d.liveMax > 0)    document.getElementById('liveMax').textContent = d.liveMax;
              if (d.liveMin < 4095 && d.liveMax > 0) {
                var span = d.liveMax - d.liveMin;
                document.getElementById('rangeSpan').textContent = span + ' counts';
                var pct = Math.min(100, Math.round(span / 4095 * 100));
                document.getElementById('rangeFill').style.width = pct + '%';
              }
              if (!manualPrefilled) {
                document.getElementById('manualMin').value = d.calMin;
                document.getElementById('manualMax').value = d.calMax;
                manualPrefilled = true;
              }
              if (!d.calibrating) {
                window.location.href = '/';
              } else {
                document.getElementById('countdown').textContent = fmt(d.secsLeft);
                if (d.secsLeft <= 10) {
                  document.getElementById('countdown').style.color = '#dc2626';
                  document.getElementById('countdown').style.fontWeight = '700';
                }
              }
            }).catch(() => {});
          }
          setInterval(poll, 500);
          poll();
        </script>
      </body>
      </html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  server.on("/calStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
    int secsLeft = calibrating ? max(0, (int)((CAL_TIMEOUT_MS - (millis() - calibStartTime)) / 1000)) : 0;
    String json = "{\"calibrating\":" + String(calibrating ? "true" : "false") +
                  ",\"liveMin\":"  + String(calibLiveMin) +
                  ",\"liveMax\":"  + String(calibLiveMax) +
                  ",\"secsLeft\":" + String(secsLeft) +
                  ",\"calMin\":"   + String(adcCalMin) +
                  ",\"calMax\":"   + String(adcCalMax) + "}";
    request->send(200, "application/json", json);
  });

  server.on("/saveCalibration", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (calibLiveMax > calibLiveMin + 100) {
      // Use exact captured values — no margin needed since map() with
      // constrain will clamp anything at or beyond these to 0/100%
      adcCalMin = calibLiveMin;
      adcCalMax = calibLiveMax;
      preferences.putInt("adcCalMin", adcCalMin);
      preferences.putInt("adcCalMax", adcCalMax);
      addLog("Calibration saved: min=" + String(adcCalMin) + " max=" + String(adcCalMax));
    } else {
      addLog("Calibration range too small — not saved");
    }
    calibrating = false;
    request->redirect("/");
  });

  server.on("/cancelCalibration", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    calibrating = false;
    addLog("Calibration cancelled");
    request->redirect("/");
  });

  server.on("/setCalibrationManual", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("calMin", true) && request->hasParam("calMax", true)) {
      int newMin = request->getParam("calMin", true)->value().toInt();
      int newMax = request->getParam("calMax", true)->value().toInt();
      if (newMin < 0 || newMax > 4095 || newMax <= newMin + 100) {
        addLog("Manual cal rejected: min=" + String(newMin) + " max=" + String(newMax));
        request->redirect("/calibrate");
        return;
      }
      adcCalMin = newMin;
      adcCalMax = newMax;
      preferences.putInt("adcCalMin", adcCalMin);
      preferences.putInt("adcCalMax", adcCalMax);
      calibrating = false;
      addLog("Manual calibration saved: min=" + String(adcCalMin) + " max=" + String(adcCalMax));
    }
    request->redirect("/");
  });

  server.on("/updateDMXRange", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("dmxOutMin", true) && request->hasParam("dmxOutMax", true)) {
      uint8_t newMin = (uint8_t)constrain(request->getParam("dmxOutMin", true)->value().toInt(), 0, 254);
      uint8_t newMax = (uint8_t)constrain(request->getParam("dmxOutMax", true)->value().toInt(), 1, 255);
      if (newMax <= newMin) {
        addLog("DMX Range rejected: min=" + String(newMin) + " max=" + String(newMax));
        request->redirect("/");
        return;
      }
      dmxOutMin = newMin;
      dmxOutMax = newMax;
      preferences.putUChar("dmxOutMin", dmxOutMin);
      preferences.putUChar("dmxOutMax", dmxOutMax);
      addLog("DMX Range: " + String(dmxOutMin) + "-" + String(dmxOutMax));
    }
    request->redirect("/");
  });

  server.on("/updateDeadband", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("deadband", true)) {
      deadband = constrain(request->getParam("deadband", true)->value().toInt(), 0, 200);
      preferences.putInt("deadband", deadband);
      addLog("Deadband: " + String(deadband));
    }
    request->redirect("/");
  });

  server.on("/updateSpikeThreshold", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("spikeThreshold", true)) {
      spikeThreshold = (uint16_t)constrain(request->getParam("spikeThreshold", true)->value().toInt(), 0, 500);
      preferences.putUInt("spikeThreshold", spikeThreshold);
      addLog("Spike Rejection: " + (spikeThreshold == 0 ? String("OFF") : String(spikeThreshold)));
    }
    request->redirect("/");
  });

  server.on("/toggleEMA", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    emaEnabled = !emaEnabled;
    emaInitialised = false; // reset so EMA re-seeds on next read
    preferences.putBool("emaEnabled", emaEnabled);
    addLog(emaEnabled ? "Input Smoothing ON" : "Input Smoothing OFF");
    request->redirect("/");
  });

  server.on("/updateEMAAlpha", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("emaAlpha", true)) {
      uint8_t val = (uint8_t)constrain(request->getParam("emaAlpha", true)->value().toInt(), 1, 100);
      emaAlpha = val;
      emaInitialised = false;
      preferences.putUChar("emaAlpha", emaAlpha);
      addLog("Smoothing alpha: " + String(emaAlpha));
    }
    request->redirect("/");
  });

  server.on("/toggleTestMode", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    testModeEnabled = !testModeEnabled;
    addLog(testModeEnabled ? "Test Mode ON" : "Test Mode OFF");
    if (testModeEnabled) { testPercent = 0; testFadeDirection = true; }
    resetDMXOutput();
    request->redirect("/");
  });

  server.on("/updateIPSettings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("staticIP", true) && request->hasParam("subnet", true) &&
        request->hasParam("gateway", true) && request->hasParam("dns", true)) {
      staticIP.fromString(request->getParam("staticIP", true)->value());
      subnet.fromString(request->getParam("subnet", true)->value());
      gateway.fromString(request->getParam("gateway", true)->value());
      dns.fromString(request->getParam("dns", true)->value());
      addLog("IP Settings Updated");
      saveNetworkSettings();
      if (!useDHCP) applyEthernetSettings();
    }
    request->redirect("/");
  });

  server.on("/dhcpToggle", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    useDHCP = !useDHCP;
    saveNetworkSettings();
    addLog(useDHCP ? "Switched to DHCP" : "Switched to Static IP");
    request->redirect("/");
  });

  server.on("/toggleWebAuth", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    webAuthEnabled = !webAuthEnabled;
    preferences.putBool("webAuthEnabled", webAuthEnabled);
    addLog(webAuthEnabled ? "Web Auth: ON" : "Web Auth: OFF");
    request->redirect("/");
  });

  server.on("/updateWebAuth", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("webUsername", true) && request->hasParam("webPassword", true)) {
      webUsername = request->getParam("webUsername", true)->value();
      webPassword = request->getParam("webPassword", true)->value();
      preferences.putString("webUsername", webUsername);
      preferences.putString("webPassword", webPassword);
      addLog("Web auth updated");
    }
    request->redirect("/");
  });

  server.on("/updateOTAPassword", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("otaPassword", true)) {
      otaPassword = request->getParam("otaPassword", true)->value();
      preferences.putString("otaPassword", otaPassword);
      ArduinoOTA.setPassword(otaPassword.c_str());
      addLog("OTA Password updated");
    }
    request->redirect("/");
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    addLog("Reboot triggered via web UI");
    request->send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
  });

  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    addLog("Factory Reset Triggered");
    preferences.clear();
    request->send(200, "text/plain", "Factory reset complete. Device will restart...");
    delay(1000);
    ESP.restart();
  });

  server.on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (!checkAuth(request)) return;
      request->send(200, "text/plain", Update.hasError() ? "Update Failed" : "Update Complete. Rebooting...");
      delay(1000);
      ESP.restart();
    },
    [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!index) {
        addLog("Firmware Update: " + filename);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(USBSerial);
          addLog("Update begin failed");
        }
      }
      if (Update.write(data, len) != len) {
        Update.printError(USBSerial);
        addLog("Firmware write failed");
      }
      if (final) {
        if (Update.end(true)) {
          addLog("Firmware Update OK");
        } else {
          Update.printError(USBSerial);
          addLog("Firmware Update Failed");
        }
      }
    }
  );

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", debugLogs);
  });

  server.begin();
  addLog("Web server started");
}

// -------------------------------------------------------
// setup()
// -------------------------------------------------------
void setup() {
  USBSerial.begin(115200);
  USB.begin();
  delay(1000);
  // Wait for USB CDC to enumerate (native USB needs a moment)
  unsigned long t = millis();
  while (!USBSerial && millis() - t < 3000);

  pinMode(FADER_PIN, INPUT);

  // Pre-fill smoothing buffer
  for (int i = 0; i < SMOOTHING_SAMPLES; i++) {
    int reading = analogRead(FADER_PIN);
    smoothingBuffer[i] = reading;
    smoothingTotal += reading;
    delay(5);
  }
  lastSmoothedValue = smoothingTotal / SMOOTHING_SAMPLES;
  emaValue = (float)lastSmoothedValue;
  emaInitialised = true;
  addLog("Fader initialized on IO1");

  preferences.begin("onefader", false);

  // Load saved settings
  myDeviceName    = preferences.getString("streamName",    "OneFader");
  otaPassword     = preferences.getString("otaPassword",    "netstage");
  webUsername     = preferences.getString("webUsername",    "admin");
  webPassword     = preferences.getString("webPassword",    "netstage");
  webAuthEnabled  = preferences.getBool("webAuthEnabled",  false);
  universe        = preferences.getUInt("universe",        1);
  dmxStartAddress = preferences.getUInt("dmxStartAddress", 1);
  sacnPriority    = preferences.getUChar("sacnPriority",   100);
  simulationMode  = false; // always start in hardware mode
  deadband         = preferences.getInt("deadband",          30);
  adcCalMin        = preferences.getInt("adcCalMin",         10);
  adcCalMax        = preferences.getInt("adcCalMax",         4085);
  emaEnabled       = preferences.getBool("emaEnabled",       true);
  emaAlpha         = preferences.getUChar("emaAlpha",        15);
  spikeThreshold   = preferences.getUInt("spikeThreshold",  200);
  useUnicast          = preferences.getBool("useUnicast",      false);
  use16bit            = preferences.getBool("use16bit",        false);
  invertFader         = preferences.getBool("invertFader",     false);
  lustrSnap           = preferences.getBool("lustrSnap",       false);
  perAddressPriority  = preferences.getBool("perAddrPri",      true);
  dmxOutMin           = preferences.getUChar("dmxOutMin",      0);
  dmxOutMax           = preferences.getUChar("dmxOutMax",      255);

  uint32_t savedUnicast = preferences.getUInt("unicastIP", 0);
  if (savedUnicast != 0) unicastTarget = IPAddress(savedUnicast);

  sacnEnabled = preferences.getBool("sacnEnabled",   true);
  oscEnabled  = preferences.getBool("oscEnabled",    false);
  oscPort     = (uint16_t)preferences.getUInt("oscPort", 8000);
  oscAddress  = preferences.getString("oscAddress",  "/fader/1");
  oscFormat   = preferences.getUChar("oscFormat",    0);
  oscInterval = (uint16_t)preferences.getUInt("oscInterval", 25);
  uint32_t savedOscIP = preferences.getUInt("oscIP", 0);
  if (savedOscIP != 0) oscTargetIP = IPAddress(savedOscIP);

  // Load Sig-Net settings
  signetEnabled    = preferences.getBool("snEnabled",    false);
  signetUniverse   = (uint16_t)preferences.getUInt("snUniverse",  1);
  signetDMXAddress = (uint16_t)preferences.getUInt("snDMXAddr",   1);
  signetPassphrase = preferences.getString("snPassphrase", "");
  signetScope      = preferences.getString("snScope",      "local");
  signetMfgCode    = (uint16_t)preferences.getUInt("snMfgCode",   0x0000);
  signetUseUnicast = preferences.getBool("snUseUnicast",  false);
  signetInterval   = (uint16_t)preferences.getUInt("snInterval",  25);
  signetSessionID  = preferences.getUInt("snSessionID",   1);
  uint32_t savedSNIP = preferences.getUInt("snUnicastIP", 0);
  if (savedSNIP != 0) signetUnicastTarget = IPAddress(savedSNIP);

  // Increment session ID each boot (anti-replay) and save immediately
  signetSessionID++;
  preferences.putUInt("snSessionID", signetSessionID);
  signetSequence = 0;

  loadNetworkSettings();

  // Register Ethernet event handlers
  Network.onEvent(onEthStart,       ARDUINO_EVENT_ETH_START);
  Network.onEvent(onEthConnected,   ARDUINO_EVENT_ETH_CONNECTED);
  Network.onEvent(onEthGotIP,       ARDUINO_EVENT_ETH_GOT_IP);
  Network.onEvent(onEthDisconnected,ARDUINO_EVENT_ETH_DISCONNECTED);
  Network.onEvent(onEthStop,        ARDUINO_EVENT_ETH_STOP);

  // ---- Ethernet startup ----
  addLog("Starting Ethernet W5500...");
  addLog("Pins: SCK=13 MISO=12 MOSI=11 CS=14 INT=10 RST=9");

  pinMode(ETH_SPI_RST, OUTPUT);
  digitalWrite(ETH_SPI_RST, LOW);
  delay(10);
  digitalWrite(ETH_SPI_RST, HIGH);
  delay(10);

  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, -1);
  pinMode(ETH_SPI_CS, OUTPUT);
  digitalWrite(ETH_SPI_CS, HIGH);
  delay(50);

  ETH.begin(ETH_PHY_W5500, 1, ETH_SPI_CS, ETH_SPI_INT, ETH_SPI_RST, SPI, ETH_SPI_HOST);
  SPI.setFrequency(25000000);

  if (!useDHCP) applyEthernetSettings();

  addLog("Waiting for Ethernet link...");
  int timeout = 0;
  while (!ethConnected && timeout < 100) {
    delay(100);
    timeout++;
  }

  if (ethConnected) {
    addLog("Ethernet connected: " + ETH.localIP().toString());
  } else {
    addLog("WARNING: No Ethernet connection - check cable");
  }

  // Generate serial and CID now that Ethernet MAC is available
  deviceSerial = generateSerial();
  addLog("Serial: " + deviceSerial);
  ETH.setHostname(("OneFader-" + deviceSerial).c_str());

  generateCID();
  deviceCID(myDeviceCID);
  deviceName(myDeviceName.c_str());

  // OTA
  ArduinoOTA.setHostname(("OneFader-" + deviceSerial).c_str());
  ArduinoOTA.setPassword(otaPassword.c_str());
  ArduinoOTA.begin();
  addLog("OTA ready");

  // sACN
  sacnBegin();
  updateDDData();

  addLog("Universe: "   + String(universe));
  addLog("DMX Addr: "   + String(dmxStartAddress));
  addLog("Priority: "   + String(sacnPriority));
  addLog("DMX Mode: "   + String(use16bit ? "16-bit" : "8-bit"));
  addLog("DMX Range: "  + String(dmxOutMin) + "-" + String(dmxOutMax));
  addLog("sACN Out: " + String(sacnEnabled ? "ON" : "OFF"));
  addLog("Per-Addr Pri: " + String(perAddressPriority ? "ON" : "OFF"));
  addLog("OSC: " + String(oscEnabled ? "ON" : "OFF"));
  addLog("Sig-Net: " + String(signetEnabled ? "ON" : "OFF"));
  addLog("Rate: 40Hz");

  // Start Sig-Net key derivation in background if passphrase is configured
  if (signetPassphrase.length() >= 10) {
    xTaskCreate(signetKeyTask, "sn_key", 8192, nullptr, 1, nullptr);
  }

  setupWebServer();

  addLog("Ready — " + String(simulationMode ? "SIMULATION" : "HARDWARE") + " mode");
}

// -------------------------------------------------------
// loop()
// -------------------------------------------------------
void loop() {
  // ---- USB Serial command handler --------------------------------------
  while (USBSerial.available()) {
    char c = USBSerial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 128) serialBuffer = "";
    }
  }

  // ---- Regular tasks --------------------------------------------------
  ArduinoOTA.handle();

  // Auto-cancel calibration if it times out
  if (calibrating && (millis() - calibStartTime > CAL_TIMEOUT_MS)) {
    calibrating = false;
    addLog("Calibration timed out — auto cancelled");
  }



  unsigned long currentMillis = millis();

  if (currentMillis - lastSACNSend >= SACN_INTERVAL) {
    lastSACNSend = currentMillis;

    if (testModeEnabled) {
      // Cycle 0-100% up then down, mimicking a real fader sweep
      if (testFadeDirection) {
        if (testPercent >= 100) { testFadeDirection = false; }
        else testPercent++;
      } else {
        if (testPercent == 0) { testFadeDirection = true; }
        else testPercent--;
      }
      // Apply invert and output range limits, same as hardware path
      uint16_t raw16 = (uint16_t)map(testPercent, 0, 100, 0, 65535);
      if (invertFader) raw16 = 65535 - raw16;
      uint16_t outMin16 = (uint16_t)((uint32_t)dmxOutMin * 257);
      uint16_t outMax16 = (uint16_t)((uint32_t)dmxOutMax * 257);
      raw16 = (uint16_t)map((long)raw16, 0, 65535, (long)outMin16, (long)outMax16);
      if (use16bit) {
        dmxData[dmxStartAddress - 1] = (raw16 >> 8) & 0xFF; // coarse
        if (dmxStartAddress <= 511) dmxData[dmxStartAddress] = raw16 & 0xFF; // fine
      } else {
        dmxData[dmxStartAddress - 1] = (raw16 >> 8) & 0xFF;
        if (dmxStartAddress <= 511) dmxData[dmxStartAddress] = 0;
      }
    } else {
      if (use16bit) {
        uint16_t val16 = readFaderValue16bit();
        uint8_t coarse = (val16 >> 8) & 0xFF;
        uint8_t fine   = val16 & 0xFF;
        dmxData[dmxStartAddress - 1] = coarse;
        if (dmxStartAddress <= 511) {
          dmxData[dmxStartAddress] = fine;
        }
      } else {
        dmxData[dmxStartAddress - 1] = readFaderValue();
        if (dmxStartAddress <= 511) {
          dmxData[dmxStartAddress] = 0;
        }
      }
    }

    // Lustr snap — if enabled and fader is being pulled down fast, force DMX to 0.
    // Only triggers when the output is already below 50 DMX (~20% of range) to avoid
    // false positives when quickly setting mid-level values. Releases when the fader
    // reverses upward (operator recovers) or physically reaches zero.
    if (lustrSnap && !testModeEnabled) {
      uint8_t coarseVal = dmxData[dmxStartAddress - 1];
      int delta = (int)lustrSnapPrevDMX - (int)coarseVal; // positive = dropping
      if (!lustrSnapActive) {
        if (delta >= 8 && coarseVal > 5 && coarseVal < 50) lustrSnapActive = true;
      }
      if (lustrSnapActive) {
        dmxData[dmxStartAddress - 1] = 0;
        if (dmxStartAddress <= 511) dmxData[dmxStartAddress] = 0;
        if (coarseVal == 0 || delta < -5) lustrSnapActive = false;
      }
      lustrSnapPrevDMX = coarseVal;
    }

    // sACN output — gated by sacnEnabled; dmxData is always updated above
    // so OSC continues to reflect the live fader value even when sACN is off
    if (sacnEnabled) {
      sacn.dmx(dmxData);
      if (use16bit) {
        sacn.dmx(dmxStartAddress,     dmxData[dmxStartAddress - 1]); // coarse
        sacn.dmx(dmxStartAddress + 1, dmxData[dmxStartAddress]);     // fine
      } else {
        sacn.dmx(dmxStartAddress, dmxData[dmxStartAddress - 1]);
      }
      sacn.send();
      if (perAddressPriority) {
        sacn.sendDD();
      }
    }
  }

  // OSC output — independent rate, always reads the latest dmxData value
  if (currentMillis - lastOSCSend >= oscInterval) {
    lastOSCSend = currentMillis;
    sendOSC();
  }

  // Sig-Net output — independent rate, mirrors dmxData
  if (currentMillis - lastSigNetSend >= signetInterval) {
    lastSigNetSend = currentMillis;
    sendSigNet();
  }
}