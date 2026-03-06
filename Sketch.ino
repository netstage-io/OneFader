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

// Native USB-C port on ESP32-S3 uses USB CDC, not UART
USBCDC USBSerial;

// W5500 Ethernet pins for Waveshare ESP32-S3-ETH
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
String firmwareVersion = "v1.6.0";
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

// Unicast settings
bool useUnicast = false;
IPAddress unicastTarget(0, 0, 0, 0);

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
  return invertFader ? (65535 - raw16) : raw16;
}

uint8_t readFaderValue() {
  if (simulationMode) {
    uint8_t value = (uint8_t)map(simulatedFaderValue, 0, 100, 0, 255);
    return invertFader ? (255 - value) : value;
  }
  // Derive 8-bit from 16-bit by taking the high byte — this guarantees
  // 0 maps to 0 and 255 maps to 255 with no off-by-one from integer division
  uint16_t raw16 = readFaderValue16bit();
  // If invert is applied, raw16 is already inverted — just take high byte
  return (uint8_t)(raw16 >> 8);
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
    sacn.begin(unicastTarget, universe, sacnPriority);
    addLog("sACN Unicast -> " + unicastTarget.toString());
  } else {
    sacn.begin(universe, sacnPriority);
    addLog("sACN Multicast");
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
        <title>%STREAM_NAME% - Configuration</title>
        <style>
          * { margin: 0; padding: 0; box-sizing: border-box; }
          body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #333;
          }
          .header { text-align: center; margin-bottom: 30px; color: white; }
          .header h1 { font-size: 2.5rem; font-weight: 700; margin-bottom: 10px; text-shadow: 0 2px 10px rgba(0,0,0,0.2); }
          .header .subtitle { font-size: 1rem; opacity: 0.9; }
          .header .subtitle a { color: white; text-decoration: none; border-bottom: 1px solid rgba(255,255,255,0.5); transition: all 0.3s; }
          .header .subtitle a:hover { border-bottom-color: white; }
          .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); gap: 24px; max-width: 1400px; margin: 0 auto; }
          .card { background: rgba(255,255,255,0.95); backdrop-filter: blur(10px); border-radius: 20px; padding: 28px; box-shadow: 0 8px 32px rgba(0,0,0,0.1); transition: transform 0.2s, box-shadow 0.2s; }
          .card:hover { transform: translateY(-2px); box-shadow: 0 12px 48px rgba(0,0,0,0.15); }
          .card-header { font-size: 1.4rem; font-weight: 600; margin-bottom: 20px; color: #667eea; display: flex; align-items: center; gap: 10px; }
          .info-grid { display: grid; gap: 12px; margin-bottom: 20px; }
          .info-item { display: flex; justify-content: space-between; align-items: center; padding: 12px; background: #f8f9fa; border-radius: 10px; font-size: 0.9rem; }
          .info-label { font-weight: 600; color: #666; }
          .info-value { color: #333; font-family: 'Courier New', monospace; font-size: 0.85rem; }
          .form-group { margin-bottom: 20px; }
          label { display: block; margin-bottom: 8px; font-weight: 600; color: #555; font-size: 0.9rem; }
          input[type="text"], input[type="number"], select { width: 100%; padding: 12px 16px; margin-bottom: 4px; border: 2px solid #e0e0e0; border-radius: 12px; font-size: 1rem; transition: all 0.3s; background: white; }
          input:focus, select:focus { outline: none; border-color: #667eea; box-shadow: 0 0 0 3px rgba(102,126,234,0.1); }
          button { width: 100%; padding: 14px; margin-bottom: 12px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; border: none; border-radius: 12px; font-size: 1rem; font-weight: 600; cursor: pointer; transition: all 0.3s; box-shadow: 0 4px 15px rgba(102,126,234,0.4); }
          button:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(102,126,234,0.6); }
          button:active { transform: translateY(0); }
          .fader-display { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 24px; border-radius: 16px; text-align: center; color: white; margin-bottom: 20px; box-shadow: 0 8px 32px rgba(102,126,234,0.3); }
          .fader-value { font-size: 3rem; font-weight: 700; margin-bottom: 8px; }
          .fader-label { font-size: 0.9rem; opacity: 0.9; margin-bottom: 16px; }
          .mode-indicator { display: inline-block; padding: 6px 14px; border-radius: 20px; font-weight: 600; font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.5px; margin-top: 8px; }
          .mode-simulation { background: #fbbf24; color: #78350f; }
          .mode-hardware { background: #34d399; color: #065f46; }
          .fader-bar { width: 100%; height: 12px; background: rgba(255,255,255,0.3); border-radius: 6px; overflow: hidden; margin-top: 16px; }
          .fader-fill { height: 100%; background: linear-gradient(90deg, #34d399, #fbbf24, #f87171); transition: width 0.15s ease-out; border-radius: 6px; }
          input[type="range"] { width: 100%; height: 8px; border-radius: 4px; background: #e0e0e0; outline: none; -webkit-appearance: none; margin: 16px 0; }
          input[type="range"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 24px; height: 24px; border-radius: 50%; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); cursor: pointer; box-shadow: 0 2px 10px rgba(102,126,234,0.5); }
          input[type="range"]::-moz-range-thumb { width: 24px; height: 24px; border-radius: 50%; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); cursor: pointer; border: none; box-shadow: 0 2px 10px rgba(102,126,234,0.5); }
          .slider-value { text-align: center; font-size: 1.2rem; font-weight: 600; color: #667eea; margin-top: 8px; }
          .network-status { background: linear-gradient(135deg, #34d399 0%, #059669 100%); color: white; padding: 16px; border-radius: 12px; margin-bottom: 20px; font-weight: 500; text-align: center; box-shadow: 0 4px 15px rgba(52,211,153,0.3); }
          .logs { background: #1e293b; color: #e2e8f0; padding: 16px; border-radius: 12px; font-family: 'Courier New', monospace; font-size: 0.8rem; height: 250px; overflow-y: auto; white-space: pre-wrap; line-height: 1.5; }
          .logs::-webkit-scrollbar { width: 8px; }
          .logs::-webkit-scrollbar-track { background: #334155; border-radius: 4px; }
          .logs::-webkit-scrollbar-thumb { background: #667eea; border-radius: 4px; }
          .button-secondary { background: linear-gradient(135deg, #64748b 0%, #475569 100%); box-shadow: 0 4px 15px rgba(100,116,139,0.4); margin-top: 16px; }
          .button-danger { background: linear-gradient(135deg, #f87171 0%, #dc2626 100%); box-shadow: 0 4px 15px rgba(248,113,113,0.4); }
          .info-text { font-size: 0.85rem; color: #666; margin-top: 8px; margin-bottom: 24px; padding: 8px 12px; background: #f8f9fa; border-radius: 8px; line-height: 1.4; }
          @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } .header h1 { font-size: 2rem; } }
        </style>
      </head>
      <body>
        <div class="header">
          <h1>OneFader</h1>
          <div class="subtitle">Created by <a href="https://netstage.io" target="_blank">netstage.io</a></div>
        </div>

        <div class="grid">

          <!-- Device & sACN Settings -->
          <div class="card">
            <div class="card-header">⚙️ Device Settings</div>
            <div class="info-grid">
              <div class="info-item">
                <span class="info-label">Serial Number</span>
                <span class="info-value">%DEVICE_SERIAL%</span>
              </div>
              <div class="info-item">
                <span class="info-label">Firmware</span>
                <span class="info-value">%FIRMWARE_VERSION%</span>
              </div>
              <div class="info-item">
                <span class="info-label">sACN CID</span>
                <span class="info-value" style="font-size:0.7rem;">%SACN_CID%</span>
              </div>
            </div>

            <form action="/updateStreamName" method="POST">
              <div class="form-group">
                <label>Stream Name</label>
                <input type="text" name="streamName" value="%STREAM_NAME%">
                <p style="font-size:0.78rem;color:#888;margin-top:6px;">⚠️ Changing the name requires a reboot to take effect on the network.</p>
              </div>
              <button type="submit">Update Name &amp; Reboot</button>
            </form>

            <form action="/updateUniverse" method="POST">
              <div class="form-group">
                <label>Universe (1-63999)</label>
                <input type="number" name="universe" value="%UNIVERSE%" min="1" max="63999">
              </div>
              <button type="submit">Update Universe</button>
            </form>

            <form action="/updateStartAddress" method="POST">
              <div class="form-group">
                <label>DMX Start Address (1-512)</label>
                <input type="number" name="dmxStartAddress" value="%DMX_START_ADDRESS%" min="1" max="512">
              </div>
              <button type="submit">Update Address</button>
            </form>

            <form action="/toggle16bit" method="POST">
              <button type="submit" class="button-secondary">%BIT_MODE_TEXT%</button>
            </form>
            <p style="font-size:0.85rem;color:#666;margin-top:-10px;">%BIT_MODE_INFO%</p>

            <form action="/updatePriority" method="POST">
              <div class="form-group">
                <label>sACN Priority (0-200)</label>
                <input type="number" name="sacnPriority" value="%SACN_PRIORITY%" min="0" max="200">
              </div>
              <button type="submit">Update Priority</button>
            </form>

            <form action="/toggleUnicast" method="POST">
              <button type="submit" class="button-secondary">%UNICAST_MODE_TEXT%</button>
            </form>

            <div style="display:%UNICAST_DISPLAY%;">
              <form action="/updateUnicast" method="POST">
                <div class="form-group">
                  <label>Unicast Target IP</label>
                  <input type="text" name="unicastIP" value="%UNICAST_IP%" placeholder="192.168.1.100">
                </div>
                <button type="submit">Set Target IP</button>
              </form>
              <p style="font-size:0.82rem;color:#888;margin-top:8px;line-height:1.5;">
                Unicast mode sends only to the target IP. Multicast is automatically suppressed.
              </p>
            </div>
          </div>

          <!-- Fader Control -->
          <div class="card">
            <div class="card-header">🎚️ Fader Control</div>

            <div class="fader-display">
              <div class="fader-value" id="faderValue">0</div>
              <div class="fader-label">Fader Level</div>
              <div class="fader-bar">
                <div class="fader-fill" id="faderFill" style="width:0%"></div>
              </div>
              <span class="mode-indicator mode-simulation" id="modeIndicator">SIMULATION</span>
            </div>

            <form action="/toggleSimulation" method="POST">
              <button type="submit" class="button-secondary">%SIMULATION_MODE_TEXT%</button>
            </form>

            <form action="/toggleInvert" method="POST" style="margin-top:16px;">
              <button type="submit" class="button-secondary">%INVERT_MODE_TEXT%</button>
            </form>
            <p class="info-text">%INVERT_INFO%</p>

            <div id="sliderContainer">
              <div class="form-group">
                <label>Simulate Fader Position</label>
                <input type="range" id="faderSlider" min="0" max="100" value="0">
                <div class="slider-value" id="sliderValue">0%</div>
              </div>
            </div>

            <form action="/toggleTestMode" method="POST" style="margin-top:12px;">
              <button type="submit" class="button-secondary">%TEST_MODE_TEXT%</button>
            </form>

          </div>

          <!-- Network Settings -->
          <div class="card">
            <div class="card-header">🌐 Network Settings</div>

            <div class="network-status" id="networkStatus">%NETWORK_STATUS%</div>

            <form action="/dhcpToggle" method="POST">
              <button type="submit" class="button-secondary">%DHCP_MODE_TEXT%</button>
            </form>

            <form action="/updateIPSettings" method="POST">
              <div class="form-group">
                <label>Static IP</label>
                <input type="text" name="staticIP" value="%STATIC_IP%">
              </div>
              <div class="form-group">
                <label>Subnet Mask</label>
                <input type="text" name="subnet" value="%SUBNET%">
              </div>
              <div class="form-group">
                <label>Gateway</label>
                <input type="text" name="gateway" value="%GATEWAY%">
              </div>
              <div class="form-group">
                <label>DNS</label>
                <input type="text" name="dns" value="%DNS%">
              </div>
              <button type="submit">Update IP Settings</button>
            </form>
          </div>

          <!-- Input Tuning -->
          <div class="card">
            <div class="card-header">🎛️ Input Tuning</div>

            <form action="/toggleEMA" method="POST">
              <button type="submit" class="button-secondary">%EMA_MODE_TEXT%</button>
            </form>
            <p class="info-text">%EMA_MODE_INFO%</p>
            <div style="display:%EMA_DISPLAY%;">
              <form action="/updateEMAAlpha" method="POST">
                <div class="form-group">
                  <label>Smoothing Amount (1=max smooth, 100=off)</label>
                  <input type="number" name="emaAlpha" value="%EMA_ALPHA%" min="1" max="100">
                </div>
                <button type="submit">Update Smoothing</button>
              </form>
            </div>

            <form action="/updateSpikeThreshold" method="POST" style="margin-top:8px;">
              <div class="form-group">
                <label>Spike Rejection (0=off, 1-500)</label>
                <input type="number" name="spikeThreshold" value="%SPIKE_THRESHOLD%" min="0" max="500">
                <p style="font-size:0.78rem;color:#888;margin-top:6px;">Discards sudden jumps larger than this value. Prevents random glitches from reaching the output. 200 is a good starting point — lower for stricter rejection, 0 to disable.</p>
              </div>
              <button type="submit">Update Spike Rejection</button>
            </form>

            <form action="/updateDeadband" method="POST" style="margin-top:8px;">
              <div class="form-group">
                <label>Deadband (ADC counts)</label>
                <input type="number" name="deadband" value="%DEADBAND%" min="0" max="200">
                <p style="font-size:0.78rem;color:#888;margin-top:6px;">Minimum change required to update the output. Increase to reduce noise at rest, decrease for finer response. 30 is a good starting point.</p>
              </div>
              <button type="submit">Update Deadband</button>
            </form>

            <div style="margin-top:20px;padding-top:20px;border-top:1px solid #e8e8f0;">
              <div class="card-header" style="margin-bottom:8px;font-size:1.1rem;">📐 Fader Calibration</div>
              <div class="info-grid" style="margin-bottom:16px;">
                <div class="info-item">
                  <span class="info-label">Cal Min (ADC)</span>
                  <span class="info-value">%CAL_MIN%</span>
                </div>
                <div class="info-item">
                  <span class="info-label">Cal Max (ADC)</span>
                  <span class="info-value">%CAL_MAX%</span>
                </div>
              </div>
              <a href="/calibrate" style="display:block;text-decoration:none;">
                <button type="button" class="button-secondary" style="margin-top:0;">Calibrate Fader</button>
              </a>
            </div>
          </div>

          <!-- System Actions -->
          <div class="card">
            <div class="card-header">🔧 System Actions</div>

            <form action="/upload" method="POST" enctype="multipart/form-data">
              <div class="form-group">
                <label>Firmware Update (OTA)</label>
                <input type="file" name="update" accept=".bin">
                <p style="font-size:0.78rem;color:#888;margin-top:6px;">⚠️ Use <strong>Sketch.ino.bin</strong> (app only) — not the merged binary.</p>
              </div>
              <button type="submit">Upload Firmware</button>
            </form>

            <form action="/toggleWebAuth" method="POST">
              <button type="submit" class="button-secondary">%WEB_AUTH_TOGGLE_TEXT%</button>
            </form>
            <p class="info-text">%WEB_AUTH_INFO%</p>

            <div style="display:%WEB_AUTH_DISPLAY%;">
              <form action="/updateWebAuth" method="POST">
                <div class="form-group">
                  <label>Web UI Username</label>
                  <input type="text" name="webUsername" value="%WEB_USERNAME%" placeholder="admin">
                </div>
                <div class="form-group">
                  <label>Web UI Password</label>
                  <input type="text" name="webPassword" value="%WEB_PASSWORD%" placeholder="Enter password">
                </div>
                <button type="submit">Update Credentials</button>
              </form>
            </div>

            <form action="/updateOTAPassword" method="POST">
              <div class="form-group">
                <label>OTA Password</label>
                <input type="text" name="otaPassword" value="%OTA_PASSWORD%" placeholder="Enter OTA password">
              </div>
              <button type="submit" class="button-secondary">Update OTA Password</button>
            </form>

            <form action="/reboot" method="POST" style="margin-top:8px;">
              <button type="submit" class="button-secondary">Reboot Device</button>
            </form>

            <form action="/reset" method="POST" style="margin-top:8px;"
                  onsubmit="return confirm('Are you sure? This will erase all settings!');">
              <button type="submit" class="button-danger">Factory Reset</button>
            </form>

            <div style="margin-top:24px;">
              <label style="margin-bottom:12px;">Debug Logs</label>
              <div class="logs" id="logs">%LOGS%</div>
            </div>
          </div>

        </div>

        <script>
          const slider = document.getElementById('faderSlider');
          const sliderValue = document.getElementById('sliderValue');
          const sliderContainer = document.getElementById('sliderContainer');
          const modeIndicator = document.getElementById('modeIndicator');

          slider.addEventListener('input', function() {
            sliderValue.innerText = this.value + '%';
            fetch('/setFader?value=' + this.value);
          });

          function updateModeUI(isSimulation) {
            sliderContainer.style.display = isSimulation ? 'block' : 'none';
            modeIndicator.innerText = isSimulation ? 'SIMULATION' : 'HARDWARE';
            modeIndicator.className = 'mode-indicator ' + (isSimulation ? 'mode-simulation' : 'mode-hardware');
          }

          setInterval(() => {
            fetch('/logs').then(r => r.text()).then(d => {
              document.getElementById('logs').innerText = d;
            });
            fetch('/faderPercent').then(r => r.text()).then(d => {
              const pct = parseInt(d);
              document.getElementById('faderValue').innerText = pct + '%';
              document.getElementById('faderFill').style.width = pct + '%';
            });
            fetch('/getMode').then(r => r.text()).then(d => updateModeUI(d === 'simulation'));

            fetch('/networkStatus').then(r => r.text()).then(d => {
              document.getElementById('networkStatus').innerHTML = d;
            });
          }, 100);

          fetch('/getMode').then(r => r.text()).then(d => updateModeUI(d === 'simulation'));

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
    html.replace("%UNICAST_MODE_TEXT%",    useUnicast ? "Switch to Multicast" : "Enable Unicast Mode");
    html.replace("%UNICAST_DISPLAY%",      useUnicast ? "block" : "none");
    html.replace("%UNICAST_IP%",           unicastTarget.toString());

    html.replace("%TEST_MODE_TEXT%",       testModeEnabled ? "Disable Test Mode" : "Enable Test Mode");
    html.replace("%EMA_MODE_TEXT%",        emaEnabled ? "Disable Input Smoothing" : "Enable Input Smoothing");
    html.replace("%EMA_MODE_INFO%",        emaEnabled ? "Smoothing active — reduces fader noise and jitter." : "Smoothing disabled — raw ADC input.");
    html.replace("%EMA_DISPLAY%",          emaEnabled ? "block" : "none");
    html.replace("%EMA_ALPHA%",            String(emaAlpha));
    html.replace("%SPIKE_THRESHOLD%",      String(spikeThreshold));
    html.replace("%DEADBAND%",              String(deadband));
    html.replace("%CAL_MIN%",               String(adcCalMin));
    html.replace("%CAL_MAX%",               String(adcCalMax));
    html.replace("%SIMULATION_MODE_TEXT%", simulationMode  ? "Switch to Hardware Mode" : "Switch to Simulation Mode");
    html.replace("%INVERT_MODE_TEXT%",     invertFader ? "Disable Fader Invert" : "Enable Fader Invert");
    html.replace("%INVERT_INFO%",          invertFader ? "Fader is INVERTED (full = 0, off = 255)" : "Fader is normal (off = 0, full = 255)");
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
    }
    request->redirect("/");
  });

  server.on("/toggle16bit", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    use16bit = !use16bit;
    preferences.putBool("use16bit", use16bit);
    addLog(use16bit ? "16-bit mode" : "8-bit mode");
    resetDMXOutput();
    request->redirect("/");
  });

  server.on("/toggleUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    useUnicast = !useUnicast;
    preferences.putBool("useUnicast", useUnicast);
    sacnBegin();
    request->redirect("/");
  });

  server.on("/updateUnicast", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    if (request->hasParam("unicastIP", true)) {
      String ipStr = request->getParam("unicastIP", true)->value();
      if (unicastTarget.fromString(ipStr)) {
        preferences.putUInt("unicastIP", (uint32_t)unicastTarget);
        if (useUnicast) sacnBegin();
        addLog("Unicast target: " + unicastTarget.toString());
      } else {
        addLog("Invalid IP: " + ipStr);
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
        </div>
        <script>
          function fmt(s) {
            var m = Math.floor(s / 60);
            var sec = s % 60;
            return m + ':' + (sec < 10 ? '0' : '') + sec;
          }
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
                  ",\"secsLeft\":" + String(secsLeft) + "}";
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
  useUnicast      = preferences.getBool("useUnicast",      false);
  use16bit        = preferences.getBool("use16bit",        false);
  invertFader     = preferences.getBool("invertFader",     false);

  uint32_t savedUnicast = preferences.getUInt("unicastIP", 0);
  if (savedUnicast != 0) unicastTarget = IPAddress(savedUnicast);

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

  addLog("Universe: "   + String(universe));
  addLog("DMX Addr: "   + String(dmxStartAddress));
  addLog("Priority: "   + String(sacnPriority));
  addLog("DMX Mode: "   + String(use16bit ? "16-bit" : "8-bit"));
  addLog("Rate: 40Hz");

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
      // Scale percent directly, applying invert if set
      if (use16bit) {
        uint16_t val16 = (uint16_t)map(testPercent, 0, 100, 0, 65535);
        if (invertFader) val16 = 65535 - val16;
        dmxData[dmxStartAddress - 1] = (val16 >> 8) & 0xFF; // coarse
        if (dmxStartAddress <= 511) dmxData[dmxStartAddress] = val16 & 0xFF; // fine
      } else {
        uint8_t val8 = (uint8_t)map(testPercent, 0, 100, 0, 255);
        if (invertFader) val8 = 255 - val8;
        dmxData[dmxStartAddress - 1] = val8;
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

    sacn.dmx(dmxData);
    if (use16bit) {
      sacn.dmx(dmxStartAddress,     dmxData[dmxStartAddress - 1]); // coarse
      sacn.dmx(dmxStartAddress + 1, dmxData[dmxStartAddress]);     // fine
    } else {
      sacn.dmx(dmxStartAddress, dmxData[dmxStartAddress - 1]);
    }
    sacn.send();
  }
}
