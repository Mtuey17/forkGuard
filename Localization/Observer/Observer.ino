/* Matthew Tuer
   - Multi-scan BLE anchor selection (stable)
   - Per-scan de-dupe (use strongest RSSI sample per anchor per scan)
   - EMA smoothing across scans (more stable than simple average)
   - Serial monitor: debug
   - AnchorUart (UART2): ONLY closest anchor name (or "NONE")
   - INT pin pulses HIGH whenever UART data is sent
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;

// ---------------------------
// CONFIG
// ---------------------------
static const int   SCAN_TIME_SECONDS = 5;
static const int   SCANS_PER_REPORT  = 2;
static const int   MAX_DEVICES       = 20;
static const int   MIN_REQUIRED_SEEN = 2;
static const float EMA_ALPHA         = 0.45f;
static const bool  USE_ACTIVE_SCAN   = true;

// ---------------------------
// SECOND UART (name-only output)
// ---------------------------
HardwareSerial AnchorUart(2);
static const int ANCHOR_UART_TX = 12;
static const int ANCHOR_UART_RX = 13;
static const uint32_t ANCHOR_UART_BAUD = 115200;

// ---------------------------
// UART INTERRUPT PIN
// ---------------------------
static const int UART_INT_PIN = 15;              // choose any free GPIO
static const uint32_t UART_INT_PULSE_MS = 10;    // pulse width

// ---------------------------
// Anchor tracking (persistent during window)
// ---------------------------
String anchorNames[MAX_DEVICES];
String anchorAddrs[MAX_DEVICES];
float  anchorEMA[MAX_DEVICES];
int    anchorSeenScans[MAX_DEVICES];
int    anchorCount = 0;

int scanCount = 0;

int findAnchorIndexByAddr(const String& addr) {
  for (int i = 0; i < anchorCount; i++) {
    if (anchorAddrs[i] == addr) return i;
  }
  return -1;
}

void resetAnchorsWindow() {
  for (int i = 0; i < anchorCount; i++) {
    anchorNames[i] = "";
    anchorAddrs[i] = "";
    anchorEMA[i] = -9999.0f;
    anchorSeenScans[i] = 0;
  }
  anchorCount = 0;
  scanCount = 0;
}

void sendAnchorMessage(const String& msg) {
  digitalWrite(UART_INT_PIN, HIGH);
  delay(1);
  AnchorUart.println(msg);
  delay(UART_INT_PULSE_MS);
  digitalWrite(UART_INT_PIN, LOW);
}

void doOneScanAccumulate() {
  Serial.println("Scanning...");

  BLEScanResults* results = pBLEScan->start(SCAN_TIME_SECONDS, false);

  String scanAddrs[MAX_DEVICES];
  String scanNames[MAX_DEVICES];
  int    scanBestRSSI[MAX_DEVICES];
  int    scanUniqueCount = 0;

  int total = results->getCount();

  for (int i = 0; i < total; i++) {
    BLEAdvertisedDevice dev = results->getDevice(i);

    if (!dev.haveName()) continue;

    String name = dev.getName().c_str();
    if (!name.startsWith("Anchor")) continue;

    String addr = dev.getAddress().toString().c_str();
    int rssi = dev.getRSSI();

    int scanIdx = -1;
    for (int j = 0; j < scanUniqueCount; j++) {
      if (scanAddrs[j] == addr) {
        scanIdx = j;
        break;
      }
    }

    if (scanIdx >= 0) {
      if (rssi > scanBestRSSI[scanIdx]) {
        scanBestRSSI[scanIdx] = rssi;
        scanNames[scanIdx] = name;
      }
    } else if (scanUniqueCount < MAX_DEVICES) {
      scanAddrs[scanUniqueCount] = addr;
      scanNames[scanUniqueCount] = name;
      scanBestRSSI[scanUniqueCount] = rssi;
      scanUniqueCount++;
    }
  }

  for (int k = 0; k < scanUniqueCount; k++) {
    const String& addr = scanAddrs[k];
    const String& name = scanNames[k];
    float sample = (float)scanBestRSSI[k];

    int idx = findAnchorIndexByAddr(addr);
    if (idx >= 0) {
      anchorEMA[idx] = EMA_ALPHA * sample + (1.0f - EMA_ALPHA) * anchorEMA[idx];
      anchorSeenScans[idx] += 1;
      anchorNames[idx] = name;
    } else if (anchorCount < MAX_DEVICES) {
      anchorAddrs[anchorCount] = addr;
      anchorNames[anchorCount] = name;
      anchorEMA[anchorCount] = sample;
      anchorSeenScans[anchorCount] = 1;
      anchorCount++;
    }
  }

  scanCount++;
  pBLEScan->clearResults();
}

void reportClosest() {
  if (anchorCount == 0) {
    Serial.println("NONE");
    sendAnchorMessage("NONE");
    return;
  }

  int bestIdx = -1;
  float bestScore = -9999.0f;

  Serial.print("Anchors (EMA): ");
  for (int i = 0; i < anchorCount; i++) {
    Serial.print(anchorNames[i]);
    Serial.print(" ema=");
    Serial.print(anchorEMA[i], 1);
    Serial.print(" seenScans=");
    Serial.print(anchorSeenScans[i]);
    Serial.print(" | ");

    if (anchorSeenScans[i] >= MIN_REQUIRED_SEEN && anchorEMA[i] > bestScore) {
      bestScore = anchorEMA[i];
      bestIdx = i;
    }
  }
  Serial.println();

  if (bestIdx < 0) {
    Serial.println("NONE");
    sendAnchorMessage("NONE");
    return;
  }

  Serial.println(anchorNames[bestIdx]);
  sendAnchorMessage(anchorNames[bestIdx]);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  AnchorUart.begin(ANCHOR_UART_BAUD, SERIAL_8N1, ANCHOR_UART_RX, ANCHOR_UART_TX);

  pinMode(UART_INT_PIN, OUTPUT);
  digitalWrite(UART_INT_PIN, LOW);

  Serial.println("Initializing BLE scan...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();

  pBLEScan->setActiveScan(USE_ACTIVE_SCAN);
  pBLEScan->setInterval(80);
  pBLEScan->setWindow(60);

  resetAnchorsWindow();
}

void loop() {
  doOneScanAccumulate();

  if (scanCount >= SCANS_PER_REPORT) {
    reportClosest();
    resetAnchorsWindow();
  }
}