/* Matthew Tuer
   - Multi-scan BLE anchor selection (stable)
   - Per-scan de-dupe (use strongest RSSI sample per anchor per scan)
   - EMA smoothing across scans (more stable than simple average)
   - Serial monitor: debug
   - AnchorUart (UART2): ONLY closest anchor name (or "NONE")
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;

// ---------------------------
// CONFIG
// ---------------------------
static const int   SCAN_TIME_SECONDS = 5;     // length of each BLE scan
static const int   SCANS_PER_REPORT  = 2;     // how many scans per decision
static const int   MAX_DEVICES       = 20;    // max anchors tracked
static const int   MIN_REQUIRED_SEEN = 2;     // must show up in at least this many scans (per report window)
static const float EMA_ALPHA         = 0.45f; // 0..1 (higher = reacts faster, lower = smoother)

// If you want more range/accuracy, active scan helps but costs power/time.
// For just RSSI, passive often works fine too.
static const bool  USE_ACTIVE_SCAN   = true;

// ---------------------------
// SECOND UART (name-only output)
// ---------------------------
HardwareSerial AnchorUart(2);
static const int ANCHOR_UART_TX = 17;       // <-- set to your wired TX pin
static const int ANCHOR_UART_RX = 16;       // <-- set to your wired RX pin (optional if not reading)
static const uint32_t ANCHOR_UART_BAUD = 115200;

// ---------------------------
// Anchor tracking (persistent during window)
// ---------------------------
String anchorNames[MAX_DEVICES];
String anchorAddrs[MAX_DEVICES];      // track by BLE address (more reliable than name)
float  anchorEMA[MAX_DEVICES];        // smoothed RSSI
int    anchorSeenScans[MAX_DEVICES];  // how many scans (within the current window) it appeared in
int    anchorCount = 0;

int scanCount = 0;

// Find index by address; returns -1 if not found
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

/*
  Do one BLE scan and gather ONE sample per anchor for this scan:
  - If an anchor appears multiple times in the scan results, keep only the strongest RSSI sample.
*/
void doOneScanAccumulate() {
  Serial.println("Scanning...");

  // Blocking scan; it auto-stops after SCAN_TIME_SECONDS
  BLEScanResults* results = pBLEScan->start(SCAN_TIME_SECONDS, false);

  // Per-scan de-dupe buffers (strongest RSSI per anchor during this scan)
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

    // Use address as the unique key (best practice)
    String addr = dev.getAddress().toString().c_str();
    int rssi = dev.getRSSI();

    // Find in per-scan unique list
    int scanIdx = -1;
    for (int j = 0; j < scanUniqueCount; j++) {
      if (scanAddrs[j] == addr) { scanIdx = j; break; }
    }

    if (scanIdx >= 0) {
      // Keep the strongest RSSI sample this scan
      if (rssi > scanBestRSSI[scanIdx]) {
        scanBestRSSI[scanIdx] = rssi;
        scanNames[scanIdx] = name; // update name if it changes
      }
    } else if (scanUniqueCount < MAX_DEVICES) {
      scanAddrs[scanUniqueCount] = addr;
      scanNames[scanUniqueCount] = name;
      scanBestRSSI[scanUniqueCount] = rssi;
      scanUniqueCount++;
    }
  }

  // Apply this scan’s samples to the window trackers (EMA)
  for (int k = 0; k < scanUniqueCount; k++) {
    const String& addr = scanAddrs[k];
    const String& name = scanNames[k];
    float sample = (float)scanBestRSSI[k];

    int idx = findAnchorIndexByAddr(addr);
    if (idx >= 0) {
      // EMA update
      anchorEMA[idx] = EMA_ALPHA * sample + (1.0f - EMA_ALPHA) * anchorEMA[idx];
      anchorSeenScans[idx] += 1;
      // keep latest name
      anchorNames[idx] = name;
    } else if (anchorCount < MAX_DEVICES) {
      anchorAddrs[anchorCount] = addr;
      anchorNames[anchorCount] = name;
      anchorEMA[anchorCount] = sample;        // initialize EMA with first sample
      anchorSeenScans[anchorCount] = 1;
      anchorCount++;
    }
  }

  scanCount++;
  pBLEScan->clearResults();
}

/*
  Pick best anchor by highest EMA RSSI (closest-ish),
  requiring it was seen in at least MIN_REQUIRED_SEEN scans in this window.
  - Serial: debug
  - AnchorUart: ONLY name (or NONE)
*/
void reportClosest() {
  if (anchorCount == 0) {
    Serial.println("NONE");
    AnchorUart.println("NONE");
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
    AnchorUart.println("NONE");
    return;
  }

  // Keep your current monitor behavior (prints the winner)
  Serial.println(anchorNames[bestIdx]);

  // Separate UART: name only
  AnchorUart.println(anchorNames[bestIdx]);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Secondary UART for name-only output
  AnchorUart.begin(ANCHOR_UART_BAUD, SERIAL_8N1, ANCHOR_UART_RX, ANCHOR_UART_TX);

  Serial.println("Initializing BLE scan...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();

  pBLEScan->setActiveScan(USE_ACTIVE_SCAN);

  // Tuning: window <= interval. Bigger values scan more, but cost power/CPU.
  // These are reasonable defaults.
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
