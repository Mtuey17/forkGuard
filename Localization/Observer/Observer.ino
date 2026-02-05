/*
Matthew Tuer


- Scan multiple times
- Average RSSI for each Anchor across scans
- Output closest Anchor through uart 
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;


static const int SCAN_TIME_SECONDS   = ;   // length of each BLE scan
static const int SCANS_PER_REPORT    = 4;   // number of scans to average before reporting
static const int MAX_DEVICES         = 20;  // max anchors to track
static const int MIN_REQUIRED_SEEN   = 1;   // must see an anchor at least this many times


String anchorNames[MAX_DEVICES];
int    anchorRSSISum[MAX_DEVICES];
int    anchorSeenCount[MAX_DEVICES];
int    anchorCount = 0;

int scanCount = 0;

// Find index of anchor name; returns -1 if not found
int findAnchorIndex(const String& name) {
  for (int i = 0; i < anchorCount; i++) {
    if (anchorNames[i] == name) return i;
  }
  return -1;
}

void resetAnchors() {
  for (int i = 0; i < anchorCount; i++) {
    anchorNames[i] = "";
    anchorRSSISum[i] = 0;
    anchorSeenCount[i] = 0;
  }
  anchorCount = 0;
  scanCount = 0;
}

void doOneScanAccumulate() {
  Serial.println("Scanning...");

  BLEScanResults* results = pBLEScan->start(SCAN_TIME_SECONDS, false);
  pBLEScan->stop();

  int total = results->getCount();

  for (int i = 0; i < total; i++) {
    BLEAdvertisedDevice dev = results->getDevice(i);

    if (!dev.haveName()) continue;

    String name = dev.getName().c_str();
    if (!name.startsWith("Anchor")) continue;

    int rssi = dev.getRSSI();

    int idx = findAnchorIndex(name);
    if (idx >= 0) {
      anchorRSSISum[idx] += rssi;
      anchorSeenCount[idx] += 1;
    } else if (anchorCount < MAX_DEVICES) {
      anchorNames[anchorCount]    = name;
      anchorRSSISum[anchorCount]  = rssi;
      anchorSeenCount[anchorCount]= 1;
      anchorCount++;
    }
  }

  scanCount++;

  pBLEScan->clearResults();
}

void reportClosestOverUart() {
  if (anchorCount == 0) {
    Serial.println("NONE");   // nothing seen
    return;
  }

  int bestIdx = -1;
  float bestAvg = -9999.0f;

  // debug print all anchors + averages
  Serial.print("Anchors: ");
  for (int i = 0; i < anchorCount; i++) {
    if (anchorSeenCount[i] <= 0) continue;
    float avg = (float)anchorRSSISum[i] / (float)anchorSeenCount[i];

    Serial.print(anchorNames[i]);
    Serial.print(" avg=");
    Serial.print(avg, 1);
    Serial.print(" seen=");
    Serial.print(anchorSeenCount[i]);
    Serial.print(" | ");

    if (anchorSeenCount[i] >= MIN_REQUIRED_SEEN && avg > bestAvg) {
      bestAvg = avg;
      bestIdx = i;
    }
  }
  Serial.println();

  if (bestIdx < 0) {
    Serial.println("NONE");
    return;
  }

  // ---- THIS is the UART output you want: just the closest anchor name ----
  Serial.println(anchorNames[bestIdx]);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Initializing BLE scan...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);

  // Your old values (fast scanning)
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(50);

  resetAnchors();
}

void loop() {
  doOneScanAccumulate();

  if (scanCount >= SCANS_PER_REPORT) {
    reportClosestOverUart();
    resetAnchors();          // start a fresh averaging window
  }

  // optional small pause between scans
  // delay(50);
}
