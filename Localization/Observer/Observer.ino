/*
Matthew Tuer, June 11, 2024 

V0.2

changed logic 
actually seems to work worse than previous version :)

scanning for one second
recording all anchors, taking an average of all RSSIs 
scanning two more times 
reporting closest anchor 
repeate 

*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEEddystoneURL.h>
#include <BLEEddystoneTLM.h>
#include <BLEBeacon.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "bleNet";
const char* password = "12345678";
const char* mqtt_server = "bleServer.local";

WiFiClient espClient;
PubSubClient client(espClient);

int scanTime = 1;  // in seconds
BLEScan* pBLEScan;

const int maxDevices = 20;
String anchorNames[maxDevices];
int anchorRSSI[maxDevices];
int anchorSeenCount[maxDevices];
int anchorCount = 0;
String previousAnchor = "";

String tagName = "Tag1";
int scanCount=0; 
bool allowSkip=true;
bool skipSend=false;
bool seenPreviousAnchor=false;


void setup_wifi() {
  delay(500);
  setCpuFrequencyMhz(80);  // Reduce power usage
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void sendNearestAnchor() {
  BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);
  //delay(200)
  pBLEScan->stop();
  int totalFoundDevices = foundDevices->getCount();
  Serial.println("Scanning...");

  for (int i = 0; i < totalFoundDevices; i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    if (device.haveName()) {
      String devName = device.getName().c_str();

      if (devName.startsWith("Anchor")) {
        bool alreadyStored = false;

        for (int j = 0; j < anchorCount; j++) {
          if (anchorNames[j] == devName) {
            alreadyStored = true;
            anchorRSSI[j] += device.getRSSI();  // update RSSI if already exists
            anchorSeenCount[j]++;
            break;
          }
        }

        if (!alreadyStored && anchorCount < maxDevices) {
          anchorNames[anchorCount] = devName;
          anchorRSSI[anchorCount]+= device.getRSSI();
          
          anchorCount++;
        }
      }
    }
  }

  scanCount++;
  if (scanCount==4){
    int maxRSSI = anchorRSSI[0]/1;
    int maxIndex = 0;
  Serial.print("Current Anchors: ");
  for (int i = 0; i < anchorCount; i++) {

    if (((anchorRSSI[i]/anchorSeenCount[i]) > maxRSSI)) {
    maxRSSI = anchorRSSI[i]/anchorSeenCount[i];
    maxIndex = i;
    }

    Serial.print(anchorNames[i]);
    Serial.print(" (RSSI: ");
    Serial.print(anchorRSSI[i]);
    Serial.print("),");
    Serial.print("seen: ");
    Serial.print(anchorSeenCount[i]);
    Serial.print(", ");

    if (anchorNames[i]==previousAnchor){
      seenPreviousAnchor=true; 
    }
  }

  bool anchorConfidence=false;
  float maxAnchor=anchorRSSI[maxIndex]/anchorSeenCount[maxIndex];
  for (int i = 0; i < anchorCount; i++) {

    float acceptableDifference=3.0;
    float currentAnchor=anchorRSSI[i]/anchorSeenCount[i];
    float difference = abs(currentAnchor-maxAnchor);
    //Serial.println(difference);
    if (anchorCount<=1){
      anchorConfidence=true;
      break;

    }
    if (difference>=acceptableDifference){
      anchorConfidence=true;
    }
    else if (difference<acceptableDifference&&i!=maxIndex){
      anchorConfidence=false;
      //Serial.println("no anchor confidence");
      break;
    }


  }



   
  Serial.println();
  //Serial.print("Max RSSI: ");
  //Serial.println(String(anchorNames[maxIndex]));
  
  //skipping publish ONCE if
  //only 1 anchor is seen
  //previous closest anchor is not seen at all 
  if ((anchorCount<=1&&allowSkip||!seenPreviousAnchor&&allowSkip)||!anchorConfidence){
    skipSend=true;
    allowSkip=false ;
    Serial.println("skipping send...");
  }

  if (!skipSend){
  
  String msg = tagName + ":" + anchorNames[maxIndex];
  char msgBuffer[50];
  Serial.println("MQTT Sent: " + msg);
  msg.toCharArray(msgBuffer, sizeof(msgBuffer));
  client.publish("bleLocalization/Tags", msgBuffer);
  allowSkip=true;
  }
  else{
    skipSend=false;
  }
  previousAnchor=anchorNames[maxIndex];
  



  //clearing arrays 
  for (int i = 0; i < anchorCount; i++) {
    anchorNames[i]="";
    anchorRSSI[i]=0;
    anchorSeenCount[i]=1;
  }
  scanCount=0;
  anchorCount=0;
  seenPreviousAnchor=false; 

  }


  pBLEScan->clearResults();
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  Serial.println("Initializing BLE scan...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(50);

  
  client.setServer(mqtt_server, 1883);

  for (int i = 0; i < maxDevices; i++) {
  anchorSeenCount[i] = 1;
}

}

void loop() {
  if (!client.connected()) {
    Serial.println("MQTT not connected...");
    client.connect("Tag1");
    scanCount=0;
  anchorCount=0;
  } else {
    client.loop();
    sendNearestAnchor();
  }

  //delay(2000);  // BLE scan every 2 seconds
}
