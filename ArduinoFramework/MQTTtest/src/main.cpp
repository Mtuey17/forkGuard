#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define ledPin 32
#define switchPin 12
#define buzzerPin 33
bool switchStatus;
#define serialMonitor Serial
String Message;


const char* ssid = "ForkGuardNet";
const char* password = "Guard1234";
const char* mqtt_server = "Dell.local";
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  setCpuFrequencyMhz(80);  // Reduce power usage
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {

  serialMonitor.begin(115200);
  pinMode(switchPin,INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  switchStatus=false;

  WiFi.begin(ssid, password);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {


  if (!client.connected()){
    Serial.println(client.state());
    Serial.println("waiting for mqtt...");
    delay(500);
    client.connect("ESP32");
  }
  else{
    client.loop();
    switchStatus=bool(digitalRead(switchPin));
  serialMonitor.println(switchStatus);
  if (switchStatus){
    digitalWrite(ledPin,1);
    digitalWrite(buzzerPin,1);
    Message="ON!";
  }
  else{
    digitalWrite(ledPin,0);
    digitalWrite(buzzerPin,0);
    Message="OFF!";
  }
  char toSend[15];
  Message.toCharArray(toSend,sizeof(toSend));
  client.publish("test", toSend);

  }

  

  
}

