#include <Arduino.h>
#include <Wire.h>
#define Serialmonitor Serial1

#define SDA 1
#define SCL 2
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serialmonitor.begin(115200,SERIAL_8N1);
  Wire.begin(SDA,SCL);//init I2C bus

}

void loop() {
  // put your main code here, to run repeatedly:
  

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}