#if defined(ESP32)
#define __ESP32_MQTT_H__
#endif

//**************************************  starts here **************************************
#ifdef __ESP32_MQTT_H__
#include "esp32-mqtt.h"


#define SENSOR 17 // pin 17 for the sensor 
#define POS_1 5
#define POS_2 4
#define POS_3 2       

float calibrationFactor = 7.14;
int interval = 1000; // for printing the pulseCount
long pulseCount;
long delta, tensec;
float flowRate;
unsigned int flowMilliLitres;
float totalMilliLitres;
long currentMillis = 0;
long previousMillis = 0;
long startMillis = 0;
int count = 0;
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}


void setup() {

  Serial.begin(115200);
  pinMode(SENSOR, INPUT);
  digitalWrite(SENSOR, HIGH);
  pinMode(POS_1, INPUT);
  pinMode(POS_2, INPUT);
  pinMode(POS_3, INPUT);
  if(digitalRead(POS_1) == HIGH)
  {
    calibrationFactor = 1;
  }
  else if (digitalRead(POS_2) == HIGH)
  {
    calibrationFactor = 2;
  }
  else if (digitalRead(POS_3) == HIGH)
  {
    calibrationFactor = 3;
  }
  
  setupCloudIoT();
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  startMillis = millis();

}



unsigned long lastMillis = 0;
void loop() {

  mqtt->loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
  }

  currentMillis = millis();
  delta = currentMillis - previousMillis;

  if (delta > interval) {

    Serial.println( pulseCount);


    flowRate = ((1000.0 / (millis() - previousMillis)) * pulseCount) / calibrationFactor;
    previousMillis = millis();

    flowMilliLitres = (flowRate / 60) * 1000;

    totalMilliLitres += flowMilliLitres;

    unsigned int frac;

    frac = (flowRate - int(flowRate)) * 10;

    //Serial.print("  Output Liquid Quantity: ");             // Output separator
    //Serial.print(totalMilliLitres);
    //Serial.println("mL");


    pulseCount = 0;

  }

  // this data
  // TODO: replace with your code
  // publish a message roughly every second.cxcx
  if (millis() - lastMillis > 600) {
    String tosend = String(int(totalMilliLitres));
    
    lastMillis = millis();
    //tosend = String(1000);      
   //publishTelemetry( "/raw_data", );                                                                                                                                                  
   count = count + 1;
     publishTelemetry(String(lastMillis));
  }
  //  datasend++;
}
#endif //  __ESP32_MQTT_H__
