#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <DHT_U.h>
#include <Adafruit_HMC5883_U.h>
#include <RH_ASK.h>

#define DHTPIN 4
#define HALLPIN 5
#define RXPIN 7
#define TXPIN 6
#define DHTTYPE DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
RH_ASK ask(2000, RXPIN, TXPIN);

uint32_t delayMS;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dht.begin();
  sensor_t sensor;
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  if(!bmp.begin()) {
    Serial.println("No BMP pressure sensor detected");
    while(1);
  }
  if(!mag.begin()) {
    Serial.println("No HMC magnetometer sensor detected");
    while(1);
  }
  if(!ask.init()) {
    Serial.println("No ASK transmitter detected");
    while(1);
  }
  pinMode(HALLPIN, INPUT);
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // put your main code here, to run repeatedly:
  static bool bFlag = true;
  static float fOldTemperature = 0, fOldHumidity = 0,  fOldPressure = 0, fOldHeading = 0, fOldRPM = 0;
  float fTemperature = 0, fHumidity = 0,  fPressure = 0, fHeading = 0, fRPM = 0;
  char cTexto[255];
  
  sensors_event_t event;
  if(bFlag) {
    // delay(delayMS);
    dht.temperature().getEvent(&event);
    fOldTemperature = event.temperature;
    dht.humidity().getEvent(&event);
    fOldHumidity = event.relative_humidity;
    bmp.getEvent(&event);
    fOldPressure = event.pressure;
    float fBMPTemperature;
    bmp.getTemperature(&fBMPTemperature);
    sprintf(cTexto, "%f %f %f %f", fOldTemperature, fOldHumidity, fOldPressure, fBMPTemperature);
    Serial.println(cTexto);
    bFlag = !bFlag;
  }
  // delay(delayMS);
  
}
