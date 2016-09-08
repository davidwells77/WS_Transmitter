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
  delay(1000);
}

void printValues(int temperature, int humidity, int pressure, int heading) {

  char buf[255];
  sprintf(buf, "Temperature=%d;Humidity=%d;Pressure=%d;Heading=%d", temperature, humidity, pressure, heading);
  Serial.println(buf);
}

void readSensors(int* temperature, int* humidity, int* pressure, int* heading) {
  
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  *temperature = (int) (event.temperature * 10);
  dht.humidity().getEvent(&event);
  *humidity = (int) (event.relative_humidity * 10);
  bmp.getEvent(&event);
  *pressure = (int) event.pressure;
  mag.getEvent(&event);
  *heading = (int) (atan2(event.magnetic.y, event.magnetic.x) * 1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  static bool first = true;
  static int oldTemperature = 0, oldHumidity = 0, oldPressure = 0, oldHeading = 0, oldRPM = 0;
  unsigned long currentMillis = millis();
  static unsigned long delayMillis = currentMillis;
  static int temperature = 0, humidity = 0,  pressure = 0, heading = 0, RPM = 0;
  
  if(first) {
    readSensors(&temperature, &humidity, &pressure, &heading);
    delayMillis = currentMillis;
    first = !first;
  }
  if(currentMillis - delayMillis >= 60000) {
    readSensors(&temperature, &humidity, &pressure, &heading);
    delayMillis = currentMillis;
  }
  if((temperature != oldTemperature) || (humidity != oldHumidity) || (pressure != oldPressure) || (heading != oldHeading)) {
    printValues(temperature, humidity, pressure, heading);
    if(oldTemperature != temperature)
      oldTemperature = temperature;
    if(oldHumidity != humidity)
      oldHumidity = humidity;
    if(oldPressure != pressure)
      oldPressure = pressure;
    if(oldHeading != heading)
      oldHeading = heading;
  }
}
