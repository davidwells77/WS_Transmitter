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
  delay(1000);
  Serial.begin(9600);
  pinMode(HALLPIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
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
  digitalWrite(LED_BUILTIN, HIGH);
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

void sendMessage(int temperature, int humidity, int pressure, int heading) {
  
  char *message = (char *)malloc(RH_ASK_MAX_MESSAGE_LEN);
  sprintf(message, "T=%d;H=%d;P=%d;D=%d", temperature, humidity, pressure, heading);
  // Serial.println(message);
  ask.send((uint8_t *)message, strlen(message));
  ask.waitPacketSent();
}

void loop() {
  // put your main code here, to run repeatedly:
  static bool first = true, led = false;
  unsigned long currentMillis = millis();
  static unsigned long delayMillis = currentMillis, blinker = currentMillis;
  static int temperature = 0, humidity = 0,  pressure = 0, heading = 0, RPM = 0;
  
  if(currentMillis - blinker >= 1000) {
    if(led) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    led = !led;
    blinker = currentMillis;
  }
  if((first) || (currentMillis - delayMillis >= 60000)) {
    readSensors(&temperature, &humidity, &pressure, &heading);
    sendMessage(temperature, humidity, pressure, heading);
    if(first) first = !first;
    delayMillis = currentMillis;
  }
}
