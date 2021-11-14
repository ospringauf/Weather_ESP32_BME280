/* 
 * ESP32 and a BME280 sensor that updates a Thingspeak channel, then goes to Deep Sleep
 * Also please refer to Adafruit for their Licences
*/

// from https://github.com/G6EJD/ESP32-8266-Thingspeak-Deep-Sleep-Examples/blob/master/ESP32_Thingspeak_Deep_Sleep_BME280.ino

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "esp_deep_sleep.h" //Library needed for ESP32 Sleep Functions

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "credentials.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros



WiFiClient client; // wifi client object


// pins
#define MY_LED 23
#define SOIL_SENSOR_PWR 22
#define SOIL_SENSOR_INPUT 35
#define MY_SDA 18
#define MY_SCL 21

#define MY_BME280_ADR 0x76
#define LED_OFF HIGH
#define LED_ON LOW
#define ALTITUDE_TH 550.0

#define ERR_WIFI 3
#define ERR_SENSOR 6

// const int UpdateInterval = 0.33 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; //20-Sec update interval for development tests, to fast for practical purposes and Thingspeak!
const int UpdateInterval = 15 * 60 * 1000000; // e.g. 15 * 60 * 1000000; for a 15-Min update interval (15-mins x 60-secs * 1000000uS)

#define pressure_offset 0.0 // no compensation
Adafruit_BME280 bme;
const char* api_key;
int channel;

float temperature, humidity, pressure, bmps;
int soil;


void UpdateThingSpeak(float temperature, float humidity, float pressure, int soil, float bmps) {
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  // set the fields with the values
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, pressure);
  ThingSpeak.setField(4, soil);
  ThingSpeak.setField(5, bmps);

  ThingSpeak.setStatus("ok");

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(channel, api_key);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}

void blink(int n, uint32_t d = 250)
{
  for (int i = 0; i < n; i++)
  {
    digitalWrite(MY_LED, LED_ON);
    delay(d);
    digitalWrite(MY_LED, LED_OFF);
    delay(d);
  }
}

int connectWifi(bool outdoor)
{
  blink(outdoor ? 2 : 1);
  Serial.println(outdoor);
  Serial.println("Start WiFi");
  delay(200);

  if (outdoor)
  {
    WiFi.begin(ssid2, password2);
    api_key = api_key2;
    channel = channel2;
  }
  else
  {
    WiFi.begin(ssid1, password1);
    api_key = api_key1;
    channel = channel1;
  }

  int retry = 20;
  while (WiFi.status() != WL_CONNECTED && retry > 0)
  {
    Serial.print(".");
    blink(1, 20);
    retry--;
    delay(500);
  }

  if (retry == 0)
  {
    Serial.println("WiFi connection failed");
  }

  return retry > 0 ? 0 : ERR_WIFI;
}

// temp in Celsius, altitude in meters
float seaLevelPressure(float altitude, float temp, float pres)
{
    float seaPress = NAN;
    if(!isnan(altitude) && !isnan(temp) && !isnan(pres))
    {
        seaPress = (pres / pow(1 - ((0.0065 *altitude) / (temp + (0.0065 *altitude) + 273.15)), 5.257));
    }
    return seaPress;
}

int readSensor()
{
  Wire.begin(MY_SDA, MY_SCL); // (sda,scl)
  delay(100);

  if (!bme.begin(MY_BME280_ADR))
  {
    Serial.println("Could not find a sensor, check wiring!");
    return ERR_SENSOR;
  }
  else
  {
    Serial.println("Found a sensor, continuing");
    while (isnan(bme.readPressure()))
    {
      Serial.println(bme.readPressure());
    }
  }

  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F + pressure_offset;
  bmps = seaLevelPressure(ALTITUDE_TH, temperature, pressure);
  return 0;
}

int readSoil() {
  pinMode(SOIL_SENSOR_PWR, OUTPUT);
  digitalWrite(SOIL_SENSOR_PWR, HIGH);
  delay(200);
  double AirValue = 860.0;   // dry sensor
  double WaterValue = 400.0; // in water
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);

  uint16_t soilMoistureValue = analogRead(SOIL_SENSOR_INPUT); //put Sensor insert into soil
  // Serial.print(soilMoistureValue);
  // Serial.print("  ");
  
  digitalWrite(SOIL_SENSOR_PWR, LOW); // turn sensor off
  return (int)(100 * (1 - (soilMoistureValue - WaterValue)/(AirValue - WaterValue)));
}

void setup_weather()
{
  // voltage is critical for a short period during startup
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  pinMode(MY_LED, OUTPUT);
  digitalWrite(MY_LED, LED_OFF);
  pinMode(4, INPUT);

  Serial.begin(115200);
  Serial.println("Starting");
  delay(200);

  bool outdoor = digitalRead(4) == 0;
  int err = connectWifi(outdoor);

  delay(500);

  if (err == 0)
    err = readSensor();

  soil = readSoil();
  if (soil > 100)
    soil = 0;

  if (err == 0)
  {
    blink(2, 100);

    String data = "t=" + String(temperature) + 
      " hum=" + String(humidity) + 
      " pres=" + String(pressure) + 
      " soil=" + String(soil) + 
      " bmps=" + String(bmps);
    Serial.println(data);

    UpdateThingSpeak(temperature, humidity, pressure, soil, bmps);
  }
  else
    blink(err, 100);

  // back to sleep
  esp_deep_sleep_enable_timer_wakeup(UpdateInterval);
  Serial.println("Going to sleep now...");
  delay(200);
  esp_deep_sleep_start();
}

void setup()
{
  setup_weather();
  // Serial.begin(115200);
  // Serial.println("Starting moisture");
  // delay(200);
}



void loop()
{
  //Do nothing as it will never get here!

  // Serial.println(readSoil());
  // delay(1000);
}
