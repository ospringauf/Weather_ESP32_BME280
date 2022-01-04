/* 
 * ESP32 and a BME280 sensor that updates a Thingspeak channel, then goes to Deep Sleep
 * Also please refer to Adafruit for their Licences
*/

// from https://github.com/G6EJD/ESP32-8266-Thingspeak-Deep-Sleep-Examples/blob/master/ESP32_Thingspeak_Deep_Sleep_BME280.ino

// TODO bulk upload to Thingspeak?

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "esp_deep_sleep.h" //Library needed for ESP32 Sleep Functions

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "credentials.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include "ESPDateTime.h"
#include "DateTimeTZ.h"
#include "time.h"
#include <nvs_flash.h>

// pins
#define MY_LED 23
#define SOIL_SENSOR_PWR 22
#define SOIL_SENSOR_INPUT 35
#define MY_SDA 18
#define MY_SCL 21

#define MY_BME280_ADR 0x76
#define LED_OFF HIGH
#define LED_ON LOW
#define MY_ALTITUDE 580.0

#define ERR_WIFI 3
#define ERR_SENSOR 6
#define RESYNC_NTP_AFTER 5
#define STORAGE 200 // TODO max 200

#define pressure_offset 0.0 // no compensation

WiFiClient client; // wifi client object
Adafruit_BME280 bme;

//const int UpdateInterval = 15 * 60 * 1000000; // e.g. 15 * 60 * 1000000; for a 15-Min update interval (15-mins x 60-secs * 1000000uS)
const ulong UpdateInterval = 30 * 60 * 1000000;
const char *api_key;
const char *my_time_zone = TZ_Europe_Berlin;
int channel;

int soil;

typedef struct
{
  float temp = 0;
  float humi = 0;
  float pres = 0;
  time_t timestamp = 0;
} sensorReadings;

sensorReadings current;

// RTC memory (max 8k), cyclic buffer (fifo)
RTC_DATA_ATTR int storeCount = 0; // number of stored readings
RTC_DATA_ATTR int readPtr = 0; // next reading to upload
RTC_DATA_ATTR sensorReadings storedReadings[STORAGE];
RTC_DATA_ATTR int resyncTime = 0; // NTP update, remaining cycles


// temp in Celsius, altitude in meters
float seaLevelPressure(float temp, float pres)
{
  float seaPress = NAN;
  if (!isnan(MY_ALTITUDE) && !isnan(temp) && !isnan(pres))
  {
    seaPress = (pres / pow(1 - ((0.0065 * MY_ALTITUDE) / (temp + (0.0065 * MY_ALTITUDE) + 273.15)), 5.257));
  }
  return seaPress;
}

void storeValues()
{
  Serial.println("--- STORING VALUES");
  if (storeCount >= STORAGE) {
    Serial.println("error: no more space in storage");
    return;
  }
  int s = (readPtr + storeCount) % STORAGE;
  storedReadings[s].temp = current.temp;
  storedReadings[s].humi = current.humi;
  storedReadings[s].pres = current.pres;
  storedReadings[s].timestamp = time(nullptr);
  storeCount = storeCount + 1;
}

void printValues(int s)
{
  // Serial.println("--- READING VALUES");
  Serial.print("#");
  Serial.print(s);
  Serial.print("  T=");
  Serial.print(storedReadings[s].temp);
  Serial.print("  H=");
  Serial.print(storedReadings[s].humi);
  Serial.print("  P=");
  Serial.print(storedReadings[s].pres);
  Serial.print(" / ");
  Serial.print(seaLevelPressure(storedReadings[s].temp, storedReadings[s].pres));
  Serial.print("  C=");
  Serial.print(storedReadings[s].timestamp);
  Serial.print(" / ");
  Serial.println(DateFormatter::format(DateFormatter::ISO8601, storedReadings[s].timestamp, my_time_zone));
}

void uploadThingSpeak()
{
  ThingSpeak.begin(client); // Initialize ThingSpeak

  // upload all stored readings
  while (storeCount > 0)
  {
    int s = readPtr;
    printValues(s);

    // set the fields with the values
    ThingSpeak.setField(1, storedReadings[s].temp);
    ThingSpeak.setField(2, storedReadings[s].humi);
    ThingSpeak.setField(3, storedReadings[s].pres);
    // ThingSpeak.setField(4, soil);
    ThingSpeak.setField(5, seaLevelPressure(storedReadings[s].temp, storedReadings[s].pres));

    time_t ts = storedReadings[s].timestamp;
    ThingSpeak.setCreatedAt(DateFormatter::format(DateFormatter::ISO8601, ts, my_time_zone));

    // ThingSpeak.setStatus("ok");

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(channel, api_key);
    if (x == 200)
    {
      Serial.print("successful upload of reading ");
      Serial.println(s);
      storeCount = storeCount - 1;
      readPtr = (readPtr + 1) % STORAGE;
      if (storeCount > 0)
      {
        // Thingspeak API has 15 seconds rate limit
        delay(20 * 1000);
      }
    }
    else
    {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
      break;
    }
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

void setupDateTime()
{
  Serial.println("--- TIME SETUP");
  // setup this after wifi connected
  // you can use custom timeZone,server and timeout
  DateTime.setServer("de.pool.ntp.org");
  // DateTime.setTimeZone(my_time_zone); // not useful, tz is lost after wakeup
  time_t t0 = time(nullptr);
  DateTime.begin(15 * 1000);
  if (DateTime.isTimeValid())
  {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    Serial.printf("Timestamp is %ld\n", DateTime.now());
    resyncTime = RESYNC_NTP_AFTER;

    if (t0 < 1000000000L)
    {
      // fix stored times
      time_t t1 = time(nullptr);
      for (int i = 0; i < storeCount; ++i)
      {
        int s = (readPtr + i) % STORAGE;
        storedReadings[s].timestamp += (t1 - t0);
      }
    }
  }
  else
  {
    Serial.println("Failed to get time from server.");
  }
}

int connectWifi(bool outdoor)
{
  blink(outdoor ? 2 : 1);
  Serial.println("--- START WIFI");
  Serial.print("switch: ");
  Serial.println(outdoor);
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
  Serial.println();

  if (retry == 0)
  {
    Serial.println("WiFi connection failed");
  }
  else
  {
    if (resyncTime <= 0)
      setupDateTime();
    else
      resyncTime = resyncTime - 1;
  }

  return retry > 0 ? 0 : ERR_WIFI;
}

int readSensor()
{
  Serial.println("--- READ SENSORS");
  Wire.begin(MY_SDA, MY_SCL); // (sda,scl)
  delay(500);

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

  current.temp = bme.readTemperature();
  current.humi = bme.readHumidity();
  current.pres = bme.readPressure() / 100.0F + pressure_offset;
  return 0;
}

int readSoil()
{
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
  return (int)(100 * (1 - (soilMoistureValue - WaterValue) / (AirValue - WaterValue)));
}

void setupWeather()
{
  // voltage is critical for a short period during startup
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  pinMode(MY_LED, OUTPUT);
  digitalWrite(MY_LED, LED_OFF);
  pinMode(4, INPUT);

  Serial.begin(115200);
  Serial.println("=== STARTING");
  Serial.print("stored: ");
  Serial.println(storeCount);
  Serial.printf("now  is %ld\n", DateTime.now()); // RTC / NTP time
  Serial.printf("time is %ld\n", time(nullptr)); // seconds since reset, counting across sleep intervals

  delay(200);

  bool outdoor = digitalRead(4) == 0;
  int err = 0;

  if (resyncTime <= 0)
    connectWifi(outdoor);

  delay(500);

  if (err == 0)
    err = readSensor();

  // soil = readSoil();
  // if (soil > 100)
  //   soil = 0;

  if (err == 0)
  {
    storeValues();
    printValues((readPtr+storeCount-1)%STORAGE);

    err = connectWifi(outdoor);
    if (err == 0)
    {
      blink(2, 100);
      uploadThingSpeak();
    }
    else
    {
      blink(err, 100);
    }
  }
  else
    blink(err, 100);

  // back to sleep
  ulong ui = UpdateInterval;
  if (storeCount > 25)
    ui = 2L * UpdateInterval;
  if (storeCount > 100)
    ui = 5L * UpdateInterval;

  esp_deep_sleep_enable_timer_wakeup(ui);
  Serial.println("--- GOING TO SLEEP NOW");
  delay(200);
  esp_deep_sleep_start();
}

void setup()
{
  // nvs_flash_erase(); // erase the NVS partition and...
  // nvs_flash_init(); // initialize the NVS partition.

  setupWeather();
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
