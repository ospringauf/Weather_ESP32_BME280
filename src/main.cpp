/* 
 * ESP8266 and AHT10 sensor that updates a Thingspeak channel, then goes to Deep Sleep
 * Also please refer to Adafruit for their Licences
 * 
 * wiring AHT10: SCL=D1 SDA=D2
 * conect D0 to RST (wakeup)
*/

// from https://github.com/G6EJD/ESP32-8266-Thingspeak-Deep-Sleep-Examples/blob/master/ESP32_Thingspeak_Deep_Sleep_BME280.ino

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

#include "credentials.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros

WiFiClient client; // wifi client object
Adafruit_AHTX0 aht;

// pins
#define MY_LED LED_BUILTIN

#define LED_OFF HIGH
#define LED_ON LOW

#define ERR_WIFI 3
#define ERR_SENSOR 6

// const int UpdateInterval = 0.33 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; //20-Sec update interval for development tests, to fast for practical purposes and Thingspeak!
const int UpdateInterval = 15 * 60 * 1000000; // e.g. 15 * 60 * 1000000; for a 15-Min update interval (15-mins x 60-secs * 1000000uS)

float temperature, humidity;

void UpdateThingSpeak(float temperature, float humidity)
{
  ThingSpeak.begin(client); // Initialize ThingSpeak

  // set the fields with the values
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);

  ThingSpeak.setStatus("ok");

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(channel, api_key);
  if (x == 200)
  {
    Serial.println("Channel update successful.");
  }
  else
  {
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

int connectWifi()
{
  Serial.println("Start WiFi");
  delay(200);

  WiFi.begin(ssid, password);

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

int readAHT10Sensor()
{
  if (aht.begin())
  {
    Serial.println("Found AHT20");
  }
  else
  {
    Serial.println("Didn't find AHT20");
  }

  sensors_event_t sehum, setemp;

  aht.getEvent(&sehum, &setemp); // populate temp and humidity objects with fresh data

  temperature = setemp.temperature;
  humidity = sehum.relative_humidity;

  Serial.print("reading complete");

  return 0;
}

void setup_weather()
{
  pinMode(MY_LED, OUTPUT);
  digitalWrite(MY_LED, LED_OFF);

  Serial.begin(9600);
  Serial.println("Starting");
  delay(200);

  int err = connectWifi();

  delay(500);

  if (err == 0)
    err = readAHT10Sensor();

  if (err == 0)
  {
    blink(2, 100);

    String data = "t=" + String(temperature) +
                  " hum=" + String(humidity);
    Serial.println(data);

    UpdateThingSpeak(temperature, humidity);
  }
  else
    blink(err, 100);

  // back to sleep
  Serial.println("Going to sleep now...");
  delay(200);
  ESP.deepSleep(UpdateInterval, WAKE_RF_DEFAULT); // Sleep for the time set by 'UpdateInterval'
  yield();
}

void setup()
{
  setup_weather();
}

void loop()
{
  //Do nothing as it will never get here!

  // Serial.println(readSoil());
  // delay(1000);
}
