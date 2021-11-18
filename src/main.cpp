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
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>

#include "credentials.h"
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros



WiFiClient client; // wifi client object
Adafruit_AHTX0 aht;

// pins
#define MY_LED LED_BUILTIN
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
  // ThingSpeak.setField(3, pressure);
  // ThingSpeak.setField(4, soil);
  // ThingSpeak.setField(5, bmps);

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

int readAHT10Sensor()
{
  if (aht.begin()) {
    Serial.println("Found AHT20");
  } else {
    Serial.println("Didn't find AHT20");
  }  

  sensors_event_t sehum, setemp;
  
  aht.getEvent(&sehum, &setemp);// populate temp and humidity objects with fresh data

  temperature = setemp.temperature;
  humidity = sehum.relative_humidity;
  pressure = 0;
  bmps = 0;

  Serial.print("reading complete");

  return 0;
}


void setup_weather()
{
  // voltage is critical for a short period during startup
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  pinMode(MY_LED, OUTPUT);
  digitalWrite(MY_LED, LED_OFF);
  pinMode(4, INPUT);

  Serial.begin(9600);
  Serial.println("Starting");
  delay(200);

  bool outdoor = 1; //digitalRead(4) == 0;
  int err = connectWifi(outdoor);

  delay(500);

  if (err == 0)
    err = readAHT10Sensor();

  // soil = readSoil();
  // if (soil > 100)
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
  Serial.println("Going to sleep now...");
  delay(200);
  ESP.deepSleep(UpdateInterval, WAKE_RF_DEFAULT); // Sleep for the time set by 'UpdateInterval'
  // ESP.deepSleep(5 * 1000000);
  yield();
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
