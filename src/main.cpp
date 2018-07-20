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

WiFiClient client; // wifi client object


#define MY_BME280_ADR 0x76
#define MY_LED 23
#define LED_OFF HIGH
#define LED_ON LOW

#define ERR_WIFI 3
#define ERR_SENSOR 6


char ThingSpeakAddress[] = "api.thingspeak.com"; // Thingspeak address
// const int UpdateInterval = 0.33 * 60 * 1000000;  // e.g. 0.33 * 60 * 1000000; //20-Sec update interval for development tests, to fast for practical purposes and Thingspeak!
const int UpdateInterval = 15 * 60 * 1000000;  // e.g. 15 * 60 * 1000000; for a 15-Min update interval (15-mins x 60-secs * 1000000uS)

#define pressure_offset 3.9 // Compensates for this location being 40M asl  
Adafruit_BME280 bme;
String api_key;

float temperature, humidity, pressure;


void UpdateThingSpeak(String DataForUpload) {
  WiFiClient client;
  if (!client.connect(ThingSpeakAddress, 80)) {
     Serial.println("connection to Thingspeak failed");
     return;
  }
  else
  {
    Serial.println(DataForUpload);
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + api_key + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(DataForUpload.length());
    client.print("\n\n");
    client.print(DataForUpload);
    delay(200);
  }
  client.stop();
}

void blink(int n, uint32_t d = 250) {
  for (int i=0; i<n; i++) {
    digitalWrite(MY_LED, LED_ON);  
    delay(d);
    digitalWrite(MY_LED, LED_OFF);
    delay(d);
  }
}

int connectWifi(bool outdoor) {
  blink(outdoor? 2 : 1);
  Serial.println(outdoor);
  Serial.println("Start WiFi");
  delay(200);
  
  if (outdoor) {
    WiFi.begin(ssid2, password2);
    api_key = api_key2;
   } else  {
    WiFi.begin(ssid1, password1);
    api_key = api_key1;
   }

  int retry = 20;
  while (WiFi.status() != WL_CONNECTED && retry > 0 ) {
      Serial.print("."); 
      blink(1, 20);
      retry--;
      delay(500); 
  }

  if (retry == 0) {
      Serial.println("WiFi connection failed");
  }

  return retry > 0 ? 0 : ERR_WIFI;

}

int readSensor() {
  // Wire.begin(19, 18); // (sda,scl) 
  Wire.begin(18, 21); // (sda,scl) 
  
  if (!bme.begin(MY_BME280_ADR)) {
    Serial.println("Could not find a sensor, check wiring!");
    return ERR_SENSOR;
  } 
  else
  {
    Serial.println("Found a sensor, continuing");
    while (isnan(bme.readPressure())) {
      Serial.println(bme.readPressure());
    }
  }

  temperature = bme.readTemperature(); 
  humidity    = bme.readHumidity();
  pressure    = bme.readPressure() / 100.0F + pressure_offset;
  return 0;
}


void setup() {
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

  if (err == 0)
  {
    blink(2, 100);
    UpdateThingSpeak("field1="+String(temperature)+"&field2="+String(humidity)+"&field3="+String(pressure)); //Send the data as text
  } 
  else  
    blink(err, 100);

  // back to sleep
  esp_deep_sleep_enable_timer_wakeup(UpdateInterval);
  Serial.println("Going to sleep now...");
  delay(200);
  esp_deep_sleep_start();
}


void loop() {
  //Do nothing as it will never get here!
}

