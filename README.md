# Weather_ESP32_BME280
ESP-32 &amp; BME280 weather station with data upload to Thingspeak

The weather station is powered by a 18500 LiPo battery that is charged by a 6V/1W solar panel connected to a TP4056 charging/protection module. I uploads temperature, humidity and air pressure readings from a Bosch Sonsortec BME280 sensor to Thingspeak, and then sleeps for 15 minutes before the next measurement.

Two HT7333 LDO voltage converters (in TO-92 package) supply the ESP-32 with constant 3.3V. A single HT7333 can only provide 250mA, which is not enough during Wifi operation.

In addition, the sketch queries GPIO 4 and can select one of two Wifi / Thingspeak channel configurations for upload. The code was written in Visual Studio Code with the Platformio extension for the Espressif platform.

![schematic](https://github.com/ospringauf/Weather_ESP32_BME280/blob/master/schematic.png)

