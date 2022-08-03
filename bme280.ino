/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS PB0
#define BME_CS_1 PB1
#define BME_CS_2 PB2

#define SEALEVELPRESSURE_HPA (1013.25)

// Adafruit_BME280 bme; // I2C
Adafruit_BME280 bme(BME_CS);    // hardware SPI
Adafruit_BME280 bme1(BME_CS_1); // hardware SPI
Adafruit_BME280 bme2(BME_CS_2); // hardware SPI
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status, status1, status2;

    // default settings
    status = bme.begin();
    status1 = bme1.begin();
    status2 = bme2.begin();
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status)
    {
        Serial.println("Could not find a valid BME280 0 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            delay(10);
    }
    if (!status1)
    {
        Serial.println("Could not find a valid BME280 1 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme1.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            delay(10);
    }
    if (!status2)
    {
        Serial.println("Could not find a valid BME280 2 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme1.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            delay(10);
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;

    Serial.println();
}

void loop()
{
    printValues();
    delay(delayTime);
}

void printValues()
{
    Serial.print("Temperature 0 = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    Serial.print("Temperature 1 = ");
    Serial.print(bme1.readTemperature());
    Serial.println(" °C");
    Serial.print("Temperature 2 = ");
    Serial.print(bme1.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure 0 = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print("Pressure 1 = ");
    Serial.print(bme1.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print("Pressure 2 = ");
    Serial.print(bme2.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity 0 = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    Serial.print("Humidity 1 = ");
    Serial.print(bme1.readHumidity());
    Serial.println(" %");
    Serial.print("Humidity 2 = ");
    Serial.print(bme2.readHumidity());
    Serial.println(" %");

    Serial.println();
}