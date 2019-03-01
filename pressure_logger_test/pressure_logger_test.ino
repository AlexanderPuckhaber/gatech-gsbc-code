/*
 * This code is currently a combination of three example programs, links are in the comments below
 * The sensors are: MPRLS Pressure sensor, BMP280 Pressure and Temperature sensor, and SD card breakout
 */


/*!
 * @file mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
 
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI (DI) - pin 11
 ** MISO (DO) - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (Use 10 for Adafruit)
 ** 

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;


/*
 * Timekeeping without a real-time-clock
 * units in microseconds
 */
unsigned long currentTime = 0;
unsigned long lastTime = 0;

const String dataFileName = "data8.txt";

/*
 * TODO: fix setup so it will continue even if not all the sensors are initialized
 * We can even have it write simple max altitude data to the onboard EEPROM if the sd card isn't found
 */
void setup() {
  Serial.begin(115200);
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");

  Serial.println(F("BMP280 test"));
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found BMP280 sensor");

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) {
      delay(10);
    }
  }
  Serial.println("card initialized.");

  /*
   * In hindsight, this was a bad idea. I'll just use a real-time clock next time
  // get last time from beginning of last line
  File dataFile = SD.open(dataFileName, FILE_WRITE);
  if (dataFile) {
    Serial.println("reading " + dataFileName + " for old timestamp: ");
    // last line shouldn't be more than 200 characters...
    dataFile.seek(dataFile.size() - 10);
    int lastLineIndex = dataFile.position();
    int lastLastLineIndex = lastLineIndex;
    while ( dataFile.available() ) {
      if (dataFile.read() == '\n') {
        lastLastLineIndex = lastLineIndex;
        lastLineIndex = dataFile.position();
      }
      Serial.println(lastLineIndex);
    }
    delay(100);
    // now we should be at the beginning of the last line...
    Serial.println("lastLastLineIndex: " + lastLastLineIndex);
    String lastTimeData = "";
    dataFile.seek(lastLastLineIndex);
    bool stopSearching = false;
    delay(100);
    Serial.println();
    while(dataFile.available() && !stopSearching) {
      String next = (String)dataFile.read();
      //Serial.print(dataFile.position());
      if (next != ',') {
        lastTimeData += next;
        Serial.print(next);
      } else {
        stopSearching = true;
      }
    }
    dataFile.close();
    delay(100);
    Serial.println("Last Time Data: " + lastTimeData);
    char tarray[sizeof(lastTimeData)];
    lastTimeData.toCharArray(tarray, sizeof(tarray));
    currentTime = atoi(tarray);  
  } else {
    Serial.println("No file matching name " + dataFileName + ", will be created later");
  }
  */
}


void loop() {
  // update time
  currentTime += (micros() - lastTime);
  lastTime = currentTime;

  
  String dataString = "";

  dataString += (String)currentTime;
  dataString += ", ";
  
  float pressure_hPa = mpr.readPressure();
  dataString += (String)pressure_hPa;
  dataString += ", ";
  dataString += (String)(bme.readPressure() / 100);
  dataString += ", ";
  dataString += (String)bme.readTemperature();
  dataString += ",\n";

  /*
  
  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  //Serial.print("Pressure (PSI): "); Serial.println(pressure_hPa / 68.947572932);

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure());
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forecast
  Serial.println(" m");

  */
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(dataFileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening " + dataFileName);
  }
  
  delay(100);
}
