#include <EEPROM.h>

/*
 * This code is currently a combination of three example programs, links are in the comments below
 * The sensors are: MPRLS Pressure sensor, BMP280 Pressure and Temperature sensor, and SD card breakout
 */

/*
 * WIRING for Arduino Nano:
 * A4: SDA
 * A5: SCL
 * 
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
/*
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
*/


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

#define BMP_SCK 5
#define BMP_MISO 4
#define BMP_MOSI 3 
#define BMP_CS 2

//Adafruit_BMP280 bme; // I2C (I fried the I2C oops)
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK); // software SPI 

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
unsigned long startTime;
unsigned long currentTime;
unsigned long lastTime;
int lastTimeEEPROMAddress = 16;

/*
 * Floats to hold sensor data
 */
float pressureMPRLS = 0;
float pressureBMP = 0;
float temperatureBMP = 0;

/*
 * Keep lowest pressure, temperature from sensors
 */
float lowestPressureMPRLS = 100000;
int lowestPressureMPRLSEEPROMAddress = 48;
float lowestPressureBMP = 100000;
int lowestPressureBMPEEPROMAddress = 48+32;
float lowestTemperatureBMP = 100;
int lowestTemperatureBMPEEPROMAddress = 48+32+32;

unsigned int timeBetweenEEPROMRecordUpdatesMs = 120*1000; //Every 2 minutes to preserve EEPROM life
unsigned long lastEEPROMRecordUpdateMs = currentTime;

float f; // temp float to use EEPROM.get

// LAUNCH: change this to a new file, make sure filename is very short
const String dataFileName = "yeet.txt";

/*
 * TODO: fix setup so it will continue even if not all the sensors are initialized
 * We can even have it write simple max altitude data to the onboard EEPROM if the sd card isn't found
 */

bool anyFailures = false;
//unsigned int timeBetweenSetupAttemptsMs = 480*1000; //Every 4 minutes
//unsigned long lastSetupAttemptMs = currentTime;

/*
 * Buzzer and Button stuff
 * Parameters for LAUNCH:
 * disableBuzzer = false;
 * beginLocationBuzzerMs = 3600000;
 * timeBetweenLocationBuzzerMs = 20*1000;
 */
bool disableBuzzer = false;
const int buttonPin = 8;
bool buttonState = 0;
const int buzzerPin = 6;
bool enableLocationBuzzer = 0;
bool disableLocationBuzzer = 0;
unsigned long beginLocationBuzzerMs = 3600000;  // you need to type 3600000 because arduino IDE is ;(
unsigned int timeBetweenLocationBuzzerMs = 10*1000;
unsigned long lastLocationBuzzerMs = currentTime;
unsigned int timeBetweenStandbyBuzzerMs = 10*1000;
unsigned long lastStandbyBuzzerMs = currentTime;
unsigned int timeBetweenErrorBuzzerMs = 6*1000;
unsigned long lastErrorBuzzerMs = currentTime;

/*
 * Timing stuff
 */
 
/*
 * EEPROM Stuff
 */

void resetEEPROM() {
  unsigned long templong;
  EEPROM.get(lastTimeEEPROMAddress, templong);
  Serial.println(templong);
  EEPROM.put(lastTimeEEPROMAddress, (unsigned long)0);
  EEPROM.get(lowestPressureMPRLSEEPROMAddress, f);
  Serial.println(f);
  EEPROM.put(lowestPressureMPRLSEEPROMAddress, (float)0);
  EEPROM.get(lowestPressureBMPEEPROMAddress, f);
  Serial.println(f);
  EEPROM.put(lowestPressureBMPEEPROMAddress, (float)0);
  EEPROM.get(lowestTemperatureBMPEEPROMAddress, f);
  Serial.println(f);
  EEPROM.put(lowestTemperatureBMPEEPROMAddress, (float)0);

  // happy chirp
  buzzerChirp(buzzerPin, 200, 200, 4000);
  delay(5*1000);
}

void updateEEPROMRecords() {
  if (currentTime > lastEEPROMRecordUpdateMs + timeBetweenEEPROMRecordUpdatesMs) {
    Serial.println("updating records in EEPROM");
    EEPROM.put(lastTimeEEPROMAddress, currentTime);
    Serial.print("saving new time: ");
    Serial.print(currentTime);
    Serial.print(" should match ");
    unsigned long templong;
    EEPROM.get(lastTimeEEPROMAddress, templong);
    Serial.println(templong);

    EEPROM.put(lowestPressureMPRLSEEPROMAddress, lowestPressureMPRLS);
    EEPROM.put(lowestPressureBMPEEPROMAddress, lowestPressureBMP);
    Serial.print("new lowest pressure: ");
    Serial.print(lowestPressureBMP);
    Serial.print(" should match: ");
    EEPROM.get(lowestPressureBMPEEPROMAddress, f);
    Serial.println(f);
    
    EEPROM.put(lowestTemperatureBMPEEPROMAddress, lowestTemperatureBMP);

    lastEEPROMRecordUpdateMs = currentTime;
  }
}

void updateLowest() {
  if (pressureMPRLS < lowestPressureMPRLS) {
    lowestPressureMPRLS = pressureMPRLS;
  }
  if (pressureBMP < lowestPressureBMP) {
    lowestPressureBMP = pressureBMP;
  }
  if (temperatureBMP < lowestTemperatureBMP) {
    lowestTemperatureBMP = temperatureBMP;
  }
}

void buzzerChirp(int buzzerPin, int duration, int startFreq, int endFreq) {
  Serial.print("chirping for: ");
  Serial.print(duration);
  Serial.println(" ms");
  
  int start = startFreq;
  int end = endFreq;

  int num = end - start + 1;
  int mult = 1;
  if (num < 0) {
    mult = -1;
    num *= mult;
  }

  double delayIncrement = duration/(double)(num);
  //Serial.print("delay increment: ");
  //Serial.println(delayIncrement);

  unsigned long startTime = micros()/1000;
  unsigned long lastTime = micros()/1000;

  int freq = startFreq;

  for (int i = 0; i < num; i++) {
    if (!disableBuzzer) {
      tone(buzzerPin, freq);
    }
    lastTime = micros()/1000;
    while (micros()/1000 < lastTime + delayIncrement) {
      delay(10);
      
      //Serial.println(lastTime);
    }

    double actualDelay = micros()/1000 - lastTime;
    i+= actualDelay/delayIncrement;

    if (mult > 0) {
      freq = (int)(startFreq + (int)(i));
    } else {
      freq = (int)(startFreq - (int)(i));
    }
    //Serial.println(freq);
  }
  noTone(buzzerPin); //stop sound
  
}

void doStandbyBuzzer() {
  if (currentTime > lastStandbyBuzzerMs + timeBetweenStandbyBuzzerMs) {
    buzzerChirp(buzzerPin, 50, 1500, 2000);
    Serial.println("STANDBY (buzzer)");
    lastStandbyBuzzerMs = currentTime;
  }
}

void doErrorBuzzer() {
  if (currentTime > lastErrorBuzzerMs + timeBetweenErrorBuzzerMs) {
    buzzerChirp(buzzerPin, 200, 50, 50);
    Serial.println("ERROR (buzzer)");
    lastErrorBuzzerMs = currentTime;
  }
}

void doLocationBuzzer() {
  if (!disableLocationBuzzer) {
    if (currentTime > lastLocationBuzzerMs + timeBetweenLocationBuzzerMs) {
      buzzerChirp(buzzerPin, 500, 200, 2000);
      tone(buzzerPin, 2000);
      delay(1000);
      //buzzerChirp(buzzerPin, 1000, 2000, 2000);
      buzzerChirp(buzzerPin, 500, 2000, 200);
      Serial.println("LOCATION (buzzer)");
      lastLocationBuzzerMs = currentTime;
    }
  } else {
    Serial.println("LOCATION BUZZER DISABLED");
  }
}

void doBuzzer() {
  Serial.print(currentTime);
    Serial.print(" ");
    Serial.print(beginLocationBuzzerMs);
    Serial.print(" ");
    Serial.println(currentTime > beginLocationBuzzerMs);
  if (currentTime > beginLocationBuzzerMs) {
    
    enableLocationBuzzer = true;
  }
  
  if (enableLocationBuzzer) {
    doLocationBuzzer();
  } else if (anyFailures) {
    doErrorBuzzer();
  } else {
    doStandbyBuzzer();
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(buttonPin, INPUT);
  // check for reset
  buttonState = digitalRead(buttonPin);
  if (buttonState) {
    // reset
    Serial.println("last EEPROM data:");
    resetEEPROM();
  }

  anyFailures = false;

  EEPROM.get(lastTimeEEPROMAddress, startTime);

  /*
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    anyFailures = true;
  }
  Serial.println("Found MPRLS sensor");
  */

  Serial.println(F("BMP280 test"));
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    anyFailures = true;
  }
  Serial.println("Found BMP280 sensor");

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    anyFailures = true;
  }
  Serial.println("card initialized.");
  
  if (anyFailures) {
    Serial.println("Failures in setup. Will try again later");
  } else {
    Serial.println("Setup successful! :)");
  }

  delay(5000);
}


void loop() {
    
  delay(2000);
  unsigned long deltatMs = (micros()/1000 + startTime - lastTime);
  
  // update time
  currentTime += deltatMs;
  lastTime = currentTime;
  
  //pressureMPRLS = mpr.readPressure();
  Serial.println(bme.readPressure());
  temperatureBMP = bme.readTemperature();
  pressureBMP = bme.readPressure(); //convert to Pa
  
  
  updateLowest();
  updateEEPROMRecords();

  String dataString = "";

  dataString += (String)currentTime;
  dataString += ", ";
  //dataString += (String)pressureMPRLS;
  //dataString += ", ";
  dataString += (String)pressureBMP;
  dataString += ", ";
  dataString += (String)temperatureBMP;
  dataString += ",\n";
  
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
    anyFailures = true;
  }

  // update button state
  buttonState = digitalRead(buttonPin);

  if (buttonState) {
    Serial.println("button pressed");
    disableLocationBuzzer = 1;
  }

  doBuzzer();
}
