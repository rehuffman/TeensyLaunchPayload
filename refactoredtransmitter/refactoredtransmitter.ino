#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <TinyGPS.h>
#include <spi4teensy3.h>
#include <string>
#include <SdFat.h> 
#include <stdio.h>
#include <stdlib.h>
#include "serial_util.h"

TinyGPS gps;

// Instantiate sensors with a unique ID.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

// Sensor writers hold samples and serialize them.
UnifiedSensorWriter accelWriter(&accel);
UnifiedSensorWriter gyroWriter(&gyro);
UnifiedSensorWriter magWriter(&mag);
AltitudeSensorWriter bmpWriter(&bmp);
GPSSensorWriter gpsWriter(&gps, &Serial1); // GPS connected to Serial1.

SensorWriter *writers[] = {
  &accelWriter,
  &gyroWriter,
  &magWriter,
  &bmpWriter,
  &gpsWriter,
};

// This flag enables debug sensor data dumps on the default Serial stream.
const bool debug = true;

// Chip Selector for SD connected to pin 10 on the Teensy 3.x
const int SDChipSelect = 4;
SdFat SD;
File dataFile;
String filename;

// Total number of ms that the loop should take. 200ms is ~5Hz update rate.
const unsigned long LOOP_TIME_MS = 200;

// Print out the sensor data from the 10 DOF to USB for debug.
void displaySensorDetails(void) {
  sensor_t sensor;

  accel.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  gyro.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  mag.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);
  bmp.getSensor(&sensor);
  sensorDetailsOut(sensor, &Serial);

  delay(500);
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial3.begin(9600);
  Wire.beginTransmission(0x32); //address of LSM303
  Wire.write(0x23);  //address of the sensitivity control register LSM303
  Wire.write(0x30); //setting the range 0x00 -> +/-2g, 0x10 -> +/-4g, 0x20 -> +/- 8g, 0x30 -> +/-16g
  Wire.endTransmission();
  SD.begin(SDChipSelect);
  int fileCounter = 1;
  String launch = "launch";
  String txt = ".txt";
  String fileCounterStr = String(fileCounter);
  filename = launch + fileCounterStr + txt;
  while(SD.exists(filename.c_str())){
    fileCounter += 1;
    fileCounterStr = String(fileCounter);
    filename = launch + fileCounterStr + txt; 
  }
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  dataFile.close();
  Serial.println("Let's launch some rockets!\n");

  // Initialize the sensors.
  if(!accel.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin()){
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  displaySensorDetails();
  Serial.print("\nInitializing SD card...");
  if (!SD.begin(SDChipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  Serial.println("card initialized.");
}

void loop(void){
  unsigned long loop_start = millis();

  // Sample each sensor and write the output to the SD card log file and the
  // XBee on Serial3. If debugging is enabled, sensors are also written to
  // Serial for viewing with the Serial Monitor.
  for (int i = 0; i < ARRAY_SIZE(writers); i++) {
    writers[i]->Update();
    writers[i]->Write(&Serial3);
    if (dataFile.isOpen()) {
      writers[i]->Write(&dataFile);
      dataFile.sync(); // sync changes to file and file allocation table, etc...
    }
    if (debug) {
      writers[i]->Write(&Serial);
    }
  }

  // Figure out how long to wait for a constant update rate. This is not
  // extremely accurate, but works well enough for ms resolution. Note that if
  // the loop actually took more time that LOOP_TIME_MS, the delay is clamped
  // to 0.
  unsigned long loop_end = millis();
  unsigned long loop_time = loop_end-loop_start;
  unsigned long loop_delay = LOOP_TIME_MS > loop_time ? LOOP_TIME_MS - loop_time : 0;
  delay(loop_delay);
}
