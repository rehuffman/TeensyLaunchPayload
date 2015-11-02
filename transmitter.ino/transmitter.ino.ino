// include the 10 dof sensor packages
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
// Libraries for XBee
//#include <XBee.h> xbee currently configured in open serial bridge mode
// Libraries for GPS
#include <TinyGPS.h> //interacts with GPS reciever

// include the SD library:
#include <SdFat.h> 
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <spi4teensy3.h>
//MOSI connected to PIN 11
//MISO connected to PIN 12
//SCLK connected to PIN 13
//SS connected to PIN 10

TinyGPS gps;
// XBee xbee = XBee(); xbee currently configured in open serial bridge mode

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

// Chip Selector for SD connected to pin 10 on the Teensy 3.x
const int chipSelect = 4;
SdFat SD;
int fileCounter = 1; 
String filename;

//Disable chip select since running multiple SPI devices
const int8_t DISABLE_CHIP_SELECT = -1;
boolean newData;

/* Print out the sensor data from the 10 DOF to USB for debug*/
void displaySensorDetails(void){
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  bmp.getSensor(&sensor);
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

//debug GPS via USB
void debug(){
  if (Serial){
    float flat, flon;
    unsigned long age; 
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("GPS,");
    Serial.print("TIME:");
    Serial.print(millis() / 1000.0);
    Serial.print(",LAT:");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",LON:");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.println("");  
  
    //debug sensors via USB
    /* Get a new sensor event */
    sensors_event_t event;
     
    /* Display the results (acceleration is measured in m/s^2) */
    accel.getEvent(&event);
    Serial.print(F("ACCEL "));
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    mag.getEvent(&event);
    Serial.print(F("MAG   "));
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
    /* Display the results (gyrocope values in rad/s) */
    gyro.getEvent(&event);
    Serial.print(F("GYRO:  "));
    Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  
  
    /* Display the pressure sensor results (barometric pressure is measure in hPa) */
    bmp.getEvent(&event);
    if (event.pressure){
      /* Display atmospheric pressure in hPa */
      Serial.print(F("PRESS "));
      Serial.print(event.pressure);
      Serial.print(F(" hPa, "));
      /* Display ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      Serial.print(temperature);
      Serial.print(F(" C, "));
      /* Then convert the atmospheric pressure, SLP and temp to altitude    */
      /* Update this next line with the current SLP for better results      */
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature)); 
      Serial.println(F(" m"));
    }
    Serial.println(F(""));
  }
}

void transmitAndWriteSensorData() {
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  /* Get a new sensor event */
  sensors_event_t event;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  dataFile.write("ACCEL ");
  dataFile.write("X: "); dataFile.print(event.acceleration.x); dataFile.write("  ");
  dataFile.write("Y: "); dataFile.print(event.acceleration.y); dataFile.write("  ");
  dataFile.write("Z: "); dataFile.print(event.acceleration.z); dataFile.write("  ");dataFile.println("m/s^2 ");
  /* Send Accel to Xbee */
  Serial3.print(F("ACCEL "));
  Serial3.print("X: "); Serial3.print(event.acceleration.x); Serial3.print("  ");
  Serial3.print("Y: "); Serial3.print(event.acceleration.y); Serial3.print("  ");
  Serial3.print("Z: "); Serial3.print(event.acceleration.z); Serial3.print("  ");Serial3.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  /* Mag to SD */
  mag.getEvent(&event);
  dataFile.write("MAG   ");
  dataFile.write("X: "); dataFile.print(event.magnetic.x); dataFile.write("  ");
  dataFile.write("Y: "); dataFile.print(event.magnetic.y); dataFile.write("  ");
  dataFile.write("Z: "); dataFile.print(event.magnetic.z); dataFile.write("  ");dataFile.println("uT");
 /* Mag to Xbee */
  mag.getEvent(&event);
  Serial3.print(F("MAG   "));
  Serial3.print("X: "); Serial3.print(event.magnetic.x); Serial3.print("  ");
  Serial3.print("Y: "); Serial3.print(event.magnetic.y); Serial3.print("  ");
  Serial3.print("Z: "); Serial3.print(event.magnetic.z); Serial3.print("  ");Serial3.println("uT");

  /* Display the results (gyrocope values in rad/s) */
  /* Gyro to SD*/
  gyro.getEvent(&event);
  dataFile.write("GYRO:  ");
  dataFile.write("X: "); dataFile.print(event.gyro.x); dataFile.write("  ");
  dataFile.write("Y: "); dataFile.print(event.gyro.y); dataFile.write("  ");
  dataFile.write("Z: "); dataFile.print(event.gyro.z); dataFile.write("  ");dataFile.println("rad/s ");  
  /* Gyro to Xbee */
  gyro.getEvent(&event);
  Serial3.print(F("GYRO:  "));
  Serial3.print("X: "); Serial3.print(event.gyro.x); Serial3.print("  ");
  Serial3.print("Y: "); Serial3.print(event.gyro.y); Serial3.print("  ");
  Serial3.print("Z: "); Serial3.print(event.gyro.z); Serial3.print("  ");Serial3.println("rad/s ");
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure){
    /* Display atmospheric pressure in hPa */
    // To Xbee
    Serial3.print(F("PRESS "));
    Serial3.print(event.pressure);
    Serial3.print(F(" hPa, "));
    // To SD
    dataFile.write("PRESS ");
    dataFile.print(event.pressure);
    dataFile.write(" hPa, ");
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial3.print(temperature);
    Serial3.print(F(" C, "));
    dataFile.print(temperature);
    dataFile.print((" C, "));
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial3.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure,temperature)); 
    dataFile.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure,temperature)); 

    Serial3.println(F(" m"));
    dataFile.println(F(" m"));
  }
  
  Serial3.println(F(""));
  dataFile.println(F(""));
  dataFile.close();
}


void transmitAndWriteGPSData() {
  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  float flat, flon;
  unsigned long age; 
  gps.f_get_position(&flat, &flon, &age);
  //transmit to Xbee and SD 
    
  Serial3.print("GPS:");
  Serial3.print("TIME:");
  Serial3.print(millis() / 1000.0);
  // Serial3.print(TinyGPS::_time);
  Serial3.print(",LAT:");
  Serial3.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial3.print(",LON:");
  Serial3.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  Serial3.println("");

  // write to SD card  
  if(dataFile.isOpen()){
    dataFile.print("GPS:");
    dataFile.print("TIME:");
    dataFile.print(millis() / 1000.0);
    dataFile.print(",LAT:");
    dataFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    dataFile.print(",LON:");
    dataFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    dataFile.println("");
  }  
  dataFile.close();
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial3.begin(9600);
   //xbee.setSerial(Serial3); xbee currently configured in open serial bridge mode
  /* Set 10DOF to +/-16G */
  Wire.beginTransmission(0x32); //address of LSM303
  Wire.write(0x23);  //address of the sensitivity control register LSM303
  Wire.write(0x30); //setting the range 0x00 -> +/-2g, 0x10 -> +/-4g, 0x20 -> +/- 8g, 0x30 -> +/-16g
  Wire.endTransmission();
  
  delay(5000);
  
  
  /* Initialise the sensors */
  if(!accel.begin()){
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()){
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin()){
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* debug SD card */
  SD.begin(chipSelect);
  String launch = "launch";
  String fileCounterStr = String(fileCounter);
  String txt = ".txt";
  filename = launch + fileCounterStr + txt; 
  while(SD.exists(filename.c_str())){                                
    fileCounter += 1;
    fileCounterStr = String(fileCounter);
    filename = launch + fileCounterStr + txt; 
    File dataFile = SD.open(filename.c_str(), FILE_WRITE);
    dataFile.close();
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    if (Serial){
      Serial.println("Card failed, or not present");
      // don't do anything more:
      return;
    }
    if (Serial){
      Serial.println("card initialized.");
    }
  }
}

void loop(void){
  newData = false;
  // get GPS data
  for (unsigned long start = millis(); millis() - start < 150;)
  {
    while(Serial1.available()){
      int c = Serial1.read();
      if (gps.encode(c)){
        newData = true;
      }
    }
  }
  //if data is available transmit it for GPS
  if(newData){
    transmitAndWriteGPSData();
  }
  //five rounds of sensor per 1 gps reading
  for(int i =0; i < 5; i++) {
    transmitAndWriteSensorData();
    delay(200); /* measurements per second  1000 = 1 sec intervals*/
  }
  debug();
}
