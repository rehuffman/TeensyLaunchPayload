#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <TinyGPS.h>

#include "serial_util.h"

char *ftoa(char *buf, float d, int precision) {
  long wholePart = (long) d;
  // Deposit the whole part of the number.
  itoa(wholePart,buf,10);
  // Now work on the faction if we need one.
  if (precision > 0) {
    // We do, so locate the end of the string and insert
    // a decimal point.
    char *endOfString = buf;
    while (*endOfString != '\0') endOfString++;
    *endOfString++ = '.';
    // Now work on the fraction, be sure to turn any negative
    // values positive.
    if (d < 0) {
      d *= -1;
      wholePart *= -1;
    }
    float fraction = d - wholePart;
    while (precision > 0) {
      // Multiply by ten and pull out the digit.
      fraction *= 10;
      wholePart = (long) fraction;
      *endOfString++ = '0' + wholePart;
      // Update the fraction and move on to the
      // next digit.
      fraction -= wholePart;
      precision--;
    }
    // Terminate the string.
    *endOfString = '\0';
  }
  return buf;
}

// Helper for dumping out details during initialization.
void sensorDetailsOut(const sensor_t& sensor, Stream *serial) {
  pprintf(serial, "\n----------- %s ----------\n"
          "Sensor:       %s\n"
          "Driver Ver:   %d\n"
          "Unique ID:    %d\n"
          "------------------------------------\n\n",
          sensorTypeName(sensor), sensor.name, sensor.version, sensor.sensor_id);
}

const char *sensorTypeName(const sensor_t& sensor) {
  const char *type_name;
  switch (sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    type_name = "ACCELEROMETER";
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    type_name = "MAGNETOMETER";
    break;
  case SENSOR_TYPE_GYROSCOPE:
    type_name = "GYROSCOPE";
    break;
  case SENSOR_TYPE_PRESSURE:
    type_name = "BAROMETER";
    break;
  default:
    type_name = "UNKNOWN";
    break;
  }
  return type_name;
}

const char *sensorTypeUnits(const sensor_t& sensor) {
  const char *unit_name;
  switch (sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    unit_name = "m/s^2";
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    unit_name = "uT";
    break;
  case SENSOR_TYPE_GYROSCOPE:
    unit_name = "rad/s";
    break;
  case SENSOR_TYPE_PRESSURE:
    unit_name = "hPa";
    break;
  default:
    unit_name = "unknown units";
    break;
  }
  return unit_name;
}

static inline char *strf(char *buf, float f) {
  return ftoa(buf, f, 5);
}

static inline void pprint_sensor_vec(const sensors_vec_t& vec, Print *p) {
  char buf[16]; // No %f in sprintf? What the fuck.
  char buf2[16];
  char buf3[16];
 
  pprintf(p, "X: %s Y: %s Z: %s ", strf(buf, vec.x), strf(buf2, vec.y), strf(buf3, vec.z));
}

UnifiedSensorWriter::UnifiedSensorWriter(Adafruit_Sensor *sensor) : sensor_(sensor) {
  sensor_t s;
  sensor->getSensor(&s);
  name_ = sensorTypeName(s);
  unit_ = sensorTypeUnits(s);
}

void UnifiedSensorWriter::Update() {
  sensor_->getEvent(&event_);
}

void UnifiedSensorWriter::Write(Print *p) {
  sensor_t s;
  sensor_->getSensor(&s);

  // The name is always written.
  pprintf(p, "%s: ", name_);

  // Write out body, format and source of data in event union depends on type...
  switch (s.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    pprint_sensor_vec(event_.acceleration, p);
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    pprint_sensor_vec(event_.magnetic, p);
    break;
  case SENSOR_TYPE_GYROSCOPE:
    pprint_sensor_vec(event_.gyro, p);
    break;
  }

  // The unit is always written.
  pprintf(p, " %s\n", unit_);
}

AltitudeSensorWriter::AltitudeSensorWriter(Adafruit_BMP085_Unified *sensor) : bmp_(sensor) {
  sensor_t s;
  sensor->getSensor(&s);
  name_ = sensorTypeName(s);
  unit_ = sensorTypeUnits(s);
}

void AltitudeSensorWriter::Update() {
  bmp_->getEvent(&event_);
}

void AltitudeSensorWriter::Write(Print *p) {
  // TODO(mgyenik) Find clean way to update this.
  const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  float alt = bmp_->pressureToAltitude(seaLevelPressure, event_.pressure); 
  char buf[16]; // No %f in sprintf? What the fuck.
  pprintf(p, "%s %s %s\n", name_, strf(buf, alt), "ft");
}

void GPSSensorWriter::Update() {
  // We fully drain the serial stream into the TinyGPS decoder, which will
  // update the latitude and longitude as encode() is called.
  while (stream_->available()) {
    gps_->encode(stream_->read());
  }
}

void GPSSensorWriter::Write(Print *p) {
  float lat;
  float lon;
  unsigned long age;
  gps_->f_get_position(&lat, &lon, &age);

  char buf[16]; // No %f in sprintf? What the fuck.
  char buf2[16];
  pprintf(p, "GPS: LAT: %s, LON: %s AGE: %lu\n", strf(buf, lat), strf(buf2, lon), age);
}