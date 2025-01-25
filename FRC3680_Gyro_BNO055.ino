#define DEBUG_SERIAL Serial

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <Adafruit_GPS.h>

#define GPSSerial Serial1

// The Adafruit_GPS object
Adafruit_GPS gps;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

Adafruit_BNO055 mpu = Adafruit_BNO055(55);

struct {
  float x;
  float y;
  float z;
} acc_off, ang_off, acc_in, ang_in, mag_in, avg_ang_in, avg_acc_in, adj_out, pos_out, vel_out;

const int intPin = 2;
const int accXPin = 12;
const int accYPin = 13;
const int GyroZPin = 14;
const int InputPin = 26;
const float Rad2Deg = 180 / 3.14;
const double Deg2Rad = 3.14 / 180; //trig functions require radians, BNO055 outputs degrees

const float AccUnits = (9.8 * 2) * 2; // 2G
const float GyroUnits = 360.0 * 2; // Full Circle
const int outputRange = 4096;

const int CalSam = 20;
const bool hasMag = false;
const float alpha = 0.89; // filter signal strength
const int Delay = 10;

uint32_t elapsedTime, previousTime, lastPrint;

int gpsReady;
bool gpsEncoded;
bool gpsAcquired;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  //while(!Serial) {}

  Serial.println("start delay");
  delay(1000);
  Serial.println("delay complete");

  // Try to initialize
  while (!mpu.begin()) {
    DEBUG_SERIAL.println("Failed to find BNO055 chip");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("BNO055 Found!");

  restore_calibration();

  gps.begin(9600);

  delay(1000);

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  mpu.setExtCrystalUse(true);

  Serial.println();
  delay(1000);

  analogReadResolution(12);
  pinMode(InputPin, INPUT);

  analogWriteFreq(500000);
  analogWriteRange(outputRange);
  analogWriteResolution(12);

  pinMode(accXPin, OUTPUT);
  pinMode(accYPin, OUTPUT);
  pinMode(GyroZPin, OUTPUT);

  // calculate_offsets();
  // calculate_filters();
}

void loop() {
  uint32_t currentTime = millis();

  if (previousTime == 0) {
    elapsedTime = 0;
  } else {
    elapsedTime = currentTime - previousTime;
  }

  double TimeSlice = elapsedTime / 1000.0;

  get_values();
  get_gpsvalues();
  // adjust_values();
  //filter_values();

  // vel_out.x = vel_out.x + (acc_in.x * TimeSlice);
  // pos_out.x = pos_out.x + (vel_out.x * TimeSlice);

  // gps coordinates
  pos_out.x = acc_in.x;

  // vel_out.y = vel_out.y + (acc_in.y * TimeSlice);
  // pos_out.y = pos_out.y + (vel_out.y * TimeSlice);

  // gps coordinates
  pos_out.y = acc_in.y;

  vel_out.z = vel_out.z + (acc_in.z * TimeSlice);
  // pos_out.z = pos_out.z + (vel_out.z * elapsedTime);

  //  use pos_out.z for angle
  pos_out.z = ang_in.x; // X is 'up'
  
  float angleScale = outputRange / GyroUnits; // 12-bit value of angle range
  float accelScale = outputRange / AccUnits; // 39.24; // 12-bit value of -19.62m/s to 19.62m/s range

  adj_out.x = pos_out.x * accelScale + outputRange / 2; // center zero at range midpoint
  adj_out.y = pos_out.y * accelScale + outputRange / 2; // center zero at range midpoint
  adj_out.z = pos_out.z * angleScale + outputRange / 2; // center zero at range midpoint

  if (currentTime - lastPrint > 500) {
    //Serial.print("AnalogRead ("); Serial.print(InputPin); Serial.print("): "); Serial.println(analogRead(InputPin));
    // Serial.print("gpsReady: "); Serial.print(gpsReady); Serial.print("; gpsEncoded: "); Serial.print(gpsEncoded); Serial.print("; gpsAcquired: "); Serial.println(gpsAcquired);

    print_analog_output();

    print_calstatus();
    adafruit_bno055_offsets_t calibData;
    mpu.getSensorOffsets(calibData);
    print_calibration(calibData);

    lastPrint = currentTime;
  }

  // send x,y pos and yaw to PWM
  analogWrite(accXPin, adj_out.x);
  analogWrite(accYPin, adj_out.y);
  analogWrite(GyroZPin, adj_out.z);

  previousTime = millis();

  delay(10);
}

void get_gpsvalues() {
  while (gps.available()) {
    char c = gps.read();
  }

  if (gps.newNMEAreceived()) {
      if (!gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  if (gps.fix) {
    Serial.print("Location: ");
    Serial.print(gps.latitude, 4); Serial.print(gps.lat);
    Serial.print(", ");
    Serial.print(gps.longitude, 4); Serial.println(gps.lon);

    // acc_in.x = gps.latitude;
    // acc_in.y = gps.longitude;

    acc_in.x = gps.lat;
    acc_in.y = gps.lon;
  }

  // bool printed = false;
  // while (true) {
  //   int available = Serial1.available();
  //   //Serial.print(available, DEC);
  //   if (available > 0) {
  //     printed = true;
  //     int val = Serial1.read();
  //     Serial.print((char)val);
  //     gpsEncoded = gps.encode(val);
  //     if (gpsEncoded) {
  //       gpsAcquired = gps.location.isValid();
  //       if (gpsAcquired) {
  //         acc_in.x = gps.location.lat();
  //         acc_in.y = gps.location.lng();
  //       }
  //     }
  //   } else {
  //     break;
  //   }
  // }
  // if (printed) {
  //   Serial.println();
  // }
}

void get_values() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  mpu.getCalibration(&system, &gyro, &accel, &mag);
  
  if (system > 0) {
    sensors_event_t g, a;
    mpu.getEvent(&g, Adafruit_BNO055::VECTOR_EULER);
    mpu.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    if (gyro > 0) {
      ang_in.x = g.orientation.x;
      ang_in.y = g.orientation.y;
      ang_in.z = g.orientation.z;
    } else {
      ang_in.x = 0;
      ang_in.y = 0;
      ang_in.z = 0;
    }

    if (accel > 0) {
      // acc_in.x = a.acceleration.x;
      // acc_in.y = a.acceleration.y;
      // acc_in.z = a.acceleration.z;
    } else {
      acc_in.x = 0;
      acc_in.y = 0;
      acc_in.z = 0;
    }

    // if (hasMag) {
    //   mag_in.x = imu.mag_x_ut();
    //   mag_in.y = imu.mag_y_ut();
    //   mag_in.z = imu.mag_z_ut();
    // }
  } else {
    ang_in.x = 0;
    ang_in.y = 0;
    ang_in.z = 0;

    acc_in.x = 0;
    acc_in.y = 0;
    acc_in.z = 0;
  }
}

void filter_values() {
  // TODO: not sure this does what we want.  We probably don't want to include 'actual' movements in the filter

  // float newValue = (float) analogRead(A0);
  // average = (strength * average) + ((1.0-strength) * newValue);
  // float result = newValue - average;

  avg_ang_in.x = (alpha * avg_ang_in.x) + ((1.0 - alpha) * ang_in.x);
  ang_in.x = ang_in.x - avg_ang_in.x;
  avg_ang_in.y = (alpha * avg_ang_in.y) + ((1.0 - alpha) * ang_in.y);
  ang_in.y = ang_in.x - avg_ang_in.y;
  avg_ang_in.z = (alpha * avg_ang_in.z) + ((1.0 - alpha) * ang_in.z);
  ang_in.z = ang_in.z - avg_ang_in.z;

  avg_acc_in.x = (alpha * avg_acc_in.x) + ((1.0 - alpha) * acc_in.x);
  acc_in.x = acc_in.x - avg_acc_in.x;
  avg_acc_in.y = (alpha * avg_acc_in.y) + ((1.0 - alpha) * acc_in.y);
  acc_in.y = acc_in.x - avg_acc_in.y;
  avg_acc_in.z = (alpha * avg_acc_in.z) + ((1.0 - alpha) * acc_in.z);
  acc_in.z = acc_in.z - avg_acc_in.z;
}

void adjust_values() {
  ang_in.x = ang_in.x - ang_off.x;
  ang_in.y = ang_in.y - ang_off.y;
  ang_in.z = ang_in.z - ang_off.z;

  acc_in.x = acc_in.x - acc_off.x;
  acc_in.y = acc_in.y - acc_off.y;
  acc_in.z = acc_in.z - acc_off.z;

  if (hasMag) {
    mag_in.x = mag_in.x - 0;
    mag_in.y = mag_in.y - 0;
    mag_in.z = mag_in.z - 0;
  }
}

void calculate_offsets() {
  Serial.println("Calculating Offsets");

  for (int i = 0; i < CalSam; i++) {
    Serial.print(".");

    get_values();

    ang_off.x += ang_in.x;
    ang_off.y += ang_in.y;
    ang_off.z += ang_in.z;

    acc_off.x += acc_in.x;
    acc_off.y += acc_in.y;
    acc_off.z += acc_in.z;

    delay(500);
  }

  Serial.println(".");

  print_offsets();

  ang_off.x = ang_off.x / CalSam;
  ang_off.y = ang_off.y / CalSam;
  ang_off.z = ang_off.z / CalSam;

  acc_off.x = acc_off.x / CalSam;
  acc_off.y = acc_off.y / CalSam;
  acc_off.z = acc_off.z / CalSam;

  avg_ang_in.x = ang_off.x;
  avg_ang_in.y = ang_off.y;
  avg_ang_in.z = ang_off.z;

  avg_acc_in.x = acc_off.x;
  avg_acc_in.y = acc_off.y;
  avg_acc_in.z = acc_off.z;

  print_offsets();
}

void calculate_filters() {
  Serial.println("Calculating Filter Values");

  for (int i = 0; i < CalSam; i++) {
    Serial.print(".");

    get_values();

    avg_ang_in.x = (alpha * avg_ang_in.x) + ((1.0 - alpha) * ang_in.x);
    ang_in.x = ang_in.x - avg_ang_in.x;
    avg_ang_in.y = (alpha * avg_ang_in.y) + ((1.0 - alpha) * ang_in.y);
    ang_in.y = ang_in.x - avg_ang_in.y;
    avg_ang_in.z = (alpha * avg_ang_in.z) + ((1.0 - alpha) * ang_in.z);
    ang_in.z = ang_in.z - avg_ang_in.z;

    avg_acc_in.x = (alpha * avg_acc_in.x) + ((1.0 - alpha) * acc_in.x);
    acc_in.x = acc_in.x - avg_acc_in.x;
    avg_acc_in.y = (alpha * avg_acc_in.y) + ((1.0 - alpha) * acc_in.y);
    acc_in.y = acc_in.x - avg_acc_in.y;
    avg_acc_in.z = (alpha * avg_acc_in.z) + ((1.0 - alpha) * acc_in.z);
    acc_in.z = acc_in.z - avg_acc_in.z;

    delay(500);
  }

  Serial.println(".");

  print_offsets();

  ang_off.x = avg_ang_in.x;
  ang_off.y = avg_ang_in.y;
  ang_off.z = avg_ang_in.z;

  acc_off.x = avg_acc_in.x;
  acc_off.y = avg_acc_in.y;
  acc_off.z = avg_acc_in.z;

  print_offsets();
}

void print_offsets() {
  Serial.print("ang_off: (");
  Serial.print(ang_off.x);
  Serial.print(", ");
  Serial.print(ang_off.y);
  Serial.print(", ");
  Serial.print(ang_off.z);
  Serial.print("); acc_off: (");
  Serial.print(acc_off.x);
  Serial.print(", ");
  Serial.print(acc_off.y);
  Serial.print(", ");
  Serial.print(acc_off.z);
  Serial.println(");");  
}

void print_analog_output() {
  // Print the values on the serial monitor
  Serial.print("accel: (");
  Serial.print(acc_in.x);
  Serial.print(", ");
  Serial.print(acc_in.y);
  Serial.print(", ");
  Serial.print(acc_in.z);
  Serial.print("); velocity: (");
  Serial.print(vel_out.x);
  Serial.print(", ");
  Serial.print(vel_out.y);
  Serial.print(", ");
  Serial.print(vel_out.z);
  Serial.print("); position: (");
  Serial.print(pos_out.x);
  Serial.print(", ");
  Serial.print(pos_out.y);
  Serial.print(", ");
  Serial.print(pos_out.z);

  if (hasMag) {
    Serial.print("); mag: (");
    Serial.print(mag_in.x);
    Serial.print(", ");
    Serial.print(mag_in.y);
    Serial.print(", ");
    Serial.print(mag_in.z);
  }
  
  Serial.print("); output: (");
  Serial.print(adj_out.x);
  Serial.print(", ");
  Serial.print(adj_out.y);
  Serial.print(", ");
  Serial.print(adj_out.z);
  Serial.println(");");

  // Serial.print("pos_out.z: ");
  // Serial.print(pos_out.z);
  // Serial.print("; pos_out.x: ");
  // Serial.print(pos_out.x);
  // Serial.print("; pos_out.y: ");
  // Serial.print(pos_out.y);
  // Serial.println(";");
}

void print_roll_pitch_yaw() {
  Serial.print("Yaw, Pitch, Roll: (");
  Serial.print(ang_in.x, 2);
  Serial.print(", ");
  Serial.print(ang_in.y, 2);
  Serial.print(", ");
  Serial.print(ang_in.z, 2);

  if (hasMag) {
    Serial.print("); mag_in: (");
    Serial.print(mag_in.x, 2);
    Serial.print(", ");
    Serial.print(mag_in.y, 2);
    Serial.print(", ");
    Serial.print(mag_in.z, 2);
  }

  Serial.print("); lin_acc_in: (");
  Serial.print(acc_in.x, 2);
  Serial.print(", ");
  Serial.print(acc_in.y, 2);
  Serial.print(", ");
  Serial.print(acc_in.z, 2);
  Serial.println(");");
}

void print_imu_output() {
  Serial.print("acc_in: (");
  Serial.print(acc_in.x);
  Serial.print(",");
  Serial.print(acc_in.y);
  Serial.print(",");
  Serial.print(acc_in.z);
  Serial.print("); gyro: (");
  Serial.print(ang_in.x);
  Serial.print(",");
  Serial.print(ang_in.y);
  Serial.print(",");
  Serial.print(ang_in.z);

  if (hasMag) {
    Serial.print("); mag_in: (");
    Serial.print(mag_in.x);
    Serial.print(",");
    Serial.print(mag_in.y);
    Serial.print(",");
    Serial.print(mag_in.z);
  }

  Serial.println(");");
}

void print_calstatus() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  mpu.getCalibration(&system, &gyro, &accel, &mag);

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print("; G:");
  Serial.print(gyro, DEC);
  Serial.print("; A:");
  Serial.print(accel, DEC);
  Serial.print("; M:");
  Serial.print(mag, DEC);
}

void print_calibration(adafruit_bno055_offsets_t calibData)
{
  Serial.print("; Accelerometer: (");
  Serial.print(calibData.accel_offset_x); Serial.print(", ");
  Serial.print(calibData.accel_offset_y); Serial.print(", ");
  Serial.print(calibData.accel_offset_z); Serial.print(")");

  Serial.print("; Gyro: (");
  Serial.print(calibData.gyro_offset_x); Serial.print(", ");
  Serial.print(calibData.gyro_offset_y); Serial.print(", ");
  Serial.print(calibData.gyro_offset_z); Serial.print(")");

  Serial.print("; Mag: (");
  Serial.print(calibData.mag_offset_x); Serial.print(", ");
  Serial.print(calibData.mag_offset_y); Serial.print(", ");
  Serial.print(calibData.mag_offset_z); Serial.print(")");

  Serial.print("; Accel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("; Mag Radius: ");
  Serial.print(calibData.mag_radius);
  Serial.println();
}

void restore_calibration()
{
  // Sys:3; G:3; A:3; M:3; Accelerometer: (-47, -95, -12); Gyro: (-2, -3, 0); Mag: (182, 308, 238); Accel Radius: 1000; Mag Radius: 1098

  adafruit_bno055_offsets_t calibData;

  calibData.accel_offset_x = -47;
  calibData.accel_offset_y = -95;
  calibData.accel_offset_z = -12;

  calibData.gyro_offset_x = -2;
  calibData.gyro_offset_y = -3;
  calibData.gyro_offset_z = 0;

  calibData.mag_offset_x = 182;
  calibData.mag_offset_y = 308;
  calibData.mag_offset_z = 238;

  calibData.accel_radius = 1000;

  calibData.mag_radius = 1098;

  mpu.setSensorOffsets(calibData);

  print_calibration(calibData);
}

