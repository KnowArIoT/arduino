#include <NewPing.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTMath.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>


// Ultrasound defines
#define TRIGGER_PIN_1 11
#define ECHO_PIN_1 12
#define TRIGGER_PIN_2 10
#define ECHO_PIN_2 13
#define MAX_DISTANCE 200

// Gas sensor defines
#define GAS_SENSOR_PIN A0

// GPS defines
#define GPSECHO false

// Shadow clock timer
unsigned int tick = 0;

// Ultrasound setup
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned int oversamples = 1;
const unsigned int filterSizeU = 10;
float filterValueU1, filterValueU2;

uint8_t timerTrigger;

// Gas sensor setup
const unsigned int filterSizeGas = 10;
float filterValueGas;
boolean toggleGas = 0;

// GPS
SoftwareSerial gpsSerial(3,2);
Adafruit_GPS GPS(&gpsSerial);
boolean usingGPSInterrupt = false;
uint32_t timer = millis();

// IMU setup
RTIMU *imu;
RTFusionRTQF fusion;
RTIMUSettings settings;


// Sensor tags
const String u1 = "u1:";      // Ultrasound sensor 1
const String u2 = "u2:";      // Ultrasound sensor 2
const String gas = "gas:";    // Gas sensor
const char* accl = "accl:";  // Accelerometer
const char* gyro = "gyro:";  // Gyroscope
const char* mag = "mag:";    // Magnetometer
const String gpsTime = "gpsTime:";
const String gpsDate = "gpsDate:";
const String gpsFix = "gpsFix:";
const String gpsSignalQuality = "gpsSignalQuality:";
const String gpsLatLong = "gpsLatLong:";
const String gpsSpeed = "gpsSpeed:";
const String gpsAngle = "gpsAngle:";

extern "C" {
  void setupTimer(void);
}

void processSensors() {

  if((tick & 0x01) == 0) {
    sampleIMU(imu);
    RTMath::display(accl, (RTVector3&)imu->getAccel());
    Serial.print('\n');
    RTMath::display(gyro, (RTVector3&)imu->getGyro());
    Serial.print('\n');
    RTMath::display(mag, (RTVector3&)imu->getCompass());
    Serial.print('\n');
  }
  if((tick & 0x07) == 0) { 
    sampleUltrasound(sonar1.ping_cm(), filterValueU1, filterSizeU, oversamples);
    printValue(u1, filterValueU1);
    
    sampleUltrasound(sonar2.ping_cm(), filterValueU2, filterSizeU, oversamples);
    printValue(u2, filterValueU2);
  }

  if((tick & 0x3F) == 0) {
    sampleGasSensor(filterValueGas, filterSizeGas); 
    printValue(gas, filterValueGas); 
  }

}

void setup() {
  Serial.begin(115200);
  filterValueU1 = sonar1.ping_cm();
  filterValueU2 = sonar2.ping_cm();
  filterValueGas = analogRead(GAS_SENSOR_PIN);

  // Initialize IMU
  imu = RTIMU::createIMU(&settings);
  initIMU(imu, fusion);

  timerTrigger = 0;
  setupTimer();

  initGPS(GPS, gpsSerial);

  Serial.println("Setup complete");
}

void loop() {
  if (timerTrigger) {
    timerTrigger = 0;
    processSensors();
  }
  sampleGPS(GPS);
}

// sensor - sensor tag ending with ":". Eg. "u1:"
void printValue(String sensor, float value) {
  Serial.print(sensor);
  Serial.print(value);
  Serial.print('\n');
}

// Simlpe IIR filter
void processSensorValue(float &filteredValue, unsigned int newValue, unsigned int _filterSize) {
  filteredValue = (((float)(_filterSize - 1)) / _filterSize) * filteredValue + (1.0f / _filterSize) * newValue;
  return filteredValue;
}

// Function sampling and filtering ultrasound sensor
void sampleUltrasound(unsigned int sample, float &filterValue, unsigned int filterSize, unsigned int oversamples) {
  for (int i = 0; i < oversamples; i++) {
    processSensorValue(filterValue, sample, filterSize);
  }
}

// Sample and filter gas sensor
void sampleGasSensor(float &filterValue, unsigned int filterSize) {
  unsigned int gasRead = analogRead(GAS_SENSOR_PIN);
  processSensorValue(filterValue, gasRead, filterSizeGas);
}

// Function initializing an IMU
void initIMU(RTIMU *imu, RTFusionRTQF &fusion) {
  Wire.begin();
  imu->IMUInit();
  fusion.setSlerpPower(0.02);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

// Sample and process the IMU
void sampleIMU(RTIMU *imu) {
  int loopCount = 1;
  while (imu->IMURead()) { // This somehow terminates after a while.. Automagic <3.. Guess you can read the source code to figure out how
    if (++loopCount >= 10)
      continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  }
}

// Init GPS
void initGPS(Adafruit_GPS &GPS, SoftwareSerial &gpsSerial) {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  setupGPSInterrupt();
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void sampleGPS(Adafruit_GPS &GPS) {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))  
      return;
    printGPS(GPS);
  }
}

void printGPS(Adafruit_GPS gps) {
  // Print time
  Serial.print(gpsTime);
  Serial.print(GPS.hour, DEC);      Serial.print('.');
  Serial.print(GPS.minute, DEC);    Serial.print('.');
  Serial.print(GPS.seconds, DEC);   Serial.print('.');
  Serial.println(GPS.milliseconds);

  // Print date
  Serial.print(gpsDate);
  Serial.print(GPS.day, DEC);       Serial.print('.');
  Serial.print(GPS.month, DEC);     Serial.print('.');
  Serial.println(GPS.year, DEC);

  // Print fix
  Serial.print(gpsFix); 
  Serial.println((int)GPS.fix);

  // Print quality
  Serial.print(gpsSignalQuality); 
  Serial.println((int)GPS.fixquality);
  
  if (GPS.fix) {
    // Print long and lat
    Serial.print(gpsLatLong);
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(","); 
    Serial.println(GPS.longitudeDegrees, 4);

    // Print speed (knots)
    Serial.print(gpsSpeed); 
    Serial.println(GPS.speed);

    // Print angle
    Serial.print(gpsAngle); 
    Serial.println(GPS.angle);
  }
}

void setupGPSInterrupt() {
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}


// Called whenever TCNT1 overflows
ISR(TIMER1_OVF_vect) {
  timerTrigger = 1;
  TCNT1 = 34286; // 64 hz by filling the counter
  
  tick < 65536 ? tick++ : tick = 0; // Shadow tick counter
}

