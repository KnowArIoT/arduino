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
#define gpsTX 3
#define gpsRX 4

// Shadow clock timer
unsigned int tick = 0;

// Ultrasound setup
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned int oversamples = 1;
const unsigned int filterSizeU = 10;
float filterValueU1, filterValueU2;

// Gas sensor setup
const unsigned int filterSizeGas = 10;
float filterValueGas;
boolean toggleGas = 0;

// GPS
//SoftwareSerial gpsSerial(
//Adafruit_GPS GPS

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

extern "C" {
  void setupTimer(void);
}

void setup() {
  Serial.begin(115200);
  //filterValueU1 = sonar1.ping_cm();
  //filterValueU2 = sonar2.ping_cm();
  filterValueGas = analogRead(GAS_SENSOR_PIN);

  // Initialize IMU
  //imu = RTIMU::createIMU(&settings);
  //initIMU(imu, fusion);

  setupTimer();

}

void loop() {
  // Sampling and processing ultrasound sensor
  /*unsigned int uS1 = sonar1.ping_cm();
    sampleUltrasound(uS1, filterValueU1, filterSizeU, oversamples);
    unsigned int uS2 = sonar2.ping_cm();
    sampleUltrasound(uS2, filterValueU2, filterSizeU, oversamples);
  */

  // Sampling and processing gas sensor
  //gasRead = analogRead(GAS_SENSOR_PIN);
  //processSensorValue(filterValueGas, gasRead, filterSizeGas);
  //sampleGasSensor(filterValueGas, filterSizeGas);

  // Print values
  /*printValue(u1, filterValueU1);
    printValue(u2, filterValueU2);*/
  //sampleIMU(imu);
  //RTMath::display("Mag:", (RTVector3&)imu->getCompass());
  //Serial.println();
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

// Called whenever TCNT1 overflows
ISR(TIMER1_OVF_vect) {
  TCNT1 = 34286; // 64 hz by filling the counter
  
  tick < 65536 ? tick++ : tick = 0; // Shadow tick counter

  if(tick % 2 == 0) {
    sampleIMU(imu);
    RTMath::display(accl, (RTVector3&)imu->getAccel());
    Serial.print('\n');
    RTMath::display(gyro, (RTVector3&)imu->getGyro());
    Serial.print('\n');
    RTMath::display(mag, (RTVector3&)imu->getCompass());
    Serial.print('\n');
  }
  if(tick % 8 == 0) { 
    sampleUltrasound(sonar1.ping_cm(), filterValueU1, filterSizeU, oversamples);
    printValue(u1, filterValueU1);
    
    sampleUltrasound(sonar2.ping_cm(), filterValueU2, filterSizeU, oversamples);
    printValue(u2, filterValueU2);
  }

  if(tick % 64 == 0) {
    sampleGasSensor(filterValueGas, filterSizeGas); 
    printValue(gas, filterValueGas); 
  }
}

