#include <NewPing.h>

// Ultrasound defines
#define TRIGGER_PIN_1 11
#define ECHO_PIN_1 12
#define TRIGGER_PIN_2 10
#define ECHO_PIN_2 13
#define MAX_DISTANCE 200

// Gas sensor defines
#define GAS_SENSOR_PIN A0

// Ultrasound setup 
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned int oversamples = 1;
const unsigned int filterSizeU = 10;
float filterValueU1, filterValueU2;

// Gas sensor setup
const unsigned int filterSizeGas = 10;
float filterValueGas;



// Sensor tags
const String u1 = "u1:";      // Ultrasound sensor 1
const String u2 = "u2:";      // Ultrasound sensor 2
const String gas = "gas:";    // Gas sensor

void setup() {
   Serial.begin(9600);
   filterValueU1 = sonar1.ping_cm();
   filterValueU2 = sonar2.ping_cm();
   filterValueGas = analogRead(GAS_SENSOR_PIN);

   
}
 
void loop() {
   // Sampling and processing ultrasound sensor
   sampleUltrasound(filterValueU1, filterSizeU, oversamples);
   sampleUltrasound(filterValueU2, filterSizeU, oversamples);

   // Sampling and processing gas sensor
   //gasRead = analogRead(GAS_SENSOR_PIN);
   //processSensorValue(filterValueGas, gasRead, filterSizeGas);
   sampleGasSensor(filterValueGas, filterSizeGas);

   // Print values
   printValue(u1, filterValueU1);
   printValue(u2, filterValueU2);
   printValue(gas, filterValueGas);
}

// sensor - sensor tag ending with ":". Eg. "u1:"
void printValue(String sensor, float value) {
  Serial.print(sensor);
  Serial.print(value);
  Serial.print('\n');
}

// Simlpe IIR filter
void processSensorValue(float &filteredValue, unsigned int newValue, unsigned int _filterSize) {
  filteredValue = (((float)(_filterSize-1))/_filterSize) * filteredValue + (1.0f / _filterSize) * newValue;
  return filteredValue;
}

// Function sampling and filtering ultrasound sensor
void sampleUltrasound(float &filterValue, unsigned int filterSize, unsigned int oversamples) {
   for (int i = 0; i < oversamples; i++) {
     unsigned int uS = sonar1.ping_cm();
     processSensorValue(filterValue, uS, filterSize);
   }
}

// Sample and filter gas sensor
void sampleGasSensor(float &filterValue, unsigned int filterSize) {
  asm(
  unsigned int gasRead = analogRead(GAS_SENSOR_PIN);
  processSensorValue(filterValue, gasRead, filterSizeGas);
}

