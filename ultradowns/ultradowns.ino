#include <NewPing.h>

#define TRIGGER_PIN_1 11
#define ECHO_PIN_1 12
#define MAX_DISTANCE 200

#define TRIGGER_PIN_2 10
#define ECHO_PIN_2 13

NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned int oversamples = 1;
const unsigned int filterSize = 10;
float filterValue1, filterValue2;

float processSensorValue(float &filteredValue, unsigned int newValue, unsigned int _filterSize) {
  filteredValue = (((float)(_filterSize-1))/_filterSize) * filteredValue + (1.0f / _filterSize) * newValue;
  return filteredValue;
}

void setup() {
   Serial.begin(9600);
   filterValue1 = sonar1.ping_cm();
   filterValue2 = sonar2.ping_cm();
}
 
void loop() {
   for (int i = 0; i < oversamples; i++) {
     unsigned int uS1 = sonar1.ping_cm();
     unsigned int uS2 = sonar2.ping_cm();
     processSensorValue(filterValue1, uS1, filterSize);
     processSensorValue(filterValue2, uS2, filterSize);
   }
   Serial.print("u1:");
   Serial.print(filterValue1);
   Serial.print('\n');
   Serial.print("u2:");
   Serial.print(filterValue2);
   Serial.print('\n');  
}
