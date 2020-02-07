#include "SinglePinCapacitiveSense.h"

// Whole functionality depends on the internal pull-up resistor, threfore the PUD 
// pull-up-disable has to be off.

// Pin's base address and it's mask has to be hardcoded, specific Arduino pins might have 
// different addresses or masks between some Arduino boards and therefore a runtime check 
// is recomended to confim if the expected Arduino pin matches the hardcoded port address
// and mask. These are hardcoded because it's necesary for performance which is critical
// in sensing capacitance without external resistor.
// Tested on Arduino Duemilanove ATmega328
// Multiple constructors are avaiable, if samples, pressThreshold are not given then,
// default values will be used instead (defaults can be changed in SinglePinCapacitiveSense.h)

// This is to work around "because it is not the address of a variable" error where template
// argument is refused because it's not a pointer to a variable.
constexpr uintptr_t PortD() {
  return (uintptr_t)&PIND;
}

SinglePinCapacitiveSense<PortD(), 4>  capacitivePin2; // Arduino pin 2 is on port D and has mask 4
SinglePinCapacitiveSense<PortD(), 8>  capacitivePin3; // Arduino pin 3 is on port D and has mask 8
SinglePinCapacitiveSense<PortD(), 16> capacitivePin4; // Arduino pin 4 is on port D and has mask 16

// If the defaults are not enough, suply own samples/pressThreshold:
// SinglePinCapacitiveSense<(uintptr_t)&PIND, 4> capacitivePin2(5, 15);

// SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>(uint8_t samples, uint16_t pressThreshold);
// SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>(uint8_t samples);
// SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>();


bool isPinsConfigValid() {
  // Checking if Arduino pin 2 coresponds to the hardcoded values in our capacitivePin2
  Serial.print("Config for capacitivePin2:");
  if (!capacitivePin2.IsValidConfig(2)) return false;

  // Checking if Arduino pin 3 coresponds to the hardcoded values in our capacitivePin3
  Serial.print("Config for capacitivePin3:");
  if (!capacitivePin3.IsValidConfig(3)) return false; 

  // Checking if Arduino pin 4 coresponds to the hardcoded values in our capacitivePin4
  Serial.print("Config for capacitivePin4:");
  if (!capacitivePin4.IsValidConfig(4)) return false; 

  return true;
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(9600);

  // Only needed when switching between different boards to make sure the,
  // base address and mask did not change, remove for production builds.
  if (!isPinsConfigValid()) {
    // If the pins are not configured correctly do not continue, check
    // if the desired pin has correct address/mask.
    while (1);
  }
}


void loop() {    
  if (capacitivePin2.IsPressed() || capacitivePin3.IsPressed() || capacitivePin4.IsPressed() ) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {      
    digitalWrite(LED_BUILTIN, LOW);
  }

  Serial.print("Sensor1= \t");
  Serial.print(capacitivePin2.GetLastMeasurementCalibrated());
  
  Serial.print("\tSensor2= \t");
  Serial.print(capacitivePin3.GetLastMeasurementCalibrated());

  Serial.print("\tSensor3= \t");
  Serial.print(capacitivePin4.GetLastMeasurementCalibrated());
  
  Serial.println();
  
  delay(50); // The loop can go faster if needed, the delay can be completely removed
}
