#include "SinglePinCapacitiveSense.h"

// Copyright 2020 Anton Krug - MIT License

// See the PollingThreePins for more comments and explanation how this works

// Showing how to use Arduino with different pin mapping
constexpr uintptr_t PortE() {
  return (uintptr_t)&PINE;
}

constexpr uintptr_t PortG() {
  return (uintptr_t)&PING;
}

// https://grobotronics.com/images/companies/1/datasheets/arduino-mega-pinout-diagram.jpg
SinglePinCapacitiveSense<PortE(), 4> sensePin2;  // ADK Pin2 = AVR PortE4
SinglePinCapacitiveSense<PortE(), 5> sensePin3;  // ADK Pin3 = AVR PortE5
SinglePinCapacitiveSense<PortG(), 5> sensePin4;  // ADK Pin4 = AVR PortG5


bool isPinsConfigValid() {
  // Check if ADK pin 2 coresponds to the values in our sensePin2
  Serial.print("Config for sensePin2:");
  if (!sensePin2.IsValidConfig(2)) return false;

  // Check if ADK pin 3 coresponds to the values in our sensePin3
  Serial.print("Config for sensePin3:");
  if (!sensePin3.IsValidConfig(3)) return false;

  // Check if ADK pin 4 coresponds to the values in our sensePin4
  Serial.print("Config for sensePin4:");
  if (!sensePin4.IsValidConfig(4)) return false;

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
  // For the first 8 iterations of this loop, the sense will not trigger as they
  // are calibrating, this can be changed with define:
  // SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT
  // And it will affect how quickly the calibration.

  static uint8_t count = 0;

  // If any of these capacitive sensors is pressed then light up the LED
  if (sensePin2.IsPressed() || sensePin3.IsPressed() || sensePin4.IsPressed()) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if ((count % 4) == 0) {
    // Update the UART 4x less often, allows LED to be refreshed more often
    // without hammering the UART with so many messages
    
    Serial.print("Sensor1= \t");
    Serial.print(sensePin2.GetLastMeasurementCalibrated());

    Serial.print("\tSensor2= \t");
    Serial.print(sensePin3.GetLastMeasurementCalibrated());

    Serial.print("\tSensor3= \t");
    Serial.print(sensePin4.GetLastMeasurementCalibrated());

    Serial.println();    
  }

  count++;
  delay(20);  // The delay can be completely removed if needed
}
