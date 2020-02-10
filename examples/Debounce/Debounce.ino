#include "SinglePinCapacitiveSense.h"

// Copyright 2020 Anton Krug - MIT License

// See the PollingThreePins example for more comments and explanation how this 
// works and how it can be configured


constexpr uintptr_t PortD() {
  return (uintptr_t)&PIND;
}

// https://www.theengineeringprojects.com/wp-content/uploads/2018/10/introduction-to-Arduino-Duemilanove-2-3.png
SinglePinCapacitiveSense<PortD(), 2> sensePin2;  // Arduino Pin2 = AVR PortD2
SinglePinCapacitiveSense<PortD(), 3> sensePin3;  // Arduino Pin3 = AVR PortD3
SinglePinCapacitiveSense<PortD(), 4> sensePin4;  // Arduino Pin4 = AVR PortD3


bool isPinsConfigValid() {
  // Check if Arduino pin 2 coresponds to the values in our sensePin2
  Serial.print("Config for sensePin2:");
  if (!sensePin2.IsValidConfig(2)) return false;

  // Check if Arduino pin 3 coresponds to the values in our sensePin3
  Serial.print("Config for sensePin3:");
  if (!sensePin3.IsValidConfig(3)) return false;

  // Check if Arduino pin 4 coresponds to the values in our sensePin4
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

  // If any of these capacitive sensors is pressed then light up the LED.
  // Use either IsPressedDebounced() and the:
  // SINGLE_PIN_CAPACITIVE_SENSE_DEBOUNCE_COUNT will be used as default.
  // Or provide your own value
  if (sensePin2.IsPressedDebounced() || sensePin3.IsPressedDebounced(15) ||  sensePin4.IsPressedDebounced()) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if ((count % 8) == 0) {
    // Update the UART 16x less often, allows LED to be refreshed more often
    // without hammering the UART with so many messages
    
    Serial.print("Sensor1= \t");
    Serial.print(sensePin2.GetDebouncedState());

    Serial.print("\tSensor2= \t");
    Serial.print(sensePin3.GetDebouncedState());

    Serial.print("\tSensor3= \t");
    Serial.print(sensePin4.GetDebouncedState());

    Serial.println();    
  }

  count++;
  // The delay can be completely removed if needed, but debounce count should
  // be increased to have similar effect
  delay(5);
}
