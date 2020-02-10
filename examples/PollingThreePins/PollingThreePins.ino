// If needed the following defines:
// SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ
// SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING
// SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT
// SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT
// SINGLE_PIN_CAPACITIVE_SENSE_DEBOUNCE_COUNT
// SPC_VAL
// Have to be defined before the SinglePinCapacitiveSense.h include
#include "SinglePinCapacitiveSense.h"

// Copyright 2020 Anton Krug - MIT License

// Whole functionality depends on the internal pull-up resistor, therefore the
// PUD pull-up-disable has to be off.

// Pin's base address and it's mask has to be hardcoded, specific Arduino pins
// might have different addresses or masks between some Arduino boards and
// therefore a runtime check is recommended to confirm if the expected Arduino
// pin matches the hardcoded port address and mask. These are hardcoded because
// it's necessary for performance which is critical in sensing capacitance
// without external resistor.

// Multiple constructors are available, if samples, pressThreshold are not
// given then, default values will be used instead (default can be changed
// with define:
// SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING

// This is to work around "because it is not the address of a variable" error
// where template argument is refused because it's not a pointer to a variable.
// https://stackoverflow.com/questions/37303968
constexpr uintptr_t PortD() {
  return (uintptr_t)&PIND;
}

// https://www.theengineeringprojects.com/wp-content/uploads/2018/10/introduction-to-Arduino-Duemilanove-2-3.png
SinglePinCapacitiveSense<PortD(), 2> sensePin2;  // Arduino Pin2 = AVR PortD2
SinglePinCapacitiveSense<PortD(), 3> sensePin3;  // Arduino Pin3 = AVR PortD3
SinglePinCapacitiveSense<PortD(), 4> sensePin4;  // Arduino Pin4 = AVR PortD3

// If the defaults are not enough, supply own samples/pressThreshold:
// SinglePinCapacitiveSense<PortD(), 2> sensePin2(5, 15);

// SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>(uint8_t samples, uint16_t pressThreshold); 
// SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>(uint8_t samples); 
// SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>();

// Might not apply to all Arduino AVR devices, but in general should be correct:
// Usually Arduino pin 0-7 are PORTD, 8-13 PORTB (do not use analogue ports for
// this). If not exactly knowing how to setup the mask and port address then
// just make the pin with some estimated values (even wrong ones) and call
// IsValidConfig(<DESIRED_ARDUINO_PIN>) it will validate the settings and if
// they are not correct it will output on the UART what is expected for that
// Arduino pin.


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
