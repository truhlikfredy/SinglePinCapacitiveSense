#ifndef SinglePinCapacitiveSense_h
#define SinglePinCapacitiveSense_h

#include <Arduino.h>

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ
#define SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ 0
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING
// How often the pin will get sampled by default
#define SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING 50
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT
// When the sampler gives up, it needs to be equal or lower than 255 and lower
// than 65535/sampling
#define SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT          200
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT
// How many same smallestMeasurement have to be in the row to consider for new
// measurementOffset (and it will take 4 measurements before any trigger can be
// detected)
#define SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT     4    
#endif


// The class implementation and declaration have are in the same header because it's a templated class

template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
class SinglePinCapacitiveSense {
  public:    
    SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>(uint8_t samples, uint16_t pressThreshold);
    SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>(uint8_t samples);
    SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>();
    
    bool     IsValidConfig(uint8_t arduinoPin);  // Check if the hard-coded values match with the Arduino pin values
    uint16_t SampleAllSamples(void);             // Just do a measurement of all this->samples samples
    bool     IsPressed(void);                    // Do measurement and decided if the sensor was pressed or not
    uint16_t GetLastMeasurementRaw(void);        // Return the value as measured
    uint16_t GetLastMeasurementCalibrated(void); // Return the measured value - the measurementOffset (primitive high-pass filtering)
    void     Calibrate(void);                    // To reset the measurementOffset (normally invoked from the constructor)
    
  private:
    uint8_t  samples;                            // How many samples to make
    uint16_t pressThreshold;                     // How much the measurement has to be above the smallestMeasurement to count as press
    uint16_t lastMeasurement;                    // Hold value of the last measurement done
    uint16_t measurementOffset;                  // Keep track what was the smallest measurement which was measured consistently for many measurements
    uint16_t smallestMeasurement;                // Keep track what was the smallest measurement done so far (to calculated the GetLastMeasurementCalibrated()
    uint8_t  smallestMeasurementStreak;          // How many of last measurements in the row were same as the smallest measurement

    void     ConstructorCommon(void);            // Prepare the pin and fields to expected state
    uint16_t SampleOnce(void);                   // Will measure the capacity once
    void     SampleCleanup(void);                // Has be invoked after the measurement is finished
};


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SinglePinCapacitiveSense(uint8_t samples, uint16_t pressThreshold) { 
  this->samples        = samples;
  this->pressThreshold = pressThreshold;

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SinglePinCapacitiveSense(uint8_t samples) { 
  this->samples        = samples;
  this->pressThreshold = this->samples;  // To consider a press: lastMeasurement >= measurementOffset + sample

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SinglePinCapacitiveSense() { 
  this->samples        = SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING;
  this->pressThreshold = this->samples;  // To consider a press: lastMeasurement >= measurementOffset + sample

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::ConstructorCommon(void) {
  this->Calibrate();
  *((volatile uint8_t *)PINx_ADDR+1) |= PIN_MASK;  // DDRx  Switch to output
  *((volatile uint8_t *)PINx_ADDR+2) &= ~PIN_MASK; // PORTx Output will be LOW
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::Calibrate(void) {
  this->smallestMeasurement       = UINT16_MAX;
  this->measurementOffset         = UINT16_MAX;
  this->smallestMeasurementStreak = 0;  
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
uint16_t SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SampleOnce(void) {
  uint8_t count = 0;

#if SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ == 1
  noInterrupts();
#endif
  *((volatile uint8_t *)PINx_ADDR+1) &= ~PIN_MASK; // DDRx Set direction to input
  *((volatile uint8_t *)PINx_ADDR+2) |= PIN_MASK;  // PORTx Enable pull-up

  while ( !(*((volatile uint8_t *)PINx_ADDR) & PIN_MASK) && count < SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT )  { 
    // Read PINx input and counting how long it took to charge
    count++;
  }

#if SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ == 1
  interrupts();
#endif

  this->SampleCleanup();

  return count;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SampleCleanup(void) {
#ifdef SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ  
  interrupts();  // Enabling interrupts in case they were not already, depending on version/variation of SampleOnce, this might happen
#endif
  
  // Pulling down the residual capacity by setting pin to output low
  // and waiting a moment to make sure it's drained
  *((volatile uint8_t *)PINx_ADDR+2) &= ~PIN_MASK; // PORTx Disable pull-up input (output will be LOW)
  *((volatile uint8_t *)PINx_ADDR+1) |= PIN_MASK;  // DDRx  Switch from input to output
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
uint16_t SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::SampleAllSamples(void) {
  this->lastMeasurement = 0;
  
  for (uint8_t sample=0; sample < this->samples; sample++) {
    // Sample multiple times before returining acumulated result
    this->lastMeasurement += this->SampleOnce();
  }

  // Count how many measurements in the row are same or smaller than the smallest measurement
  if (this->lastMeasurement <= this->smallestMeasurement) {
    this->smallestMeasurementStreak++;
  } else {
    this->smallestMeasurementStreak = 0;
  }

  // If enough of them were consistently low, consider it as the new offset
  if (this->smallestMeasurementStreak > SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT) {
    this->smallestMeasurementStreak = 0;
    this->measurementOffset = this->smallestMeasurement;
  }

  // Update the smallest measurement if the currently last measurement is smaller
  if (this->lastMeasurement < this->smallestMeasurement) {
    this->smallestMeasurement = this->lastMeasurement;
  }
  
  return this->lastMeasurement;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
bool SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::IsPressed(void) {
  this->SampleAllSamples();
  
  if (this->measurementOffset == UINT16_MAX) {
    // If the pin is still calibrating, do not trigger any presses
    return false;
  } else {
    return  this->lastMeasurement >= (this->pressThreshold + this->measurementOffset) ? true : false;  
  }
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
uint16_t SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::GetLastMeasurementRaw(void) {
  return this->lastMeasurement;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
uint16_t SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::GetLastMeasurementCalibrated(void) {
  if (this->lastMeasurement < this->measurementOffset) {
    // If the measured value is below our expected minimim, then do not return
    // negative value and just return no press. (return type is unsigned anyway)
    return 0;
  } else {
    return this->lastMeasurement - this->measurementOffset;  
  }
}


template<uintptr_t PINx_ADDR, uint8_t PIN_MASK>
bool SinglePinCapacitiveSense<PINx_ADDR, PIN_MASK>::IsValidConfig(uint8_t arduinoPin) {
  bool status = true;

  // Check pin's port base address
  volatile uint8_t* port = portInputRegister(digitalPinToPort(arduinoPin));
  if ((uintptr_t)port != PINx_ADDR) {
    Serial.print(" PINx_ADDR should be ");
    Serial.print((uint16_t)(uintptr_t)port);
    Serial.print(".");
    status = false;
  }

  // Check pin's bit mask
  if (digitalPinToBitMask(arduinoPin) != PIN_MASK) {
    Serial.print(" PIN_MASK should be ");
    Serial.print(digitalPinToBitMask(arduinoPin));
    Serial.print(".");
    status = false;
  }

  // Both address and mask passed
  if (status) {
    Serial.print(" OK");
  }

  Serial.println();
  return status;
}


#endif
