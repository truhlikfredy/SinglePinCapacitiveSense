#ifndef SinglePinCapacitiveSense_h
#define SinglePinCapacitiveSense_h

// Copyright 2020 Anton Krug - MIT License

#include <Arduino.h>

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ
#define SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ 0
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING
// How often the pin will get sampled by default
#define SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING 16
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT
// When the sampler gives up, it needs to be equal or lower than 255 and lower
// than 65535/sampling (if SPC_VAL uint16_t is used). Giving it value too low
// might disregard genuine presses
#define SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT          254
#endif

#ifndef SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT
// How many same smallestMeasurement have to be in the row to consider for new
// measurementOffset (and it will take 4 measurements before any trigger can be
// detected)
#define SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT     8
#endif

#ifndef SPC_VAL
// Decide what type the cumulative value will have, if taking small measurements
// and using low sampling count then uint16_t might be used. It will try to detect
// if 16-bit unsigned int is enough, or 32-bit has to be used.
#if (16 * SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT * SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING) > UINT16_MAX
#define SPC_VAL uint32_t // It might not fit into 16-bit so use 32-bit
#else
#define SPC_VAL uint16_t // Worst case scenario should fit into the 16-bit
#endif

#endif

// The class implementation and declaration have are in the same header because it's a templated class

template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
class SinglePinCapacitiveSense {
  public:    
    SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>(uint8_t samples, uint16_t pressThreshold);
    SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>(uint8_t samples);
    SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>();
    
    bool     IsValidConfig(uint8_t arduinoPin);  // Check if the hard-coded values match with the Arduino pin values
    SPC_VAL  SampleAllSamples(void);             // Just do a measurement of all this->samples samples
    bool     IsPressed(void);                    // Do measurement and decided if the sensor was pressed or not
    SPC_VAL  GetLastMeasurementRaw(void);        // Return the value as measured
    SPC_VAL  GetLastMeasurementCalibrated(void); // Return the measured value - the measurementOffset (primitive high-pass filtering)
    void     Calibrate(void);                    // To reset the measurementOffset (normally invoked from the constructor)
    
  private:
    uint8_t  samples;                            // How many samples to make
    uint16_t pressThreshold;                     // How much the measurement has to be above the smallestMeasurement to count as press
    SPC_VAL  lastMeasurement;                    // Hold value of the last measurement done
    SPC_VAL  measurementOffset;                  // Keep track what was the smallest measurement which was measured consistently for many measurements
    SPC_VAL  smallestMeasurement;                // Keep track what was the smallest measurement done so far (to calculated the GetLastMeasurementCalibrated()
    uint8_t  smallestMeasurementStreak;          // How many of last measurements in the row were same as the smallest measurement

    void     ConstructorCommon(void);            // Prepare the pin and fields to expected state
    uint16_t SampleOnce(void);                   // Will measure the capacity once
    void     SampleCleanup(void);                // Has be invoked after the measurement is finished
};


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SinglePinCapacitiveSense(uint8_t samples, uint16_t pressThreshold) { 
  this->samples        = samples;
  this->pressThreshold = pressThreshold;

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SinglePinCapacitiveSense(uint8_t samples) { 
  this->samples        = samples;
  this->pressThreshold = this->samples;  // To consider a press: lastMeasurement >= measurementOffset + sample

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SinglePinCapacitiveSense() { 
  this->samples        = SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING;
  this->pressThreshold = this->samples;  // To consider a press: lastMeasurement >= measurementOffset + sample

  this->ConstructorCommon();
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::ConstructorCommon(void) {
  this->Calibrate();

  // Should be safe even if IRQ happened between these two lines
  *((volatile uint8_t *)PINx_ADDR+2) &= ~(1 << PIN_BIT); // PORTx Output will be LOW (Input to High-Z)
  *((volatile uint8_t *)PINx_ADDR+1) |=   1 << PIN_BIT;  // DDRx  Switch to output
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::Calibrate(void) {
  this->smallestMeasurement       = UINT16_MAX;
  this->measurementOffset         = UINT16_MAX;
  this->smallestMeasurementStreak = 0;  
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
uint16_t SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SampleOnce(void) {
  // 16-bit counter concatinated from two 8-bit counters
  uint8_t minor = 0; // Minor is expected to be 0-15
  uint8_t major = 0; // Major should be capped with SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT

#if SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ == 1
  noInterrupts();
#endif

  // Should be safe even if IRQ happened between these two lines
  *((volatile uint8_t *)PINx_ADDR+1) &= ~(1 << PIN_BIT); // DDRx Set direction to input
  *((volatile uint8_t *)PINx_ADDR+2) |=  (1 << PIN_BIT); // PORTx Enable pull-up

  // Reserve 15 registers as buffer
  uint8_t b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14;

  // http://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf
  // https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1gcc_asm.html
  // https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1io_ops.html
  // http://www.ethernut.de/en/documents/arm-inline-asm.html
  // https://stackoverflow.com/questions/3898435

  // Sample the input 16 times in row, control logic is spread out so it will
  // not create big jitter stall of sampling after 16 samples are taken
  // but spread out and smaller stalls.
  // IN is inert to SREG and we can spread the control logic between sampling
  asm (
    "sample%=: \n\t"
    "in %[reg0],   %[addr] \n\t"
    "in %[reg1],   %[addr] \n\t"
    "in %[reg2],   %[addr] \n\t"
    "in %[reg3],   %[addr] \n\t"
    "inc %[major] \n\t"                // Increment the major counter
    "in %[reg4],   %[addr] \n\t"
    "in %[reg5],   %[addr] \n\t"
    "in %[reg6],   %[addr] \n\t"
    "in %[reg7],   %[addr] \n\t"
    "cpi %[major], %[major_max] \n\t"  // Compare major counter with SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT
    "in %[reg8],   %[addr] \n\t"
    "in %[reg9],   %[addr] \n\t"  
    "in %[reg10],  %[addr] \n\t"
    "in %[reg11],  %[addr] \n\t"
    "brcc timeout%= \n\t"              // Branch if carry cleared (major > SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT)
    "in %[reg12],  %[addr] \n\t"
    "in %[reg13],  %[addr] \n\t"
    "in %[reg14],  %[addr] \n\t"
    "sbis %[addr], %[bit] \n\t"        // Skip if bit in I/O is set, no need to read the sample into a register
    "rjmp sample%= \n\t"               // After 16 samples the pin was not set yet, so continue sampling
    // This whole loop can sample 16 samples in 21 clocks (jump included) :
    // 16 x 1clk samples, 3 x 1clk count logic, 1 x 2clk repeat jump.
    // Averaging 1.3 clocks per sample (upto 4080 samples when SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT is 255 )


    // Go through all 15 registers and count how long they were not set
    // The binary search was tempting, but buffer is too small and caused
    // a lot of jumping. And at this point we are not in the hurry anyway.
    // This is not critical part of the sampling and binary search might have
    // introduced bugs, hard readability and not easy to scale (if more/or less
    // registers will be added to the buffer)
    "sbrs %[reg0], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg1], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg2], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg3], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg4], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg5], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg6], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg7], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg8], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg9], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg10], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg11], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg12], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg13], %[bit] \n\t"
    "inc %[minor] \n\t"

    "sbrs %[reg14], %[bit] \n\t"
    "inc %[minor] \n\t"

    "rjmp end%= \n\t"                  // Finished counting


    // Major counter timeouted, return 0
    "timeout%=: \n\t"                  
    "clr %[major] \n\t"                // clear major => eor major,major

    // Return our regular major + minor results
    "end%=:"


    : [reg0]  "=r"(b0), 
      [reg1]  "=r"(b1),
      [reg2]  "=r"(b2),
      [reg3]  "=r"(b3),
      [reg4]  "=r"(b4),
      [reg5]  "=r"(b5),
      [reg6]  "=r"(b6),
      [reg7]  "=r"(b7),
      [reg8]  "=r"(b8),
      [reg9]  "=r"(b9),
      [reg10] "=r"(b10),
      [reg11] "=r"(b11),
      [reg12] "=r"(b12),
      [reg13] "=r"(b13),
      [reg14] "=r"(b14),
      [major] "+d"(major),    // Have to use 'd' because I want to use CPI which only works on higher 16 registers
      [minor] "+r"(minor)
    : [addr]  "I"(PINx_ADDR - __SFR_OFFSET), // Same effect as _SFR_IO_ADDR(PINx_ADDR), changing absolute address to IO address
      [bit]   "I"(PIN_BIT),
      [major_max] "M"(SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT)
  );

#if SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ == 1
  interrupts();
#endif

  this->SampleCleanup();

  return major << 4 | minor;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
void SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SampleCleanup(void) {
#if SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ == 1
  interrupts();  // Enabling interrupts in case they were not already, depending on version/variation of SampleOnce, this might happen
#endif
  
  // Pulling down the residual capacity by setting pin to output low
  // and waiting a moment to make sure it's drained. Should be safe even if IRQ happened in the middle
  *((volatile uint8_t *)PINx_ADDR+2) &= ~(1 << PIN_BIT); // PORTx Disable pull-up input (output will be LOW)
  *((volatile uint8_t *)PINx_ADDR+1) |=  (1 << PIN_BIT); // DDRx  Switch from input to output
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SPC_VAL SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::SampleAllSamples(void) {
  this->lastMeasurement = 0;
  
  for (uint8_t sample=0; sample < this->samples; sample++) {
    // Sample multiple times before returining acumulated result
    this->lastMeasurement += this->SampleOnce();
  }

  // Update the smallest measurement if the currently last measurement is smaller
  // but ingore the 0/timeout measurements
  if (this->lastMeasurement < this->smallestMeasurement && this->lastMeasurement != 0) {
    this->smallestMeasurement = this->lastMeasurement;
  }

  // Count how many measurements in the row are same or smaller than the smallest measurement
  if (this->lastMeasurement == this->smallestMeasurement) {
    this->smallestMeasurementStreak++;
  } else {
    this->smallestMeasurementStreak = 0;
  }

  // If enough of them were consistently low, consider it as the new offset
  if (this->smallestMeasurementStreak > SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT) {
    this->smallestMeasurementStreak = 0;
    this->measurementOffset = this->smallestMeasurement;
  }

  return this->lastMeasurement;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
bool SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::IsPressed(void) {
  this->SampleAllSamples();
  
  if (this->measurementOffset == UINT16_MAX) {
    // If the pin is still calibrating, do not trigger any presses
    return false;
  } else {

    if (this->lastMeasurement >= (this->pressThreshold + this->measurementOffset)) {
      return true;
    } else {
      return false;
    }
  }
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SPC_VAL SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::GetLastMeasurementRaw(void) {
  return this->lastMeasurement;
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
SPC_VAL SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::GetLastMeasurementCalibrated(void) {
  if (this->lastMeasurement < this->measurementOffset) {
    // If the measured value is below our expected minimim, then do not return
    // negative value and just return no press. (return type is unsigned anyway)
    return 0;
  } else {
    return this->lastMeasurement - this->measurementOffset;  
  }
}


template<uintptr_t PINx_ADDR, uint8_t PIN_BIT>
bool SinglePinCapacitiveSense<PINx_ADDR, PIN_BIT>::IsValidConfig(uint8_t arduinoPin) {
  bool status = true;

  // Check pin's port base address
  volatile uint8_t* port = portInputRegister(digitalPinToPort(arduinoPin));
  if ((uintptr_t)port != PINx_ADDR) {
    Serial.print(" PINx_ADDR should be ");
    Serial.print((uint16_t)(uintptr_t)port);
    Serial.print(".");
    status = false;
  }

  // Check pin's bit
  if (digitalPinToBitMask(arduinoPin) != 1 << PIN_BIT) {
    Serial.print(" PIN_BIT should be ");
    Serial.print(15 - __builtin_clz(digitalPinToBitMask(arduinoPin)));
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
