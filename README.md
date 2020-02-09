[![Build Status](https://travis-ci.org/truhlikfredy/SinglePinCapacitiveSense.svg?branch=master)](https://travis-ci.org/truhlikfredy/SinglePinCapacitiveSense)

# Getting started in 1 minute

[![1min getting started video](https://img.youtube.com/vi/KwPeKHTvGJs/0.jpg)](https://www.youtube.com/watch?v=KwPeKHTvGJs)


# Features

- Requires only **1 digital input** on an AVR.
- It doesn't require any additional capacitor/resistor, **no external components**.
- **Hand-crafted assembly** sampler is very optimized.
- It should be faster than the 2-pin or ADC method. Capable to capture up to 4080 samples with performance of **1.3 clock per sample**!
- Allows being tweaked from the constructors and from the defines as well.
- If small enough samples iterations are used, then on compile-time it will be **auto-detected** and the some return types and internal variables can **shrink** from uint32_t to uint16_t (making footprint smaller)
- Because v2.0 has much higher sampling speed, it can recognize the **pressure** much more **precisely**.

# Disadvantages

- Because using **internal pull-up** (and there is no pull-down) the sensing is done only on the 'charging' cycle and can't be tested again on the 'discharging' cycle. **No discharge measurement** means some accuracy is lost and this system is more sensitive to noise and environment changes. This pull-up resistance is not specified (around 20k-100k) and could be different between devices.
- No way to increase the resistance and no way increase the sensitivity, only touch can be detected, **no proximity sensor**
- Depends on the clock speed, tested on 16MHz, but should work at much lower clocks as now the sampling is very fast.
- Because the C++ templates are used the code size will increase when more sensors are created.
- Will not work on non-AVR Arduinos (if needed I could make ports).
- Will not work well on IRQ heavy applications. To remedy the sensing can block IRQs:

`#define SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ 1`

# Tested devices

|Board | Build on the CI | Tested on real HW |
|------|-----------------|----------------|
|Arduino Duemilanova - ATmega328 - 16MHz | PASS | PASS|
|Arduino Uno | PASS | Do not have HW to test|
|Arduino Nano - ATmega328 | PASS | Do not have HW to test|
|Arduino Mega 2560 | PASS | Do not have HW to test|

Note: To see the CI build logs, visit the [Travis-CI](https://travis-ci.org/truhlikfredy/SinglePinCapacitiveSense) website.


# Defines

The following defines can be configured before including the SinglePinCapacitiveSense.h


To specify what type will be used for the total sum variables and return types.If left undefined then it will be auto-detected with taking SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING and SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT into consideration:

`#define SPC_VAL uint32_t`


To block all IRQs while sampling:

`#define SINGLE_PIN_CAPACITIVE_SENSE_BLOCK_IRQ 1`


How many times the sensor will be sampled (when constructor arguments are left to default), this value is used to calculate if the total sum value will fit into a uint16_t type or uint32_t has to be used. Therefore if sampling in a constructor is used with much higher value, then either specify SPC_VAL by hand or updatethis sampling define so then the SPC_VAL will be detected correctly:

`#define SINGLE_PIN_CAPACITIVE_SENSE_DEFAULT_SAMPLING 16`


To change how many sets of the 16 micro samples have to be taken before the capacitance considered too high and sampling aborted as an error (non-touched result). Has to be lower than 255 and it shouldn't be too low because even untouched pin might take some time to get passed by the sampler:

`#define SINGLE_PIN_CAPACITIVE_SENSE_TIMEOUT 254`


When calibrating the inputs to detect what is the lowest noise level, how many of these have to be returned in the row to consider the global minimum and not some intermittent temporary local minimum/noise. This will make the first X iteration of the touch sensing ineffective as the sensor will be calibrating itself:

`#define SINGLE_PIN_CAPACITIVE_SENSE_STREAK_COUNT 8`

# How this works

Typically the capacitive sensing forms a small capacitor between the sensor and the finger, charging (and discharging) it while measuring the time (or voltage) can estimate the capacitance and from that figure out that a press has happened. Because area and distance can affect capacitance, therefore pressing harder on the surface can increase the capacity and the strength can be detected as well. Microchip's Atmel Qtouch has dedicated HW pins and has good accuracy and sensitivity, but this HW feature is not present everywhere. There is SW QTouch support with as well, but for limited targets. And then there are Arduino libraries implementing this in software, either depending on ADC pin, or 2-pins with an external resistor. This library takes the 2-pin approach, but instead of using separate sending pin with a large resistor, the pin is setup to INPUT and an internal pull-up resistor is enabled. See the unmarked resistor on the left side of the pin's diagram:

![pin's pull-up schema](../assets/images/pullup.png?raw=true)

This resistor is around **20k-100k** and is a magnitude smaller than it would be ideal and the charge up process happens much faster than typically, therefore the code needs to be sampling very quickly. See charge up characteristics when nothing is touched:

![normal operation](../assets/images/free-running.png?raw=true)

When the sensor is touched the capacitance increases and charge-up slows down, but not as radically as it would be desired and this still requires fast sampling, see figure below:

![charging up when touching](../assets/images/press.png?raw=true)

The fact that with **TTL 5V** the input can be considered **HIGH** already at **2V** is making this even more time-critical. On the figures above the voltage can get over 2V because the sampling is not as fast and charged up beyond the 2V before it was tested and pulled down (Note that release 2.0 and above is much faster). Another problem is that the pull-up resistor is not exactly specified and can be different between the devices. So this approach is not as robust/portable as the previously mentioned approaches, but if somebody can tune it for their device/environment/conditions, then it should be fine for hobby projects. However, this shouldn't be used for any critical input on a system that needs to be dependable.

# Why C++ templates are used

Because the code can't be generic, the PORT address and pin have to be hardcoded. If port and pin are generic, they are treated as any value, and 16bit addressing is used (using a pair of 8bit registers, X/Y/Z), pin mask can be generated and stored, but negation has to be evaluated (and stored) as well (just to be able to set/clear/test the bits). Because these can change on runtime the compiler might struggle to optimize this further and will involve many clocks.

With hard-coded approach the compiler can see few things:

- The pin/mask is actually one-hot schema and this can be detected by the compiler, instead of testing value with the immediate AND (ANDI) it can be tested directly by a bit set/clear test as the AVR has instructions to test specific bits.

- That the PORT pointer is not regular 16-bit pointer, for lower address regions there are instructions with I/O direct addressing, for 6-bit addresses (0x20 - 0x5F) there is input (IN) instruction where the address is part of the OP-code and there is no need to use the 16-bit register pairs, it loads the content into a register and with ANDI mask can be tested for the bit. However, for the even lower 5-bit region (0x20 - 0x3F) there are instructions (SBIC/SBIS) that can test for a specific bit is set/clear without even loading the content into a register or needing to invoke ANDI. Not just this combines two steps into one, but both address and bit location are direct and are part of the OP-code, so no need to load registers with the address and bit. 

- Because the compiler knows in advance that the desired port is in the **I/O 5-bit** region and that the mask is **one-hot**, it can effectively utilize the **SBIC** / **SBIS** instructions and drastically speed up the runtime execution, which so essential for this approach to work. And wouldn't be possible if the code would have to be generic and work with any mask and pointer. It's cleaner as well, everything needed is contained in the instruction and doesn't require any other registers to be populated. Now 15 samples are taken with IN instruction into registers and the 16th with SBIS (if it fails only then the previously 15 samples are analyzed, but then the speed doesn't matter). Overflow counter checking is done between the IN samples (~1 extra instruction between 4 sampling instructions).

# References

## Capacitive touch

[https://en.wikipedia.org/wiki/Capacitive_sensing](https://en.wikipedia.org/wiki/Capacitive_sensing)

## AVR

[http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)

[https://www.arduino.cc/en/Reference/PortManipulation](https://www.arduino.cc/en/Reference/PortManipulation)

[https://garretlab.web.fc2.com/en/arduino/inside/hardware/arduino/avr/cores/arduino/Arduino.h/digitalPinToBitMask.html](https://garretlab.web.fc2.com/en/arduino/inside/hardware/arduino/avr/cores/arduino/Arduino.h/digitalPinToBitMask.html)

[http://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf](http://ww1.microchip.com/downloads/en/devicedoc/atmel-0856-avr-instruction-set-manual.pdf)

## ASM in C

[https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1gcc_asm.html](https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1gcc_asm.html)

[https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1io_ops.html](https://www.microchip.com/webdoc/AVRLibcReferenceManual/inline_asm_1io_ops.html)

[http://www.ethernut.de/en/documents/arm-inline-asm.html](http://www.ethernut.de/en/documents/arm-inline-asm.html)

[https://stackoverflow.com/questions/3898435](https://stackoverflow.com/questions/3898435)

## C++ templating

[https://stackoverflow.com/questions/37303968](https://stackoverflow.com/questions/37303968)

[https://stackoverflow.com/questions/115703](https://stackoverflow.com/questions/115703)
