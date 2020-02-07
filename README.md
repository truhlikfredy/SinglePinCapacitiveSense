
# Features

- Requires only **1 digital input** on an AVR
- Doesn't require any additional capacitor/resistor, **no external components**
- Should be faster than 2 pin or ADC method

# Disadvantages

- Because using **internal pull-up** (and there is no pull-down) the sensing is done only on the 'charging' cycle and can't be tested again on the 'discharging' cycle. **No discharge measurement** means some accuracy is lost and this system is more sensitive to noise and environment changes. This pull-up resistance is not specified and could be different between devices.
- No way to increase the resistance and no way increase the sensitivity, only touch can be detected, **no proximity sensor**
- Depends on fast clock, tested on **16MHz**, this **could be improved** and the performance still can be better and should be possible to support lower clocks.
- Because the C++ templates are used the code size will increase when more sensors are created.

# Tested devices

- Arduino Duemilanova - ATmega328

# How this works

Typically the capacitive sensing forms a small capacitor between the sensor and the finger, charging (and discharging) it while measuring the time (or voltage) can estimate the capacitance and from that figure out that a press has happened. Because area and distance can affect capacitance therefore pressing harder on the surface can increase the capacity and the strength can be detected as well. Microchip's Atmel Qtouch has dedicated HW pins have good accuracy and sensitivity, but this HW feature is not present everywhere. There is SW QTouch support with as well, but for a selected targets. And then there are Arduino libraries implementing this in software, either depending on ADC pin, or 2-pins with an external resistor. This library takes the 2-pin approach, but instead of using separate sending pin with a large resistor, the pin is setup to INPUT and a internal pull-up resistor is enabled. See the unmarked resistor on the left side of the pin's diagram:

![pin's pull-up schema](../assets/images/pullup.png?raw=true)

This resistor is around **20k-100k** and is magnitude smaller than it would be ideal and the charge up process happens much faster than typically, therefore the code needs to be sampling very quickly. See charge up characteristics when nothing is touched:

![normal operation](../assets/images/free-running.png?raw=true)

When sensor is touched the capacitance increases and charge-up slows down, but not as radically as it would be desired and this still requires fast sampling, see figure below:

![charging up when touching](../assets/images/press.png?raw=true)

The fact that whit **TTL 5V** the input can be considered **HIGH** already at **2V** is making this even more time critical. On the figures above the voltage can get over 2V because the sampling is not as fast and charged up beyond the 2V before it was tested and pulled down. Other problem is that the pull-up resistor is not exactly specified and can be different between the devices. So this approach is not as robust/portable as the previously mentioned approaches, but if somebody can tune it for their device/environment/conditions, then it should be fine for hobby projects. However this shouldn't be used for any critical input on a system which needs to be dependable.

# Why C++ templates are used

Because the code can't be generic, PORT address and pin have to be hardcoded. If port and pin are generic, they are treated as any value, and 16bit addressing is used (using a pair of 8bit registers, X/Y/Z), pin mask can be generated and stored, but negation has to be evaluated (and stored) as well and that is just to be able set/clear/test the bits. Because these can change on runtime the compiler might struggle to optimize this further and will involve many clocks.

With hard-coded approach the compiler can see few things:

  - The pin/mask is actually one-hot schema and this can be detected by the compiler, instead of testing value with the immediate AND (ANDI) it can be tested directly by a bit set/clear test as the AVR has instructions to test specific bits.

  - That the PORT pointer is not regular 16-bit pointer, for lower address regions there are instructions with I/O direct addressing, for 6-bit addresses (0x20 - 0x5F) there is input (IN) instruction where the address is part of the OP-code and there is no need to use the 16-bit register pairs, it loads the content into a register and with ANDI mask can be tested for the bit. However for even lower 5-bit region (0x20 - 0x3F) there are instructions (SBIC/SBIS) which can test for a specific bit being set/clear without even loading the content into a register or needing to invoke ANDI. Not just this combines two steps into one, but both address and bit location are direct and are part of the OP-code, so no need to load registers with the address and bit. 
  
  - Because compiler knows in advance that the desired port is in the **I/O 5-bit** region and that the mask is **one-hot**, it can effectively utilized the **SBIC** / **SBIS** instructions and drastically speed up the runtime execution, which so essential for this approach to work. And wouldn't be possible if the code would have to be generic and work with any mask and pointer. It's cleaner as well, everything needed is contained in the instruction and doesn't require any other registers to be populated. Even when this speed is great, I reckon that the sampling rate can be still significantly improved.
