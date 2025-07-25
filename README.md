![20250623_204616_470_1DXII copy](https://github.com/user-attachments/assets/606b6dc2-5ae6-4be6-a74b-a8d9c2276296)


# Micro-DMM

This is the PCB design and code for a 4.5 digit DMM/ instrumentation module. It is designed and written primairly for Arduino-style development boards.

I've developed this DMM in my work testing and troubleshooting electronics to make my life easier. 

I've had Deep-Wiki index my documentation. It did a mostly okay job. You can read its summary here:
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/oihdesigns/Micro-DMM)


![20250623_205912_480_1DXII copy](https://github.com/user-attachments/assets/18e67b72-3f34-445f-8bca-6235bc45dc89)

## The main features are:

- The most recent designs have **open-lead detection in voltmeter mode**. I believe this makes it the first voltmeter in exisitance that can automatically, safely, and positively tell the difference between "there is low impedance between these points" vs "current does not and can not flow between these two points." Please see the white paper for details. 
- Using keyboard emulation to directly type values into a computer for me. (Extremely useful when troubleshooting a populated PCB - you can quickly ohm out both the problem board and a good one directly into Excel to look for different impedances between given contacts.)
- Very sensitive low-resistance readings (accurate to better than 0.01 Ohms with proper lead zeroing).
- LED blinks when I'm reading logic level voltage or higher.
- Simultaneous current and voltage measurement abilities (requires a Hall sensor-based ammeter module). 
- Data logging for current and voltage, with customizable triggers to autolog both.

I have a long and detailed write-up of the circuitry design on my personal website: https://www.oihdesigns.com/arduino-multimeter-project

There are a couple videos here:
https://youtu.be/rzj-ar2Qv_A
and here:
https://youtu.be/hZ4eH-7RNsU

Here are more photos:

![20250623_205043_760_1DXII copy](https://github.com/user-attachments/assets/1e8d43a8-2d98-4d29-bcc4-4436717be3c0)
![20250623_202545_180_1DXII copy](https://github.com/user-attachments/assets/012b9ef7-557c-4ccc-9eac-9cd608fcdfd7)
![20250623_203203_470_1DXII copy](https://github.com/user-attachments/assets/2eb9bdcf-79eb-4e2a-9211-deb4775e8079)

## Design Prioities and Compromises:
This meter is designed to complement and support the more expensive equimpent generally found in a lab, rather than replace it. With that in mide, I've made the following design decisions on my priorites:

### Minimal Protections
- The voltmeter circuitry is not designed to measure amplitudes higher than is dangerous to humans. By not designing a tool that is supposed to keep people safe I can dramatically simplify and shrink the design. (The said, I've nailed the voltmeter circuit with a 1000V megaohmmeter. No problem)
- Likewise, the resistance circuit can be damaged if you accidentally measure too much /unregulated voltage. This is a tradeoff for a very fast and accurate autoranging ohmmeter circuit that can accurately read from less than 0.01 to more than 5,000,000 Ohms. 

### Streamlined Functions
- There is no built in ammeter, inductance, or capacitance function. Ammeter function is easily added as needed with either an off-the-shelf module, or a low value resistor across the voltmeter. There isn't an easy way to add indutance or capacitance. I don't use any of these often enough for them to be worth taking up the PCB space.  

## Current Support:
As of writing I have functional versions (I confirmed the core functions work) that support the:
- Arduino Uno R4
- Arduino Giga (with or without the display shield)
- Arduino ESP32 Nano (via jumpers)
- Seeed Studio XIAO RA4M1 (same processer as the Uno R4) 

![2025-06-30 15 28 38-1](https://github.com/user-attachments/assets/d1df3a64-35a9-4641-99ba-be2e2ba61623)
![2025-06-25 20 25 30](https://github.com/user-attachments/assets/2e6708fb-ebba-4c70-9a25-8d26cee135b0)

# Future Plans:

## Major Function Things to Address:
- Writing to MicroSD cards over the AUX SPI Bux on the RA4M1 XIAO boards. I set this up and tested it, but it didn't work and needs to be troubleshot.
- Getting the code to run on the XIAO RP2350 board. It compiles but does not run. 

## Potential Refinements To-Do List:
- Revise the PCB design to shrink the physical size (the current design was made with hand-soldering in mind). This would involve:
  - Smaller footprint components for things like the pullup resistors.
  - Dedicated multiple transistor packages for the switching functions rather than the discreet components.
  - There are probably other ways to shrink the design.
  - There is some potential performance improvement room to the analog circuit if I take advantage of the ADS1115's max input voltage of ~6.144vdc instead of 5vdc.
- There is always room for cleanup in the code to make it more elegant. 

## Ideas for Expanding on the Current Function / :
- Make a version that uses an ADS1256 instead of the ADS1115 to support measurements speeds up to 30,000 samples/second and resolutions of up to 24bits.
- Make a version that runs off a Raspberry Pi.
- Implement WiFi/Bluetooth features on the boards that support it.
- Implement DAC output as a simple function generator for the boards that support it.
- Add a jumper on the PCB and write the code to allow four-wire resistance readings.

If you're interested in helping develop this please feel free to reach out. 

# FAQ:
- Q: What is the logic voltage?
  - A:  The current PCB is 5V specific, except for a couple I've modified with level shifter modules on the SDA/SCL lines. The next version will have the level shifters integrated that allow use with either a 5V or 3.3V logic level board. 

- Q: Is there voltage isolation on the USB?
  - A: There is enough isolation (500K Ohms) that you will not hurt anything by measuring voltage (in either polarity) on something connected to the wall while the meter is also connected to the wall, but not enough to prevent ground loops from interfering with measurements. See the longer, full write up on my website, and then buy the Adafruit USB voltage isolation board.


- Q: The Adafruit isolation board says "100mA." What does the meter draw?
  - A: With the XIAO RA4M1, 50-60mA typical when measuring voltage. Closer to 100mA when measuring resistance. The resistance circuit goes into low-power mode several seconds after being open (except in a no-sleep mode).

- Q: Does it work on an Arduino Uno R3?
  - A: As a basic meter, yes, but there's no keyboard emulation.    

- Q: Can I buy one?
  - A: Eventually, yes, probably? I do not have a current timeline for bringing these to market. If you want one please reach out to me. If you want to hand-solder it yourself I can sell you a single PCB and maybe some of the components with higher individual costs.
 


