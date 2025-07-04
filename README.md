![20250623_204616_470_1DXII copy](https://github.com/user-attachments/assets/606b6dc2-5ae6-4be6-a74b-a8d9c2276296)
# Micro-DMM

This is the PCB design and code for a 4.5 digit DMM/ instrumentation module. It is designed and written primairly for Arduino-style development boards.

I've developed this DMM in my work testing and troubleshooting electronics to make my life easier. 

![20250623_205912_480_1DXII copy](https://github.com/user-attachments/assets/18e67b72-3f34-445f-8bca-6235bc45dc89)

## The main features are:

- Using keyboard emulation to directly type values into a computer for me (most useful for toubleshooting populated PCBs).
- Very sensitive low-resistance readings (accurate to better than 0.01 Ohms with proper lead zeroing).
- LED blinks when I'm reading logic level voltage or higher.
- Simultaneous current and voltage measurement abilities.
- Data logging for current and voltage, with customizable triggers to autolog both.

I have a long and detailed write-up of the circuitry design on my personal website: https://www.oihdesigns.com/arduino-multimeter-project


![20250623_205043_760_1DXII copy](https://github.com/user-attachments/assets/1e8d43a8-2d98-4d29-bcc4-4436717be3c0)
![20250623_202545_180_1DXII copy](https://github.com/user-attachments/assets/012b9ef7-557c-4ccc-9eac-9cd608fcdfd7)
![20250623_203203_470_1DXII copy](https://github.com/user-attachments/assets/2eb9bdcf-79eb-4e2a-9211-deb4775e8079)

## Design Prioities and Compromises
I've made the following design decisions on how I 

## Current Support:

As of writing I have functional versions (I confirmed the core functions work) that support the:
- Arduino Uno R4
- Arduino Giga (with or without the display shield)
- Arduino ESP32 Nano (via jumpers)
- Seeed Studio XIAO RA4M1 (same processer as the Uno R4) 


## (Some of my) Future Plans:
- Make a version that uses an ADS1256 instead of the ADS1115 to support measurements speeds up to 30,000 samples/second and resolutions of up to 24bits.
- Make a version that runs off a Raspberry Pi.
- Implement WiFi/Bluetooth features on the boards that support it.
- Implement DAC output as a simple function generator for the boards that support it.

If you're interested in helping develop this please feel free to reach out. 
