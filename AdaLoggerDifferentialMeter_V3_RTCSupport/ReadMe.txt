"Adalogger" Based Datalogger Documentation:

Repository: 
https://github.com/oihdesigns/Micro-DMM/blob/main/AdaLoggerDifferentialMeter_V3_RTCSupport/ 


Settings:

DIP Switches:


Solo Switch:
1: ON/OFF. 
Note it's inverted from the switch label.


4x Switches:

1: Autorange

UP: Autorange Disable
DOWN: Autorange Enable

Autorange makes the board more accurate at values below about 5vdc. This is at the expense of logging speed (min reading interval ~2.2ms, instead of ~0.5ms).

Leave autorange off unless you need accuracy below ~5vdc)

2: CH2 Suppress 

UP: CH2 Disable
DOWN: CH2 Enable

Turn the second differential pair of analog inputs on. This increases the logging speed dramatically (When logging on CH1 only, you can run at a max ~0.5mS between samples, with both channels it falls to several mS between samples). 

Range is fixed if both channels are on.

Can only be set during boot.

3: Serial Updates:
 
UP: Serial Disable
DOWN: Serial Enable

Suppresses the serial updates. Just if you don't want them.


4: SD Usage

UP: SD Disable
DOWN: SD Enable

Suppress the microSD card usage. Useful for streaming data in / troubleshooting. 

Can only be set during boot.

Pin 9: Slow Log Pin

Connected to Ground: Normal Logging Interval (1s)
Floating / Connected High: Logs at 60s Interval 


Notes on Usage:

The range (as I write this) is roughly -90 to +90v.

There is a button that when pressed writes "mark" on that row of the CSV file.

There is a temp sensor. It's not super accurate, and is more for relative change rather than specific value. It's close to the board right now. Idea is to put it on a few feet of wire so it can be placed in a spot of interest.

There is a battery level reading now. Full is ~4.2v, low is 3.6v. 

The program makes a file on the SD card with the date time the board was turned on.

The threshold for log triggers is adjusted via a potentiometer connected to A0. As of writing it is setup as an exponential function with a max value of ~8vdc. Generally about ~0.25vdc is good..

Open and edit the bottom of the Python script on the card to the files you want to view, and then run to script to plot them. The script is very much in beta.

The display telling you the SD card is good/erroring is not fully proved out. 

The analog input is isolated from the board, so you should be able to run the board off car power and measure the car power with minimal/zero noise interference. Probably better to use a battery to keep everything on the same timebase. 

