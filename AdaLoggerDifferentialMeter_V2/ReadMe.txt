"Adalogger" Based Datalogger Documentation:

Settings:

Connect Pin 4 to Ground to suppress the microSD card usage. Useful for streaming data in / troubleshooting. Can only be set during boot.

Connect Pin 25 to Ground to suppress the serial updates. Just if you don't want them.

Connect Pin 24 to Ground to turn CH2 off. Can only be set during boot. This increases the logging speed dramatically (When logging on CH1 only, you can run at a max ~0.5mS between samples, with both channels it falls to several mS between samples). 

The program makes two files on the sd card. "Log.csv" is the voltage every two seconds. "Capture.csv" is triggered when the reading fluctuates. 

The threshold for log triggers is adjusted via a potentiometer connected to A0. As of writing it is setup as an exponential function with a max value of ~8vdc. Generally about ~0.25vdc is good..

Open and edit the bottom of the Python script on the card to the files you want to view, and then run to script to plot them.

The display telling you the SD card is good/erroring is not fully proved out. 