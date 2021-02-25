# weatherstation

(This is a work in proress as of Feb 2021)

This project is my code and notes for creating an Arduino-powered
weather station that monitors air temperature, water temperature, and
humidity, and communicates wirelessly with a Raspberry Pi, which is
responsible for collecting the data and publishing it to a TBD
location (probably Twitter).

Goals of this project:

- Wireless monitoring of water temperature at my lake house
- Do something fun during pandemic
- Hands-on experience with microcontrollers
- Brush up on C++

## Software

The `arduino` folder contains the sketch.  I tried to keep it as
non-blocking as possible.  The sensor reads are blocking, as are the
radio transactions, but other than that, the main loop executes in a
few ms each time.

The `rpi` folder contains the Pi side of things.

## Hardware

### Functionality

The Arduino side of things is a weatherproof enclosure with the board,
sensor connectors, the LCD, and a button.   The LCD is off by
default -- pressing the button turns it on and pressing it again
cycles through 4 screens of info.  The LCD turns off 5 seconds after
the lass button press.  3 LEDs indicate sensor activity and RF
activity.  A piezo buzzer can be triggered remotely to test
connectivity over longer ranges.

Communication consists of 4 byte payloads:  1 byte for a command, and
3 bytes of data (args or return values).   One advantage of Fahrenheit
is that temperatures fit within an unsigned char (the sensors don't
work below 0F), and by definition relative humidity can't be less than 0.


### Power

The Pi will be inside the house and plugged into an AC adapter.   The
Arduino is harder.   Currently deciding betewen solar+battery,
hardwired, or battery with periodic AC charging.   Power consumption
is about 66-70mA just collecting readings and periodic radio
transmissions.  The LCD backlight adds about 12mA, and the buzzer
another 10, for about 100mA peak.

### Arduino

- Arduino Uno R3
- Assorted LEDs and resistors
- DHT11 temperature sensor (mine came on a breakout board with the
  relevant resistors)
- DS18B20 temperature sensor
- LCD 1602 display with an I2C backpack
- nRF24L01 transciever module

### Raspberry Pi

- Raspberry Pi Model B (older 26-pin GPIO header)
- LCD 1602 display without an I2C backpack
- nRF24L01 transciever module

### Hardware Source Notes

- The Arduino came as part of a kit: "Inland Basic Starter Kit V2.0 for
  Arduino, part 052035" from Microcenter via Amazon.  Amazon link is
  now dead, but it's your basic starter kit.  The kit included the
  1602 display (without the backpack), the DHT11, LEDs, buttons,
  buzzers, etc.   It was $23.99, possibly on clearance.  If I was
  starting over, I'd probably get a DHT22.

- The LCD with the I2C backpack was amzn.com/B08CXZ5VWL ($10.59 for
  2).  One had some bad solder joints, but was easily fixed.  The trim
  pot on the back is for the LCD contrast, and the jumper is so you
  can add a hardware switch for the backlight.

- The RF transceivers were amzn.com/B082397NTR ($9.99 for a pair).  I
  chose the ones with SMA connectors and antennas for maximum
  flexibility.

- The DS18B20 was amzn.com/B087JQ6MCP ($9.49), but there are a ton of
  them in this form factor.  They all come with short cables, and I
  wanted at least 20 feet, so I ordered 25 feet of UL2464 cable from
  Amazon.  Got 22AWG, 24 would have been fine.  Removed the
  heatshrink, re-soldered the sensor to the new able, added new heatshrink.

### Library Notes

Unfortunately, the Arduino library ecosystem appears polluted with
abandonware, duplicates, etc.  Adafruit and Sparkfun have decent
libraries, but not for all types of hardware.

It took me longer than I'd like to find an I2C library for the
display, and I settled on
https://github.com/johnrickman/LiquidCrystal_I2C, which claims to be
an archived copy of something at GitLab, which is now 404.  Bummer.
But I forked it, so at least I'll have it for the future.

The RF24 library is https://github.com/nRF24/RF24, which fortunately
supports both Linux on ARM and Arduino, so I can use the same library
on both ends (the Linux one even has Python bindings).

The DS18B20 is https://github.com/RobTillaart/DS18B20_RT, a stripped
down version that only does what I need.  It uses the OneWire library.

The DHT11 library is https://github.com/RobTillaart/DHTlib.

### Arduino Code

- I used the Arduino IDE and the libraries above to write this, and
  the excellent examples with most libraries.   There's probably
  better ways to do some of this, and there's definitely more memory
  efficient ways.   But I prefer working with objects and verbosity of
  code.   The highly procedural synchronous nature of a lot Arduino
  starter code wasn't what I was going for.



