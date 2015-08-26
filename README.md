# sbgc-api-examples
SimpleBGC Serial API examples and libraries

This folders contains SBGC library and example sketches for Arduino, 
that shows how to control SimpleBGC-driven gimbal via Serial API.

API specs are available at the http://www.basecamelectronics.com/serialapi/


Contents:

	libraries/SBGC_lib - common library that can be used in any C/C++ project
	libraries/SBGC_Arduino - Arduino-specific library used in the examples


How to run examples:

1. Copy the content of this folder (preserving subfolder structure) to your "Sketchbook" folder, 
	 as specified in the "File -> Preferences -> Sketchbook location" in the Arduino IDE.
	 This step is important to let a compiler to find and include libraries.
2. Open any *.ino example in the Arduino IDE, compile and upload to your Arduino board.
3. Connect Arduino's serial interface to the UART connector on the SimpleBGC board:
    Arduino GND -> SimpleBGC GND
    Arduino TX -> SimpleBGC RX (optional)
    Arduino RX -> SimpleBGC TX
    You can power Arduino separatly or via +5V from onboard UART connector


All examples were tested with the Arduino 1.0 IDE, Pro Mini and UNO boards.