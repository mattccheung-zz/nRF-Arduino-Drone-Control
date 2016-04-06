## First version of arduino firmware for futureBOX project.

this version works with deviationTX protocols (which require some minor modifications)


### how to use:
- open protocol.h and comment out protocols you dont want to use and uncomment the ones you want (limit on arduino uno is arround 10 protocols)
- check the protocols you uncommented if they require some modifications (see below) [most of them were already fixed ... but who knows]
- open arduino.ino in arduino IDE
- upload to your board

### modifications to devTX protocols:
- all subfolder includes must be commented out (like #include <config/tx.h> )
- all printf statements must be commented out 

you can connect to serial interface (1MB speed), 5 commands are supported. command structure is as follows:

0x35 0x7a -- header
0xXX -- command
10 bytes command payload
1 byte CRC

### commands:
* 0x0A *: get list of supported models.
all payload bytes should be zero

it will output one (enabled) protocol per line

* 0x0B *: set active protocol
first payload byte sets the active protocol number (as returned from 0x0A command)
other payload bytes should be 0

0x0C: set model
sets the model properties

0x00: BIND
binds to the selected protocol, all payload bytes should be 0

0x01: DATA
sends the data (joystick, buttons)

* byte0 - throttle
* byte1 - aileron
* byte2 - rudder
* byte3 - elevator
* byte4 - not used
* byte5 - not used
* byte6 - flags byte 3 (not used)
* byte7 - flags byte 2 (not used)
* byte8 - flags byte 1 (CHANNEL 5 - CHANNEL 12)