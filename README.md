# photon_feeder_independent
rs485 transceiver direct communication with photon feeder

# build
```
g++ main.cpp PhotonVirtual.cpp -o photon_virtual
```

# run
connect your usb2rs485 transceiver and find its serial port name. in the example here it's /dev/ttyUSB0
load tty0tty kernel module, then:
```
./photon_virtual /dev/tnt1 /dev/ttyUSB0
```
# openpnp
add a new gcode driver. change baudrate to 2000000. close openpnp to save your machine settings.
the /dev/tnt0 interface needs to be configured manually in your machine.xml file, because it does not show up in openpnp gui.
search for the new gcode driver and change serial device manually to /dev/tnt0.
run openpnp again and add gcode commands the new driver:
COMMAND_CONFIRM_REGEX: 
```
^ok.*
```
ACTUATOR_READ_COMMAND: 
```
M485 {Value}
```
ACTUATOR_READ_REGEX: 
```
rs485-reply: (?<Value>.*)
```

connect to your machine. 
the automatic photon feeder detection should work now. enjoy!

