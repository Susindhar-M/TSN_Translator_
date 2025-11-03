Pin Description

Teensy 1 Teensy 2

10 (CS0)--------------10 (CS0) 11 (MOSI)-------------12 (MISO) 12 (MISO)-------------11 (MOSI) 13 (SCK)--------------13 (SCK)

Why MOSI and MISO are interchanged ?

In a microcontroller MOSI is always an output and MISO is always an input so when using microcontroller as slave we interchange connections to preserve the roles of the pin.
