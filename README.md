Most of AVR have predefined clock for SCL. In case of long range I2C connexion, the signal is not "perfect" at the end. So here is a software I2C library which permit to slow down the communication beetween controler & module.

Tested on :
Attiny 84
Attiny 85
Arduino Nano
