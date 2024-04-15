#include "Arduino.h"

#include "../tinyI2C.h"


void setup() {

}

void loop() {
    TINYI2C Wire = TINYI2C(18, 19, 1);

    while (1) {
    Wire.beginTransmission(0x44);
    Wire.write(0xFD);
    Wire.endTransmission();
    
    delay(20);

    Wire.readFrom(0x44, 6);

    delay(500);
    }
  };





}
