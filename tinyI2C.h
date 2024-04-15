#include "Arduino.h"

#ifndef TINYI2C_H
#define TINYI2C_H

class TINYI2C {

    public:
        uint8_t SCL_bit;
        uint8_t SCL_port;
        volatile uint8_t *SCL_reg;
        volatile uint8_t *SCL_out;

        uint8_t SDA_bit;
        uint8_t SDA_port;
        volatile uint8_t *SDA_reg;
        volatile uint8_t *SDA_out;
        volatile uint8_t *SDA_in;

        uint8_t read_buffer[10] = {};

    TINYI2C(uint8_t _SDA_pin, uint8_t _SCL_pin) {

        SCL_bit = digitalPinToBitMask(_SCL_pin);
        SCL_port = digitalPinToPort(_SCL_pin);
        SCL_reg = portModeRegister(SCL_port);
        SCL_out = portOutputRegister(SCL_port);

        SDA_bit = digitalPinToBitMask(_SDA_pin);
        SDA_port = digitalPinToPort(_SDA_pin);
        SDA_reg = portModeRegister(SDA_port);
        SDA_out = portOutputRegister(SDA_port);
        SDA_in = portInputRegister(SDA_port);

        cli();
        //digitalWrite(TINYI2C::SDA, HIGH);
        *SDA_out |= SDA_bit;
        //pinMode(TINYI2C::SDA, OUTPUT);
        *SDA_reg |= SDA_bit;
        wait();
        //digitalWrite(TINYI2C::SCL, HIGH);
        *SCL_out |= SCL_bit;
        //pinMode(TINYI2C::SCL, OUTPUT);
        *SCL_reg |= SCL_bit;
        wait();
        sei();
        
        //on tente de reset le bus pour eviter les glitchs
        beginTransmission(0);
        endTransmission();
        cli();
        //digitalWrite(TINYI2C::SDA, HIGH);
        *SDA_out |= SDA_bit;
        //pinMode(TINYI2C::SDA, OUTPUT);
        *SDA_reg |= SDA_bit;
        wait();
        //digitalWrite(SCL, HIGH);
        *SCL_out |= SCL_bit;
        //pinMode(SCL, OUTPUT);
        *SCL_reg |= SCL_bit;
        wait();
        sei();
    };

    bool beginTransmission(uint8_t address, bool read_write = 0)
    {//read_write = 0 write 1 read
        cli();
        //digitalWrite(SDA, LOW);
        *SDA_out &= ~SDA_bit;
        //pinMode(SDA, OUTPUT);
        *SDA_reg |= SDA_bit;
        wait();
        //digitalWrite(SCL, LOW);
        *SCL_out &= ~SCL_bit;
        //pinMode(SCL, OUTPUT);
        *SCL_reg |= SCL_bit;
        wait();
        sei();

        if(!write(address << 1 | read_write)) {
        //1 = read 0 = write
            Serial.println("Erreur d'ACK (Begin Transmission)");
            //digitalWrite(SCL, HIGH);
            *SCL_out |= SCL_bit;

            return false;
        }

        //digitalWrite(SCL, HIGH);
        //*SCL_out |= SCL_bit;

        return true;
    };

    bool write(uint8_t data) {
        uint8_t mask = 0b10000000;
        cli();
        
        //pinMode(SDA, OUTPUT);
        *SDA_reg |= SDA_bit;
        
        while (mask) {
            if (data & mask)
                //digitalWrite(SDA, HIGH);
                *SDA_out |= SDA_bit;
            else
                //digitalWrite(SDA, LOW);
                *SDA_out &= ~SDA_bit;
            wait();
            //on fait un tic d'horloge
            //digitalWrite(SCL, HIGH);
            *SCL_out |= SCL_bit;
            wait();
            //digitalWrite(SCL, LOW);
            *SCL_out &= ~SCL_bit;
            wait();

            mask = mask >> 1;
        }

        if (read_ACK()) {
            sei();
            return false;
        }
        sei();
        return true;
    };

    bool read_ACK() {
        //on récupère l'ACK
        cli();
        //pinMode(SDA, INPUT);
        *SDA_reg &= ~SDA_bit;
        *SDA_out &= ~SDA_bit;
        //digitalWrite(SCL, HIGH);
        *SCL_out |= SCL_bit;
        wait();
        //bool ACK = digitalRead(SDA);
        bool ACK = *SDA_in & SDA_bit;
        //digitalWrite(SCL, LOW);
        *SCL_out &= ~SCL_bit;
        wait();
        //digitalWrite(SDA, ACK);
//        pinMode(SDA, OUTPUT);
//        wait();
        sei();
        return ACK;
    };

    void readFrom(uint8_t address, uint8_t nbByteRead) {

        beginTransmission(address, true);

        cli();
        //pinMode(SDA, INPUT);
        *SDA_reg &= ~SDA_bit;
        *SDA_out &= ~SDA_bit;

         for (uint8_t a = 0; a != nbByteRead; a++)
        {
            uint8_t read_byte = 0;
            for (uint8_t a = 8; a != 0; a--) {
                //digitalWrite(SCL, HIGH);
                *SCL_out |= SCL_bit;
                wait();
                //if (digitalRead(SDA))
                if (*SDA_in & SDA_bit)
                    bitSet(read_byte, a-1);
                //digitalWrite(SCL, LOW);
                *SCL_out &= ~SCL_bit;
                wait();
            }

            read_buffer[a] = read_byte;

            //on écrit un ACK
            //digitalWrite(SDA, LOW);
            *SDA_out &= ~SDA_bit;
            //pinMode(SDA, OUTPUT);
            *SDA_reg |= SDA_bit;
            wait();

            //digitalWrite(SCL, HIGH);
            *SCL_out |= SCL_bit;
            wait();
            //digitalWrite(SCL, LOW);
            *SCL_out &= ~SCL_bit;
            wait();

            //pinMode(SDA, INPUT);
            *SDA_reg &= ~SDA_bit;
            *SDA_out &= ~SDA_bit;
        }
        sei();
        endTransmission();
    };


    void endTransmission() {
        cli();
        //digitalWrite(SCL, LOW);
        *SCL_out &= ~SCL_bit;
        //pinMode(SCL, OUTPUT);
        *SCL_reg |= SCL_bit;

        wait();
        //digitalWrite(SDA, LOW);
        *SDA_out &= ~SDA_bit;
        //pinMode(SDA, OUTPUT);
        *SDA_reg |= SDA_bit;
        wait();
      
        //digitalWrite(SCL, HIGH);
        *SCL_out |= SCL_bit;
        wait();
//        digitalWrite(SDA, HIGH);
        *SDA_out |= SDA_bit;
        wait();
        sei();
    };

    void wait(uint8_t timer = 0) {
        for (uint8_t a = 0; a < timer; a++) {
            asm volatile("nop");
        }
    }

};


#endif