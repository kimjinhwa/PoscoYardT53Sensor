/*-------------------------------------------------------------------------
RTC library

Written by Michael C. Miller.

I invest time and resources providing this open source code,
please support me by dontating (see https://github.com/Makuna/Rtc)

-------------------------------------------------------------------------
This file is part of the Makuna/Rtc library.

Rtc is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

Rtc is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with Rtc.  If not, see
<http://www.gnu.org/licenses/>.
-------------------------------------------------------------------------*/

#include <Arduino.h>

#pragma once

//ThreeWire command Read/Write flag 
const uint8_t THREEWIRE_READFLAG = 0x01;

class ThreeWire 
{
public:
    ThreeWire(uint8_t ioPin, uint8_t clkPin, uint8_t cePin) :
        _ioPin(ioPin),
        _clkPin(clkPin),
        _cePin(cePin)
    {
        pinMode(_cePin, OUTPUT);
        pinMode(_clkPin, OUTPUT);
        pinMode(_ioPin, OUTPUT);
        digitalWrite(_cePin, LOW); // default, not enabled
        digitalWrite(_clkPin, LOW); // default, clock low
    }

    void begin() {
        pinMode(_cePin, OUTPUT);
        pinMode(_clkPin, OUTPUT);
        //pinMode(_ioPin, INPUT);
        digitalWrite(_cePin, LOW); // default, not enabled
        digitalWrite(_clkPin, LOW); // default, clock low
        delay(1);
    }

    void end() {
    }

    void beginTransmission()
    {
        pinMode(_clkPin, OUTPUT);
        digitalWrite(_clkPin, LOW);

        digitalWrite(_cePin, LOW); // Enable device
        delay(1);
        digitalWrite(_cePin, HIGH); // Enable device
        delayMicroseconds(60);           // at least 3us
        digitalWrite(_cePin, LOW);  // Enable device
        delayMicroseconds(3);            // at least 14ns
    }

    void endTransmission() {
        digitalWrite(_clkPin, LOW);
        digitalWrite(_cePin, HIGH); // Enable device
    }

    uint16_t read16() {
        uint16_t lo = 0;

        for (uint8_t bit = 0; bit < 16; bit++) {
            digitalWrite(_clkPin, HIGH);
            delayMicroseconds(1);
            lo |= (digitalRead(_ioPin) << (15-bit));
            digitalWrite(_clkPin, LOW);
            delayMicroseconds(1);
        }
        return lo;
    }
    uint8_t read() {
        uint8_t value = 0;

        for (uint8_t bit = 0; bit < 8; bit++) {
            // first bit is present on io pin, so only clock the other
            // bits
        
            // Clock up, prepare for next
            digitalWrite(_clkPin, HIGH);
            delayMicroseconds(1);

            // Clock down, value is ready after some time.
            digitalWrite(_clkPin, LOW);
            value |= (digitalRead(_ioPin) << bit);
            delayMicroseconds(1);        // tCL=1000ns, tCDD=800ns
        }

        return value;
    }

private:
    const uint8_t _ioPin;
    const uint8_t _clkPin;
    const uint8_t _cePin;

    void resetPins() {
        // just making sure they are in a default low power use state
        // as required state is set when transmissions are started
        // three wire devices have internal pull downs so they will be low
        // pinMode(_clkPin, INPUT);
        // pinMode(_ioPin, INPUT);
        // pinMode(_cePin, INPUT);
    }
};

    // void write(uint8_t value, bool isDataRequestCommand = false) {
    //     for (uint8_t bit = 0; bit < 8; bit++) {
    //         digitalWrite(_ioPin, value & 0x01);
    //         delayMicroseconds(1);     // tDC = 200ns

    //         // clock up, data is read by DS1302
    //         digitalWrite(_clkPin, HIGH);
    //         delayMicroseconds(1);     // tCH = 1000ns, tCDH = 800ns

    //         // for the last bit before a read
    //         // Set IO line for input before the clock down
    //         if (bit == 7 && isDataRequestCommand) {
    //             pinMode(_ioPin, INPUT);
    //         }

    //         digitalWrite(_clkPin, LOW);
    //         delayMicroseconds(1);     // tCL=1000ns, tCDD=800ns

    //         value >>= 1;
    //     }
    // }
