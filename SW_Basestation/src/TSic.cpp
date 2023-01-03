/*
 * Library for reading the digital temperature sensor TSIC506 based on the ZACwire communication protocol.
 * The temperature values are read out via an interrupt service routine in the main programme.
 *
 * This library is based on version 2.3 by Roman Schmitz from 2016.11.01, which can be found at
 * https://github.com/Schm1tz1/arduino-tsic.
 *
 * This library was only tested with the TSIC506 sensor.
 * This library was only tested with an Arduino Nano.
 * With this library, the Tsic sensor is permanently supplied with 5 volts, the supply pin is not switched on and off.
 * Due to the permanent power supply, the sensor will provide new temperature values at regular intervals.
 * According to the data sheet, a new value is output every 10Hz (100ms).
 * Application Note for the TSIC-Sensor: https://docs.rs-online.com/34a5/0900766b81690bba.pdf
 *
 * Version 1.0 (by Thomas Landauer, 2022.11.09)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *    https://playground.arduino.cc/Code/Tsic/
 */

#include "TSic.h"
#include "Arduino.h"

// Initialisation of the inputs
TSIC::TSIC(uint8_t data_pin)
{
    m_data_pin = data_pin;
    pinMode(
        m_data_pin, INPUT_PULLUP); // pullup is required to have a defined HIGH state. TSIC signal is pulled to ground.
}

/* Read binary data from sensor and check parity
 Returns 0 on success; Returns 1 on failure */
uint8_t TSIC::TSic_ReadData(uint16_t* temp_data)
{
    uint16_t Byte1 = 0;
    uint16_t Byte2 = 0;
    // Reading 2 bytes from the sensor using the Zacwire protocol
    /*** start critical timing ***/
    if (TSic_ReadByte(&Byte1, 0)) {
        return 1;
    }
    if (TSic_ReadByte(&Byte2, 1)) {
        return 1;
    }
    /*** End of critical timing ***/

    // Serial.print("Tsic_Byte1:");
    // Serial.println(TSic_byte1, BIN);
    // Serial.print("Tsic_Byte2:");
    // Serial.println(TSic_byte2,BIN);

    if (TSic_ParityCheck(&Byte1)) {
        // Serial.println("Error Parity check byte 1");
        Byte1 = 0;
        Byte2 = 0;
        return 1;
    }
    if (TSic_ParityCheck(&Byte2)) {
        // Serial.println("Error Parity check byte 2");
        Byte1 = 0;
        Byte2 = 0;
        return 1;
    }

    *temp_data = (Byte1 << 8) + Byte2;

    Byte1 = 0;
    Byte2 = 0;
    // check for boundaries
    if (*temp_data == 0)
        return 1;
    if (*temp_data == 2047) // 2047 means +60°C -> High boundary HT=60
        return 1;
    return 0;
}

// Read Data Packet (10 Bits) from TSic Sensor
// 1 Data Packet contains: 1 Start-bit, 8 Data-bits, 1 Parity-bit
// The data is sent with MSB first
// The bit format is duty cycle encoded
// Returns 1 on error
uint8_t TSIC::TSic_ReadByte(uint16_t* temp_data, uint8_t bytenum)
{
    uint16_t strobetime = 0;
    uint16_t bitstrobetime = 0;
    uint16_t timeout = 0;

    if (bytenum == 1) {
        while (digitalRead(m_data_pin)) {
            timeout++;
            delayMicroseconds(10);
            if (timeout > TSICTIMEOUT) {
                return 1;
            }
        }
    }

    // Measure strobetime
    // strobetime usually ~62,5us (8kHz baud rate - 125us bit window)
    // i.e. 6 cycles @ 10us cycles
    // If the sensor is not connected, the timeout is triggered and the function is terminated. (after 100 cycles)
    timeout = 9900;
    while (!digitalRead(m_data_pin)) { // waiting for rising edge
        strobetime++;
        timeout++;
        delayMicroseconds(10);
        if (timeout > TSICTIMEOUT) {
            return 1;
        }
    }

    // read 8-data bits and 1 parity bit
    for (uint8_t i = 0; i < 9; i++) {
        timeout = 0;
        // wait for next bit to start (falling edge)
        while (digitalRead(m_data_pin)) {
            timeout++;
            if (timeout > TSICTIMEOUT) {
                return 1;
            }
        }
        timeout = 0;
        bitstrobetime = strobetime;

        // now get bit bitstrobetime
        while (bitstrobetime--) {
            timeout++;
            delayMicroseconds(10);
            if (timeout > TSICTIMEOUT) {
                return 1;
            }
        }
        *temp_data <<= 1;
        // if pin is HIGH set bit to 1
        if (digitalRead(m_data_pin)) {
            *temp_data |= 1;
        }
        timeout = 0;

        // wait for next rising edge (end of bit)
        while (!digitalRead(m_data_pin)) {
            timeout++;
            if (timeout > TSICTIMEOUT) {
                return 1;
            }
        }
    }
    return 0;
}

// check the parity bit (even parity)
uint8_t TSIC::TSic_ParityCheck(uint16_t* temp_data)
{
    uint8_t parity = 0;

    for (uint8_t i = 0; i < 9; i++) {
        if (*temp_data & (1 << i))
            parity++;
    }
    if (parity % 2)
        return 1; // parity error occured
    *temp_data >>= 1; // delete parity bit
    return 0; // no parity error
}

// convert temperature from uint to float with 1 decimal place
float TSIC::Tsic_TempConversion(uint16_t* temp_data)
{
    float celsius = 0;
    // int16_t tempbuf = 0;
    //// speed-optimised calculation at the cost of slightly poorer resolution:
    //// °C = (temp_value/2047*(HT-LT)+LT)
    //// TSIC506: HT= +60 °C, LT= -10 °C
    // tempbuf = ((*temp_data * 175L) >> 9) - 99;
    // celsius = tempbuf / 10 + (float)(tempbuf % 10) / 10; // shift comma by 1 digit e.g. 26,4°C

    // calculation according datasheet
    // slower because of floating point operation but more precise than integer calculation
    celsius = (*temp_data / 2047.0) * 70 - 10;
    return celsius;
}
