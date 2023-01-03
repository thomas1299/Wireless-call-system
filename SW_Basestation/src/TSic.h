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

#ifndef TSIC_h
#define TSIC_h

#include "Arduino.h"

#define TSICTIMEOUT 10000 //

class TSIC {
public:
    explicit TSIC(uint8_t data_pin);
    float Tsic_TempConversion(uint16_t *temp_data);
    uint8_t TSic_ReadData(uint16_t *temp_data);

private:
    uint8_t m_data_pin;
    uint8_t TSic_ReadByte(uint16_t *temp_data, uint8_t bytenumber);
    uint8_t TSic_ParityCheck(uint16_t *temp_data);
};

#endif /* TSIC_H_ */