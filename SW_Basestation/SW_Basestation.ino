/*
 * Firmware for running the basestation of the wirless controlled nurse call system
 * This Firmware was only tested with an Arduino Nano.
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
 */

#include "src/TSic.h" // library inside src folder
#include <Arduino.h>
#include <RF24.h> // library for nRF24L01 Transceiver
#include <avr/power.h> // library to en- or disable components like i2c or timer
#include <avr/sleep.h>
#include <math.h>

#define USE_SLEEP 1 // set to 1 for Arduino sleep function (if enabled, DEBUG and NRFDEBUG will not work properly)
#define DEBUG 0 // set to 1 to print debug messages (slows down execution)
#define NRFDEBUG 0 // set to 1 to print nrf specific debug messages (slows down execution)
#define NRFSENDINTERVAL 25 // defines send interval of nrf-module; multiple times of 4ms (100ms=25*4ms)

#define NUMOFBLINKS 3 // sets the number of flashes after successful initialisation
#define BLINKINTERVAL 500 // sets the flashing interval after successful initialisation (1=1ms)
#define ALARMBLINKINTERVAL 62 // defines alarm blink interval; multiple times of 4ms (248ms=62*4ms)
#define BATTERYCHKINTERVAL 300 // define battery check period; multiple times of 1s (1min = 60*1s)
#define NUM_SAMPLES 10 // defines number of ADC samples for battery voltage measurement
#define BATTERYTHRESHOLD 3.0 // defines the threshold for an empty battery
#define USEBUZZER 1 // set to 1 to enable Buzzer Audio output

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif
#if NRFDEBUG == 1
#define nrfdebug(x) Serial.print(x)
#define nrfdebugln(x) Serial.println(x)
#else
#define nrfdebug(x)
#define nrfdebugln(x)
#endif

// timer variables
volatile uint16_t nrf_timing = 0;
volatile uint16_t alarm_timing = 0;
volatile uint16_t battery_timing = 0;

// Pin for Reset button
const int ResetButton = 2;

// TSic506 Variables
const uint8_t TSic_Pin = 3; // Signal Pin of TSIC-Sensor
TSIC Sensorx(TSic_Pin);
volatile float TSic_temp = NAN; // Temperature in °C
volatile bool TSic_newData = 0; // flag to indicate new data
volatile uint8_t TSic_isrCount = 0; // TSic_isrCount for error check
float temp_val[2] = { NAN, NAN };
float dT_val[2] = { NAN, NAN };
float alarm_sensitivity = 0.04;

// indicator LEDs - active High
const int led_red = 17;
const int led_yellow = 18;
const int led_blue = 19;
const int led_green = 7; // correct to 7
// Buzzer pin
const int Buzzer = 8;

// Variables for Battery measurement
const int Bat_measure = A0; // Analog Pin A0
const int Bat_En = 15; // Digital Pin 15
float battery_voltage = 0;

// Error values
volatile bool nrf_error = 0;
volatile bool nrf_ack_error = 0;
volatile bool TSic_error = 0;
volatile bool Alarm_flag = 0;

// variables for nRF24 Transceiver
// creating a new instance for tranceiver called radio
RF24 radio(9, 10); // CE-Pin, CSN-Pin
const byte PairAddress[6] = "Nurse"; // define address for communicating
char payload[10] = "Sync Msg 0";
// debug variables
volatile uint32_t sleeptimebegin = 0;
volatile uint32_t previoustime = 0;
volatile uint32_t currenttime = 0;

void setup()
{
    // Initialisation of the GPIO-Pins
    pinMode(led_red, OUTPUT);
    pinMode(led_yellow, OUTPUT);
    pinMode(led_blue, OUTPUT);
    pinMode(led_green, OUTPUT);
    pinMode(Bat_En, OUTPUT);
    pinMode(Buzzer, OUTPUT);

    digitalWrite(Buzzer, LOW);
    digitalWrite(led_red, LOW);
    digitalWrite(led_yellow, LOW);
    digitalWrite(led_blue, LOW);
    digitalWrite(led_green, LOW);
    digitalWrite(Bat_En, LOW);

    // Initialisation of ResetButton Pin
    pinMode(ResetButton, INPUT_PULLUP);

#if (NRFDEBUG == 1) || (DEBUG == 1) // Opens the serial interface only if debug options are set.
    Serial.begin(115200);
#endif

    // clear pending interrupts before attachInterrupt()
    EIFR = (1 << INTF0); // clear interrupt 0 flag
    EIFR = (1 << INTF1); // clear interrupt 1 flag

    // begin initialisation of nrf transceiver
    if (!Nrf_Init()) {
        debugln("nrf init ok---------");
        radio.powerDown();
    } else {
        // Initialisation failed - hardware reset required
        // nrf-radio module needs hardware reset to operate properly
        // code stops working here
        nrf_error = 1;
        Error_handle();
    }

    // allow external interrupts on the defined pins
    attachInterrupt(digitalPinToInterrupt(ResetButton), ISR_reset_button, FALLING);
    attachInterrupt(digitalPinToInterrupt(TSic_Pin), TSic_isr, FALLING);

    // now wait for a correct sensor value
    // If the value is incorrect, switch on the green LED
    TSic_error = 1;
    while (1) {
        if (TSic_newData == 1) {
            TSic_newData = 0;
            TSic_error = Tsic_ErrorCheck(&TSic_temp);
            if (TSic_error == 0) {
                Error_handle();
                break;
            }
            Error_handle();
        }
        Error_handle();
    }
    // Init finished and all LEDs flash x times
    blink_all_leds(NUMOFBLINKS, BLINKINTERVAL);
    debugln("init finish"); // for testing purpose
    currenttime = 0;
    previoustime = 0;
    timer2_init(); // Starting timer 2
}

void loop()
{
    // code starts here after sleep

    // true if new Temperature data is available and if no alarm is set
    if (TSic_newData == 1 && Alarm_flag == 0) {
        TSic_newData = 0;
        TSic_error = Tsic_ErrorCheck(&TSic_temp); // Checks if Data is a valid number
        Error_handle();
        Alarm_flag = Alarmdetection(); // Check if alarm condition has been reached
        if (Alarm_flag == 1) // set the alarm immediately
        {
            digitalWrite(led_red, HIGH);
#if USEBUZZER == 1
            digitalWrite(Buzzer, HIGH);
#endif
            alarm_timing = 0;
        }
    }

    // Starts the transmission process to the handheld
    if (nrf_timing >= NRFSENDINTERVAL) // true every ___ms=NRFSENDINTERVAL * 4ms
    {
        nrf_timing = 0; // reset time counter
        previoustime = currenttime;
        currenttime = millis();
        nrfdebugln("_________________");
        nrfdebug(currenttime - sleeptimebegin);
        nrfdebugln("ms sleeped");
        nrfdebug(currenttime - previoustime);
        nrfdebugln("ms between 2 wakeups");

        // Checks if sensor is connected.
        // If TSIC-ISR has not been executed since the last transmission, the sensor is not connected.
        if (Alarm_flag == 0) {
            if (TSic_isrCount == 0) {
                TSic_error = 1; // sensor not connected!
            } else {
                TSic_isrCount = 0;
            }
        }
        Error_handle();
        // wakes up nrf-module from sleep
        // takes up to approx. 20ms for power up
        radio.powerUp();
        Nrf_Init();

        nrf_error = !radio.isChipConnected(); // checks if the module is connected to the SPI bus
        if (nrf_error == 1) {
            Error_handle();
        }
        // The payload is selected depending on the flag set.
        if (Alarm_flag == 1) {
            strncpy(payload, "ALARM", 10);
        } else {
            if (TSic_error == 1) {
                strncpy(payload, "TSIC_err", 10);
            } else {
                strncpy(payload, "SYNC", 10);
            }
        }
        nrfdebug("payload:");
        nrfdebugln(payload);
        // Sends the message and waits for an acknowledge from the handheld.
        // The Auto ACK feature of the NRF module is used.
        if (radio.write(&payload, sizeof(payload))) {
            nrfdebugln("ACK received");
            nrfdebugln("_________");
            nrf_ack_error = 0;
        } else {
            nrfdebugln("TX failed");
            nrfdebugln("_________");
            nrf_ack_error = 1;
        }
        // puts nrf-module into sleep
        radio.powerDown();
        Error_handle();
    }

    // Battery Voltage check
    if (battery_timing >= BATTERYCHKINTERVAL) // true every BATTERYCHKINTERVAL*1sec
    {
        battery_timing = 0; // reset time counter
        battery_voltage = Battery_measurement(Bat_En, Bat_measure, NUM_SAMPLES); // measures the voltage of the battery
        debug("Battery Voltage [V]:");
        debugln(battery_voltage);
        if (battery_voltage <= BATTERYTHRESHOLD) // True if the battery voltage is below the limit value
        {
            // turn the ledPin on
            digitalWrite(led_yellow, HIGH);
            debugln("Battery Low");
        } else {
            digitalWrite(led_yellow, LOW);
            debugln("Battery normal condition");
        }
    }

#if USE_SLEEP == 1
    EnterSleep(); // Put the Arduino in sleep mode to save energy
#endif
}

/*
Saves the new temperature value and checks if there is an alarm.
The function returns 1 if the alarm condition is fulfilled and 0 if not.
The alarm condition is fulfilled if the temperature rise dT of 2 values
is valid and their average value exceeds the set limit value and the temperature
trend goes in the same direction (positive or negative), then the alarm is triggered.
*/
bool Alarmdetection()
{
    // store old val in temp_val[0]
    // store new val in temp_val[1]
    debugln("______________________");
    debug("old temp_val[0]: ");
    debugln(temp_val[0]);
    temp_val[0] = temp_val[1];
    temp_val[1] = TSic_temp;
    debug("temp_val[0]: ");
    debugln(temp_val[0]);
    debug("temp_val[1]: ");
    debugln(temp_val[1]);
    // check if stored temperature values are valid numbers
    // if valid numbers store old dT_value in dT_val[0] and new value in dT_val[1]
    if (isnan(temp_val[0]) || isnan(temp_val[1])) {
        dT_val[1] = NAN;
    } else {
        dT_val[0] = dT_val[1];
        dT_val[1] = temp_val[1] - temp_val[0];
    }
    debug("dT_val[0]: ");
    debugln(dT_val[0]);
    debug("dT_val[1]: ");
    debugln(dT_val[0]);
    // do further processing only if dT_values are valid (not NAN)
    if (!isnan(dT_val[0]) || !isnan(dT_val[1])) {
        // check if absolute value of both dT_values is greater than the threshold value
        if ((abs(dT_val[0]) > alarm_sensitivity) && (abs(dT_val[1]) > alarm_sensitivity)) {

            // now check wether both values have a positive or negative tendency and only then set alarm
            if ((dT_val[0] > 0 && dT_val[1] > 0) || dT_val[0] < 0 && dT_val[1] < 0) {
                debugln("ALARM >>>> 0");
                debugln("______________________");
                return 1; // Set alarm
            }
        }
    }
    return 0;
}

// Checks if Temperaturdata is a valid number
bool Tsic_ErrorCheck(float* Tempdata)
{
    if (isnan(*Tempdata)) // isnan()returns 1 if value is NaN -> error reading sensor
    {
        return 1;
    } else {
        return 0;
    }
}

// puts the Arduino into the defined sleep mode
void EnterSleep(void)
{
    set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
    cli(); // deactivate interrupts
    sleep_enable(); // sets the SE (sleep enable) bit
    // sleep_bod_disable(); // disable Brown Out Detector (~60us faster resume to aktive )
    sei(); // enable interrupts
    sleep_cpu(); // sleep now!!
    sleep_disable(); // deletes the SE bit
}

// Switches certain LEDs on and off depending on flags
void Error_handle(void)
{
    // If the tsic sensor does not react or gives incorrect values, switch on the green LED.
    if (TSic_error == 1) {
        digitalWrite(led_green, HIGH);

    } else {
        digitalWrite(led_green, LOW);
    }
    // If no ack is received from the handheld within the specified time, turn on the blue LED.
    if (nrf_ack_error == 1) {
        digitalWrite(led_blue, HIGH);
    } else {
        digitalWrite(led_blue, LOW);
    }
    // If the nrf radio module no longer responds, switch on the red and blue leds and stop the program here.
    // restart required
    if (nrf_error == 1) {
        TIMSK2 = 0; // Disable timer2 interrupts
        TCCR2B = 0; // reset timer/counter control register B - no clock = timer stopped
        TCCR2A = 0; // reset timer/counter control register A
        nrfdebugln("Radio failed! - Please check wire connection and then reset");
        debugln("Radio failed! - Please check wire connection and then reset");
        digitalWrite(led_red, HIGH);
        digitalWrite(led_blue, HIGH);
        while (1)
            ;
    }
}

// Interrupt service routine to reset the Alarm
void ISR_reset_button()
{
    // Button resets the alarm flag to 0
    Alarm_flag = 0;
}

// Interrupt service routine to read data from TSIC-Sensor
void TSic_isr(void)
{
    volatile uint16_t TSic_raw = 0; // Variable for saving the 10-bit raw data from the TSic sensor
    volatile uint8_t ReadError = 0;
    /*start critical timing */
    ReadError = Sensorx.TSic_ReadData(&TSic_raw);
    /* end critical timing */

    // TSic_newData = 0;
    if (ReadError == 0) {
        TSic_temp = Sensorx.Tsic_TempConversion(&TSic_raw); // Reads raw data from the sensor
        //  debug("TSIC_raw dec: ");
        //  debug(TSic_raw);
        //  debug("TSIC_raw bin: ");
        //  Serial.print(TSic_raw,BIN);
        //  debug("| temp: ");
        //  debugln(TSic_temp);
    } else {
        TSic_raw = NAN;
        TSic_temp = NAN;
    }
    TSic_newData = 1; // Tsic_new -> new value available
    TSic_isrCount++; // increase isr counter (needed to detect if sensor is connected)
    if (TSic_isrCount == 10) {
        TSic_isrCount = 0;
    }
    EIFR = (1 << INTF1); // clear interrupt 1 flag
}

// function to initialise the nrf radio module
// returns 1 on error an 0 on success
bool Nrf_Init()
{
    if (!radio.begin()) // radio.begin() returns 0 on error
    {
        return 1;
    }
    // checks whether the radio module is an nRF24L01+ model or a non+ model and sets the data rate
    if (radio.isPVariant()) {
        radio.setDataRate(RF24_250KBPS);
    } else {
        radio.setDataRate(RF24_1MBPS);
    }
    // Set Power Amplifier level and Low Nois Amplifier state.
    // amplification of nrf-Module without LNA-chip (module without antenna)
    // RF24_PA_MIN = -18dBm
    // RF24_PA_LOW = -12dBm
    // RF24_PA_HIGH = -6dBm
    // RF24_PA_MAX = -0dBm
    // amplification of nrf-Module with LNA-chip (module with antenna)
    // LnaEnable = 1:
    // RF24_PA_MIN = -6dBm
    // RF24_PA_LOW = -0dBm
    // RF24_PA_HIGH = 3dBm
    // RF24_PA_MAX = 7dBm
    // LnaEnable = 0:
    // RF24_PA_MIN = -12dBm
    // RF24_PA_LOW = -4dBm
    // RF24_PA_HIGH = 1dBm
    // RF24_PA_MAX = 4dBm
    radio.setPALevel(RF24_PA_LOW, 0); // set PA-Level to Low with LNA disabled
    radio.openWritingPipe(PairAddress);
    radio.stopListening(); // set nrf into TX-Mode
    return 0;
}

// timer2 initialisation function
void timer2_init()
{
    // 4ms - 250Hz timer
    // timer frequency = (16000000/((249+1)*256))
    cli(); // deactivate interrupts
    // clear all timer registers
    TIMSK2 = 0; // Disable timer2 interrupts
    TCCR2B = 0; // reset timer/counter control register B - no clock = timer stopped
    TCCR2A = 0; // reset timer/counter control register A
    TCNT2 = 0; // Reset the timer2 set initial counter value
    TCCR2A |= (1 << WGM21); // set to CTC Mode TCCR2A = 0b00000010
    TCCR2B |= (1 << CS22) | (1 << CS21); // set prescaler to 256
    OCR2A = 249; // Compare match every 4 ms
    TIMSK2 |= (1 << OCIE2A); // Enable INT on compare match A; TIMSK2 = 0b00000010;
    sei(); // Enable the global interrupt
}

// Timer2 compare match A Interrupt Service routine
ISR(TIMER2_COMPA_vect) // every 4ms
{
    static uint8_t ms_tick = 0;
    nrf_timing++;
    alarm_timing++;
    ms_tick++;
    if (ms_tick >= 250) // true if 1second is reached; multiple times of 4ms (1s=250*4ms)
    {
        ms_tick = 0;
        battery_timing++;
    }
    // Handle Alarm
    if (Alarm_flag == 1) // if alarm is triggered blink red LED and Buzzer
    {
        if (alarm_timing >= ALARMBLINKINTERVAL) // true every ___ms = ALARMBLINKINTERVAL * 4ms
        {
            alarm_timing = 0;
            digitalWrite(led_red, !digitalRead(led_red));
#if USEBUZZER == 1
            digitalWrite(Buzzer, !digitalRead(Buzzer));
#endif
        }
    } else {
        alarm_timing = 0;
        digitalWrite(led_red, LOW);
        digitalWrite(Buzzer, LOW);
    }
}

// Function to make all LEDs flash x times
void blink_all_leds(uint8_t numofblinks, uint16_t msintervall)
{
    uint32_t currentMillis = 0;
    uint32_t previousMillis = 0;
    uint8_t blinkcount = 0;

    while (1) {
        currentMillis = millis();
        if (currentMillis - previousMillis >= msintervall) {
            previousMillis = currentMillis;
            digitalWrite(led_red, !digitalRead(led_red));
            digitalWrite(led_yellow, !digitalRead(led_yellow));
            digitalWrite(led_blue, !digitalRead(led_blue));
            digitalWrite(led_green, !digitalRead(led_green));
            if (digitalRead(led_red) == LOW) {
                blinkcount++;
                debug("blinkcount:"); // for testing purpose
                debugln(blinkcount); // for testing purpose
            }
        }
        if (blinkcount >= numofblinks) {
            digitalWrite(led_red, LOW);
            digitalWrite(led_yellow, LOW);
            digitalWrite(led_blue, LOW);
            digitalWrite(led_green, LOW);
            break;
        }
    }
}

// Function to measure the battery voltage
float Battery_measurement(int enable_pin, int measurement_pin, int samples)
{
    float voltage = 0;
    int16_t averageValue = 0;
    int16_t R1 = 100; // 100kOhm Resistor
    int16_t R2 = 100; // 100kOhm Resistor
    digitalWrite(enable_pin, HIGH); // activate battery measurement
    for (int i = 0; i < samples; ++i) // read samples from the sensorpin
    {
        averageValue += analogRead(measurement_pin);
    }
    averageValue = averageValue / samples; // calculate average
    digitalWrite(enable_pin, LOW);
    // 10bit adc resolution of 4,9mV
    voltage = averageValue * (5.0 / 1023) * ((R1 + R2) / R2);
    return voltage;
}