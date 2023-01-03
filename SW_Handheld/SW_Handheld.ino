/*
 * Firmware for running the handheld of the wirless controlled nurse call system
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

#include <Arduino.h>
#include <RF24.h> // library for nRF24L01 Transceiver
#include <avr/power.h> // library to en- or disable components like i2c or timer
#include <avr/sleep.h>
#include <math.h>

#define USE_SLEEP 1 // set to 1 for Arduino sleep function (if enabled, DEBUG and NRFDEBUG will not work properly)
#define DEBUG 0 // set to 1 to print debug messages (slows down execution)
#define NRFDEBUG 0 // set to 1 to print nrf specific debug messages (slows down execution)
#define NRFRECVINTERVAL 19 // defines sleep time; multiple times of 4ms (76ms=19*4ms)
// note that NRFRECVINTERVAL should not be higher than 19 (76ms) because nrf-wakeup needs approximately 20ms
#define TIMEOUTINTERVAL 300 // defines receive timeout in ms

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

// indicator LEDs - active High
const int led_red = 17;
const int led_yellow = 18;
const int led_blue = 19;
const int led_green = 7;
// Buzzer pin
const int Buzzer = 8;

// Variables for Battery measurement
const int Bat_measure = A0; // Analog Pin A0
const int Bat_En = 15; // Digital Pin 15
float battery_voltage = 0;

// Error values
volatile bool nrf_error = 0;
volatile bool Alarm_flag = 0;
volatile bool TSic_error = 0;

// variables for nRF24 Transceiver
// creating a new instance for tranceiver called radio
RF24 radio(9, 10); // CE-Pin, CSN-Pin
const byte PairAddress[6] = "Nurse"; // define address for communicating
char dataReceived[10];
bool outofsyncflag = false;
bool timeoutflag = false;
// debug variables
volatile uint32_t sleeptimebegin = 0;
volatile uint32_t previoustime = 0;
volatile uint32_t currenttime = 0;
volatile uint32_t msgreceived = 0;
volatile uint32_t lastreceived = 0;
volatile uint32_t startWaiting = 0;

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

#if (NRFDEBUG == 1) || (DEBUG == 1) // Opens the serial interface only if debug options are set.
    Serial.begin(115200);
#endif
    // clear pending interrupts before attachInterrupt()
    EIFR = (1 << INTF0); // clear interrupt 0 flag
    EIFR = (1 << INTF1); // clear interrupt 1 flag

    // begin initialisation of nrf transceiver
    if (!Nrf_RX_Init()) {
        nrfdebugln("nrf init ok---------");
        radio.powerDown();
    } else {
        // Initialisation failed - hardware reset required
        // nrf-radio module needs hardware reset to operate properly
        // code stops working here
        nrf_error = 1;
        Error_handle();
    }

    // Init finished and all LEDs flash x times
    blink_all_leds(NUMOFBLINKS, BLINKINTERVAL);
    debugln("init finish"); // for testing purpose
    // attachInterrupt(digitalPinToInterrupt(ResetButton), ISR_reset_button, FALLING);
    currenttime = 0;
    previoustime = 0;
    timer2_init(); // Starting timer 2
}

void loop()
{
    // code starts here after sleep

    // Starts RX mode to receive messages from the base station
    if (nrf_timing >= NRFRECVINTERVAL) // true every ___ms=NRFRECVINTERVAL * 4ms
    {
        timeoutflag = false; // reset timeoutflag
        previoustime = currenttime; // timestamp last wakeup
        currenttime = millis(); // wakeup time
        nrfdebugln("_________________");
        nrfdebug(currenttime - sleeptimebegin);
        nrfdebugln("ms sleeped");
        nrfdebug(currenttime - previoustime);
        nrfdebugln("ms between 2 wakeups");

        // wakes up nrf-module from sleep
        // takes up to approx. 20ms for power up
        radio.powerUp();
        Nrf_RX_Init();

        nrf_error = !radio.isChipConnected(); // checks if the module is connected to the SPI bus
        if (nrf_error == 1) {
            Error_handle();
        }
        startWaiting = millis();
        // Now wait for a message from the base station.
        // radio.available() retuned 1 if message was received
        while (!radio.available() && !timeoutflag) {
            // If no message was received within the reception window of 32ms set the outofsyncflag
            if (((millis() - startWaiting) > 32) && (outofsyncflag == false)) {
                outofsyncflag = true;
                Error_handle();
                nrfdebugln("Out of sync");
            }
            // If no message is received within TIMEOUTINTERVAL set the timeout flag and stop waiting for a message.
            if ((millis() - startWaiting) > TIMEOUTINTERVAL) {
                timeoutflag = true;
                nrf_timing = 0;
            }
        }
        if (timeoutflag == true) {
            nrfdebugln("Failed receiving, timeout.");
        } else // message received
        {
            radio.read(&dataReceived, sizeof(dataReceived));
            outofsyncflag = 0;
            timeoutflag = 0;

            lastreceived = msgreceived;
            msgreceived = millis();
            nrfdebug(msgreceived - lastreceived);
            nrfdebugln(" ms between 2 messages");
            nrfdebug(msgreceived - currenttime);
            nrfdebugln(" ms waited for message after wakeup");

            nrfdebug("Data received: ");
            nrfdebugln(dataReceived);

            if (strcmp(dataReceived, "ALARM") == 0) {
                if (Alarm_flag == 0) {
                    Alarm_flag = 1;
                    nrfdebugln("Alarm activated");
                    // activate the alarm immediately
                    digitalWrite(led_red, HIGH);
#if USEBUZZER == 1
                    digitalWrite(Buzzer, HIGH);
#endif
                }
            }
            // Depending on which message has been received, set certain flags.
            if (strcmp(dataReceived, "TSIC_err") == 0) {

                TSic_error = 1;
                nrfdebugln("TSic Error on Basestation");
            }
            if (strcmp(dataReceived, "SYNC") == 0) {
                if (Alarm_flag == 1) {
                    Alarm_flag = 0;
                    nrfdebugln("Alarm deactivated");
                }
                if (TSic_error == 1) {
                    TSic_error = 0;
                    nrfdebugln("TSic OK");
                }
            }
        }
        Error_handle();
        sleeptimebegin = millis();
        // deletes and resets the timer
        timer2_init();
        nrf_timing = 0;
        // puts nrf-module into sleep
        radio.stopListening();
        radio.powerDown();
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
            // low battery
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
    // If no message is received from the base station within the specified time, turn on the blue LED.
    if (outofsyncflag == 1) {
        digitalWrite(led_blue, HIGH);
    } else {
        digitalWrite(led_blue, LOW);
    }
    // When TSic_error is received from the base station, turn on the green led.
    if (TSic_error == 1) {
        digitalWrite(led_green, HIGH);

    } else {
        digitalWrite(led_green, LOW);
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

// function to initialise the nrf radio module
// returns 1 on error an 0 on success
bool Nrf_RX_Init()
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
    radio.openReadingPipe(1, PairAddress);
    radio.startListening(); // set nrf into RX-Mode
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

// Function to make all LEDs flash x tim
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
float Battery_measurement(const int enable_pin, const int measurement_pin, const int samples)
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
