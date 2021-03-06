/**
 * @file
 * @author  Larry Tesler <ltesler@ufl.edu>
 * @version 2.0
 *
 *
 * @section DESCRIPTION
 *
 * Code for a 8 (expandable to 16) channel pulse sequencer utilizing the Teensy LC MCU
 * This can be used as low cost alternative to a delay generator if a jitter and resolution
 * of 1 us is acceptable.
 *
 * The sequence is phase locked to a LVTTL (3.3V) pulse on the pin 3 which triggers an interrupt that
 * starts the sequence.
 *
 * Utilizes one of two periodic interrupt timers (PIT) on the Teensy LC to keep track of time while
 * keeping interrupts disabled; this reduces jitter caused by the interrupts. Additionally using the
 * PIT allows fine adjustment of delay times since each increment of the PIT corresponds to 41.7 ns
 * (F_BUS = 24 MHz).
 */
#include <Arduino.h>
#include "DelayGenerator.h"


const uint8_t triggerpin = 3; /*!< pin that TTL level trigger is pulsed into */
const uint8_t gpioCport[8] = {15, 22, 23, 9, 10, 13, 11, 12}; /*!< pin assignments for the GPIO port C */
const uint8_t gpioDport[8] = {2, 14, 7, 8, 6, 20, 21, 5}; /*!< pin assignments for the GPIO port D */
const int correction = 36; /*!< correction factor used in PIT timer calculation (see loadSeq) */

void setup() {
    //initialize all variables to zero values
    clearSeq();
    // initialize pins to correct modes
    pinMode(triggerpin, INPUT);
    for (int i = 0; i < 8; i++) pinMode(gpioCport[i], OUTPUT);
    FGPIOC_PCOR = 0xFFFFFFFF; // set all output pins low
    FGPIOC_PSOR = 0x00000010; // set initial pin states
    // initialize periodic interrupt timer (PIT)
    SIM_SCGC6 |= SIM_SCGC6_PIT; //enable clock to PIT module
    PIT_MCR = 0; // enable PIT module
    PORTA_PCR3 |= PORT_PCR_IRQC(9); // set pin to interrupt on rising edge
    NVIC_ENABLE_IRQ(IRQ_PORTA); // enable interrupts from pin
    NVIC_SET_PRIORITY(IRQ_PORTA, 0); // set interrupt priority to maximum (pin 3 is on port A)
    SCB_SHPR3 = (64 << 24) | (SCB_SHPR3 & 0x00FFFFFF); // set systick priority to 64
    Serial.begin(9600); // start serial port

}

/**
 * Run pulse sequence if PIT timer is running, or load pulse sequence through serial
 */
void loop() {

    while (1) {
        if (PIT_TCTRL0) {
            noInterrupts(); // disable all interrupts
            playSeq();
            interrupts(); // renable all interupts
        } else if (Serial.available()) {
            loadSeq();
        } else {

        }
    }
}
/**
 *  Start the PIT timer when pin 3 receives a rising edge
 *
 *  Note, jitter can be reduced to 100 ns if playSeq() is placed here instead of in loop()
 */
FASTRUN void porta_isr() {
    PORTA_ISFR = PORTA_ISFR; // reset interrupt flag
    PIT_TCTRL0 = 1; // start PIT
}

/**
 * Load pulse sequence profile through serial and set PIT timer duration
 *
 * Uses the following format: 2^(pin(s) number), start time (us), end time (us).
 * (e.g. 4, 0 , 100000 16, 1000, 20000 1, 2000, 40000)
 * The sequences must be enter in chronological order based on start time, this function will
 * automatically order the end times.
 *
 */
void loadSeq() {
    uint32_t starttime, pulsepins, duration;
    size_t startlen = 0;
    clearSeq();
    while (Serial.available() > 0) {

        pulsepins = (uint32_t) Serial.parseInt(); // 8 bit int representing pins to be triggered
        starttime = (uint32_t) Serial.parseInt(); // start time of pulse in microseconds
        endtimes[seqlen][0] = (uint32_t) Serial.parseInt(); // end time of pulse in microseconds
        endtimes[seqlen][1] = pulsepins;
        seqlen++;
        // optimize by grouping pulses with identical start times together
        if (startlen == 0) {
            starttimes[startlen] = starttime;
            pulseregstart[startlen] = pulsepins;
            startlen++;
        } else if (starttime == starttimes[startlen - 1]) {
            pulseregstart[startlen - 1] += pulsepins;
        } else {
            starttimes[startlen] = starttime;
            pulseregstart[startlen] = pulsepins;
            startlen++;
        }
    }
    // sort endtimes in chronological order.
    qsort(&endtimes, seqlen, 2 * sizeof(uint32_t), [](const void *pa, const void *pb) -> int {
        return *((const int (*)[2]) pa)[0] > *((const int (*)[2]) pb)[0];
    });
    // convert all times to PIT clock cycle units (F_BUS = 24 MHz for teensy LC)
    duration = endtimes[seqlen - 1][0] * (F_BUS / 1000000); // duration of sequence in clock cycles
    duration += 500; // add some clock cycles as a bit of a buffer to prevent timer overflow
    PIT_LDVAL0 = duration; // load initial time into PIT register;
    for (int i = 0; i < seqlen; i++) {
        // since PIT counts down instead of up, a bit of math is needed
        // an empirical correction factor is also added
        endtimes[i][0] = duration - (endtimes[i][0]) * (F_BUS / 1000000);
        endtimes[i][0] += correction; // correction has units of clock cycles
    }
    for (int i = 0; i < startlen; i++) {
        starttimes[i] = duration - (starttimes[i]) * (F_BUS / 1000000);
        starttimes[i] += correction;
    }
    Serial.println("load complete!");
}
/**
 *  Play the pulse sequence in chronological order by polling the PIT timer
 *
 */
FASTRUN void playSeq() {
    looping = true;
    // continuously loop while waiting for timing events to occur
    while (looping) {
        if (PIT_CVAL0 <= starttimes[startcounter]) {
            FGPIOC_PTOR = pulseregstart[startcounter]; // toggle pins
            startcounter++;
        }
        if (PIT_CVAL0 <= endtimes[endcounter][0]) {
            FGPIOC_PTOR = endtimes[endcounter][1];
            endcounter++;
            if (endcounter >= seqlen) {
                looping = false; // end loop after sequence complete
            }
        }

    }
    PIT_TCTRL0 = 0; // turn off PIT
    startcounter = 0, endcounter = 0; // reset counter

}

/**
 * Set sequence arrays and counter variables to zero
 */
void clearSeq() {
    seqlen = 0, startcounter = 0, endcounter = 0;
    memset(pulseregstart, 0, sizeof(pulseregstart));
    memset(starttimes, 0, sizeof(starttimes));
    memset(endtimes, 0, sizeof(endtimes));
}
