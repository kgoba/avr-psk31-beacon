/* avr-psk31-beacon
 * PSK31 Beacon generator with AF output.
 *
 * (c) Karl-Martin Skontorp <kms@skontorp.net> ~ http://22pf.org/ ~ LA9PMA
 * Licensed under the GNU GPL 2.0 or later.
 */

#define F_CPU 18432E3                       // CPU clock frequency
#define BB_F 732                            // ???
#define SIN_LUT_SIZE 128                    // Number of entries in sine lookup table
//#define SYM_PERIOD ((0.032 * SIN_LUT_SIZE) / (1.0 / BB_F))
#define SYM_PERIOD 2304

// PWM on OC0A (PD6)
#define PWMPIN_SETUP  DDRD |= (1 << PD6)

// Output on PD0
#define OUTPIN_SETUP  DDRD |= (1 << PD0)
#define OUTPIN_TOGGLE PORTD ^= (1 << PD0)
#define OUTPIN_TOGGLE PORTD ^= (1 << PD0)

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "varicode.h"
#include "fifo.h"

uint8_t txChar(void);


void prepareSinLUT(void);

// Sine lookup table
int8_t sinLUT[SIN_LUT_SIZE];

// Output compare interrupt
ISR(SIG_OUTPUT_COMPARE0A) {
    static uint8_t shift;
    static uint8_t k;
    static uint8_t i;
    static uint16_t p;

    p++;

    if (p == SYM_PERIOD / 2) {
        if (txChar() == 0) {
            shift = 1;
        } else {
            shift = 0;
        }

        // Toggle output pin
        OUTPIN_TOGGLE;
    } 
    else if (p == SYM_PERIOD) {
        p = 0;
        if (shift) {
            // Shift phase by 180 degrees
            i += SIN_LUT_SIZE / 2;
        }
    }

    uint16_t x;

    x = sinLUT[i++ % SIN_LUT_SIZE];

    // Modulate amplitude in case of shift
    if (shift) {
        x *= sinLUT[(p / 36)];        // phase = -pi .. 0
    } else {
        // Full amplitude
        x *= 0x7f;                  
    }

    x = x >> 7;             // convert back to Q7
    x += 128;               // convert to UQ7

    // Modify PWM duty
    OCR0A = 255 - x;
}

uint8_t txChar() {
    static char txString[] = 
        "\t"
        "\t\t...  LA9PMA/B LA9PMA/B  ...\r\n"
        "\t\t... Experimental Beacon ...\r\n"
        "\t\tLA9PMA/B < JO59fg70 Tonsberg >\r\n"
        "\t\tLA9PMA/B < 5W in dipole >\r\n"
        "\t\tLA9PMA/B < 28.321.732MHz >\r\n"
        "\t\tLA9PMA/B < Rpts to kms@skontorp.net >\r\n"
        "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
    static char *txStringPos = txString;
    static int8_t txBitPos = 0;
    uint8_t result;
    uint8_t patternLength;

    patternLength = psk31[(uint8_t) *txStringPos].length;

    if (txBitPos == patternLength || (txBitPos == patternLength + 1)) {
        txBitPos++;
        return 0;
    } else if (txBitPos == patternLength + 2) {
        txBitPos = 0;
        if (*++txStringPos == '\0') {
            txStringPos = txString;
        }
        patternLength = psk31[(uint8_t) *txStringPos].length;
    }

    if (psk31[(uint8_t) *txStringPos].pattern & (1 << (patternLength - 1 - txBitPos))) {
        result = 1;
    } else {
        result = 0;
    }

    txBitPos++;

    return result;
}

void prepareSinLUT() {
    uint8_t l;

    // Compute sin(x) for x = -pi .. pi and store in Q7 format
    for (l = 0; l < SIN_LUT_SIZE; l++) {
        double d;

        d = sin((((((double) l) / SIN_LUT_SIZE)) - 0.5) * M_PI * 2.0);
        d *= 127.0;
        sinLUT[l] = d;
    }
}

int main(void) {
    wdt_reset();
    wdt_disable();

    // Configure IO pins
    PWMPIN_SETUP();
    OUTPIN_SETUP();

    // Compute lookup table
    prepareSinLUT();

    // Setup Timer0 for PWM output at 72 kHz
    TCCR0A = _BV(COM0A1) | _BV(COM0A0) |  // Enable PWM output on OC0A
              _BV(WGM01) | _BV(WGM00);    // Fast PWM mode (TOP=255) 
    TCCR0B = _BV(CS00);                   // Prescaler FCPU/1
    TIMSK0 = _BV(OCIE0A);                 // Enable OCR0A compare interrupt

    // Enable serial UART
    //UCSR0B = _BV(TXEN0);
    //UCSR0B |= _BV(UDRIE0);
    //UBRR0L = 9;

    // Prepare and enter sleep loop
    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();
    for (;;) {
        sleep_mode();
    }
}
