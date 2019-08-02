#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "light.h"


#define DEBOUNCE_TIME 200
#define INCREMENT_FACTOR 10

//------------------------------------------------------------------------------
// Type Definitions
//------------------------------------------------------------------------------
//
// Type: state_t
//
// The states of the FSM.
//
typedef enum
{
    OFF,
    RED,
    GREEN,
    BLUE,
    WHITE
} state_t;



//------------------------------------------------------------------------------
// Globals
//------------------------------------------------------------------------------
//
// The current state of the state machine.
//
volatile state_t state = OFF;

//
// The magnitude and direction of the rotary movement.
//
volatile int8_t  rotary_movement = 0;



//------------------------------------------------------------------------------
// Interrupt Service Routines
//------------------------------------------------------------------------------
//
// Changes the state of the state machine when the button is pressed.
//
ISR(INT0_vect)
{
    static uint16_t last_interrupt_time = 0;
    uint16_t interrupt_time = counter_get();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME)
    {
        switch(state)
        {
        case OFF:
            state = RED;
            break;
        case RED:
            state = GREEN;
            break;
        case GREEN:
            state = BLUE;
            break;
        case BLUE:
            state = WHITE;
            break;
        case WHITE:
            state = OFF;
            break;
        default:
            state = OFF;
            break;
        }
    }
    last_interrupt_time = interrupt_time;
}

//
// Decodes a movement of the rotary encoder.
//
ISR(PCINT1_vect)
{
    static uint8_t A;
    static uint8_t B;

    A &= 0x03;// clears 6 MSBs
    B &= 0x03;

    A = (A << 1) | ( bit_is_set(PINC, PC4) ? 1:0 );
    B = (B << 1) | ( bit_is_set(PINC, PC5) ? 1:0 );

    if((0x01 == A) && (0x03 == B))// anti-clockwise
    {
        rotary_movement--;
    }
    if((0x06 == A) && (0x04 == B))// anti-clockwise
    {
        rotary_movement--;
    }
    if((0x03 == A) && (0x01 == B))// clockwise
    {
        rotary_movement++;
    }
    if((0x04 == A) && (0x06 == B))// clockwise
    {
        rotary_movement++;
    }
}



//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
//
int main()
{
    button_initialise();
    rotary_encoder_initialise();
    counter_initialise();
    pwm_initialise();

    // initialise indicator light
    DDRC |= _BV(PC0) | _BV(PC1) | _BV(PC2);

    // enable all interrupts
    sei();

    // variables
    state_t state_old = state;
    uint8_t dutycycle_red = 0xF;
    uint8_t dutycycle_green = 0xF;
    uint8_t dutycycle_blue = 0xF;
    uint8_t dutycycle_white = 0xF;

    while(1)
    {
        if( state != state_old)
        {
            switch(state)
            {
            case OFF:
                pwm_disable_all();
                PORTC &= 0xF8;// resets three LSBs
                break;
            case RED:
                pwm_enable_all();
                PORTC &= 0xF8;
                PORTC |= 0x01;
                break;
            case GREEN:
                pwm_enable_all();
                PORTC &= 0xF8;
                PORTC |= 0x02;
                break;
            case BLUE:
                pwm_enable_all();
                PORTC &= 0xF8;
                PORTC |= 0x04;
                break;
            case WHITE:
                pwm_enable_all();
                PORTC &= 0xF8;
                PORTC |= 0x07;
                break;
            default:
                pwm_disable_all();
                state = OFF;
                break;
            }
        }
        if( rotary_movement != 0 )
        {
            switch(state)
            {
            case RED:
                dutycycle_red += (rotary_movement * INCREMENT_FACTOR);
                OCR0A = dutycycle_red;
                break;
            case GREEN:
                dutycycle_green += (rotary_movement * INCREMENT_FACTOR);
                OCR0B = dutycycle_green;
                break;
            case BLUE:
                dutycycle_blue += (rotary_movement * INCREMENT_FACTOR);
                OCR2A = dutycycle_blue;
                break;
            case WHITE:
                dutycycle_white += (rotary_movement * INCREMENT_FACTOR);
                OCR2B = dutycycle_white;
                break;
            default:
                break;
            }
            rotary_movement = 0;
        }
        state_old = state;
        _delay_ms(50);
    }
    return 0;
}
