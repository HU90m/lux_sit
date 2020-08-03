#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "light.h"


// 16MHz clk has a 1024 pre-scalar so measured in 0.064 ms
// 16 bit counter
#define DEBOUNCE_TIME 2000

#define INCREMENT_FACTOR 3


// Pinout Config
#define RED_PWM_REG OCR0B
#define GREEN_PWM_REG OCR0A
#define BLUE_PWM_REG OCR2B
#define WHITE_PWM_REG OCR2A


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
    ALL,
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
volatile state_t state = ALL;

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
        case ALL:
            state = WHITE;
            break;
        case WHITE:
            state = BLUE;
            break;
        case BLUE:
            state = GREEN;
            break;
        case GREEN:
            state = RED;
            break;
        case RED:
            state = ALL;
            break;
        default:
            state = ALL;
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
// Functions
//------------------------------------------------------------------------------
//
uint8_t find_new_dutycycle(uint8_t current_dutycycle, int8_t dutycycle_change) {
    int16_t new_dutycycle;

    new_dutycycle = current_dutycycle + dutycycle_change;

    if(new_dutycycle > 255){
        return 255;
    }
    if(new_dutycycle < 0){
        return 0;
    }

    return current_dutycycle + dutycycle_change;
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
    int8_t increment_factor;
    uint8_t dutycycle_red = 0x0;
    uint8_t dutycycle_green = 0x0;
    uint8_t dutycycle_blue = 0x0;
    uint8_t dutycycle_white = 0x0;

    while(1)
    {
        if( state != state_old)
        {
            switch(state)
            {
            case ALL:
                PORTC &= 0xF8;// resets three LSBs
                break;
            case RED:
                PORTC &= 0xF8;
                PORTC |= 0x04;
                break;
            case GREEN:
                PORTC &= 0xF8;
                PORTC |= 0x02;
                break;
            case BLUE:
                PORTC &= 0xF8;
                PORTC |= 0x01;
                break;
            case WHITE:
                PORTC &= 0xF8;
                PORTC |= 0x07;
                break;
            default:
                state = ALL;
                break;
            }
        }
        if( rotary_movement != 0 )
        {
            switch(state)
            {
            case ALL:
                increment_factor = rotary_movement * INCREMENT_FACTOR;
                dutycycle_red = find_new_dutycycle(
                    dutycycle_red,
                    increment_factor
                );
                dutycycle_green = find_new_dutycycle(
                    dutycycle_green,
                    increment_factor
                );
                dutycycle_blue = find_new_dutycycle(
                    dutycycle_blue,
                    increment_factor
                );
                dutycycle_white = find_new_dutycycle(
                    dutycycle_white,
                    increment_factor
                );
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case RED:
                dutycycle_red = find_new_dutycycle(
                    dutycycle_red,
                    rotary_movement * INCREMENT_FACTOR
                );
                RED_PWM_REG = dutycycle_red;
                break;
            case GREEN:
                dutycycle_green = find_new_dutycycle(
                    dutycycle_green,
                    rotary_movement * INCREMENT_FACTOR
                );
                GREEN_PWM_REG = dutycycle_green;
                break;
            case BLUE:
                dutycycle_blue = find_new_dutycycle(
                    dutycycle_blue,
                    rotary_movement * INCREMENT_FACTOR
                );
                BLUE_PWM_REG = dutycycle_blue;
                break;
            case WHITE:
                dutycycle_white = find_new_dutycycle(
                    dutycycle_white,
                    rotary_movement * INCREMENT_FACTOR
                );
                WHITE_PWM_REG = dutycycle_white;
                break;
            default:
                break;
            }
            if (
                dutycycle_red |
                dutycycle_green |
                dutycycle_blue |
                dutycycle_white
            ) {
                pwm_enable_all();
            } else {
                pwm_disable_all();
            }
            rotary_movement = 0;
        }
        state_old = state;
        _delay_ms(50);
    }
    return 0;
}
