#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "light.h"


// 16MHz clk has a 1024 pre-scalar so measured in 0.064 ms
// 16 bit counter
#define DEBOUNCE_TIME 2000

#define UPPER_INCREMENT_FACTOR 3
#define LOWER_INCREMENT_FACTOR 1
#define INCREMENT_FACTOR_THRESHOLD 48

#define BREATHE_LOWER_INCREMENT_FACTOR 1
#define BREATHE_UPPER_INCREMENT_FACTOR 10
#define BREATHE_INCREMENT_THRESHOLD 100

#define BREATHE_COUNTER_UPPER 600
#define BREATHE_COUNTER_LOWER 1

#define MAIN_LOOP_DELAY 10


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
    WHITE,
    BREATHE
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
            state = BREATHE;
            break;
        case BREATHE:
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
uint8_t find_new_dutycycle(uint8_t current_dutycycle, int8_t change) {
    int16_t new_dutycycle;
    uint8_t increment_factor;

    if( current_dutycycle > INCREMENT_FACTOR_THRESHOLD)
        increment_factor = UPPER_INCREMENT_FACTOR;
    else
        increment_factor = LOWER_INCREMENT_FACTOR;

    new_dutycycle = current_dutycycle + (change * increment_factor);

    if(new_dutycycle > 255){
        return 255;
    }
    if(new_dutycycle < 0){
        return 0;
    }

    return current_dutycycle + (change * increment_factor);
}

uint16_t find_new_breathe_counter_max(
        uint16_t current_breathe_counter_max,
        int8_t change
) {
    int32_t new_breathe_max;
    uint8_t increment_factor;

    if( current_breathe_counter_max > BREATHE_INCREMENT_THRESHOLD)
        increment_factor = BREATHE_UPPER_INCREMENT_FACTOR;
    else
        increment_factor = BREATHE_LOWER_INCREMENT_FACTOR;

    new_breathe_max = current_breathe_counter_max + (change * increment_factor);

    if(new_breathe_max > BREATHE_COUNTER_UPPER){
        return BREATHE_COUNTER_UPPER;
    }
    if(new_breathe_max < BREATHE_COUNTER_LOWER){
        return BREATHE_COUNTER_LOWER;
    }

    return current_breathe_counter_max + (change * increment_factor);
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

    uint8_t dutycycle_red = 0x0;
    uint8_t dutycycle_green = 0x0;
    uint8_t dutycycle_blue = 0x0;
    uint8_t dutycycle_white = 0x0;

    // breathe variables
    uint16_t breathe_counter = BREATHE_COUNTER_UPPER -1;
    uint16_t breathe_counter_max = BREATHE_COUNTER_UPPER;
    uint8_t breathe_count_up = 0;

    float white_multiplier;
    float blue_multiplier;
    float green_multiplier;
    float red_multiplier;


    while(1)
    {
        // State Change
        if( state != state_old)
        {
            switch(state)
            {
            case ALL:
                PORTC &= 0xF8;// resets three LSBs
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case RED:
                PORTC &= 0xF8;
                PORTC |= 0x04;
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case GREEN:
                PORTC &= 0xF8;
                PORTC |= 0x02;
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case BLUE:
                PORTC &= 0xF8;
                PORTC |= 0x01;
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case WHITE:
                PORTC &= 0xF8;
                PORTC |= 0x07;
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case BREATHE:
                PORTC &= 0xF8;
                PORTC |= 0x05;
                break;
            default:
                state = ALL;
                break;
            }
        }
        // Rotary Movement
        if( rotary_movement != 0 )
        {
            switch(state)
            {
            case ALL:
                dutycycle_red = find_new_dutycycle(
                    dutycycle_red,
                    rotary_movement
                );
                dutycycle_green = find_new_dutycycle(
                    dutycycle_green,
                    rotary_movement
                );
                dutycycle_blue = find_new_dutycycle(
                    dutycycle_blue,
                    rotary_movement
                );
                dutycycle_white = find_new_dutycycle(
                    dutycycle_white,
                    rotary_movement
                );
                RED_PWM_REG = dutycycle_red;
                GREEN_PWM_REG = dutycycle_green;
                BLUE_PWM_REG = dutycycle_blue;
                WHITE_PWM_REG = dutycycle_white;
                break;
            case RED:
                dutycycle_red = find_new_dutycycle(
                    dutycycle_red,
                    rotary_movement
                );
                RED_PWM_REG = dutycycle_red;
                break;
            case GREEN:
                dutycycle_green = find_new_dutycycle(
                    dutycycle_green,
                    rotary_movement
                );
                GREEN_PWM_REG = dutycycle_green;
                break;
            case BLUE:
                dutycycle_blue = find_new_dutycycle(
                    dutycycle_blue,
                    rotary_movement
                );
                BLUE_PWM_REG = dutycycle_blue;
                break;
            case WHITE:
                dutycycle_white = find_new_dutycycle(
                    dutycycle_white,
                    rotary_movement
                );
                WHITE_PWM_REG = dutycycle_white;
                break;
            case BREATHE:
                breathe_counter_max = find_new_breathe_counter_max(
                    breathe_counter_max,
                    rotary_movement
                );
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

        // Breathe algorithm
        if( state == BREATHE ) {

            white_multiplier = dutycycle_white;
            white_multiplier /= breathe_counter_max;

            blue_multiplier = dutycycle_blue;
            blue_multiplier /= breathe_counter_max;

            green_multiplier = dutycycle_green;
            green_multiplier /= breathe_counter_max;

            red_multiplier = dutycycle_red;
            red_multiplier /= breathe_counter_max;

            WHITE_PWM_REG = breathe_counter * white_multiplier;
            BLUE_PWM_REG = breathe_counter * blue_multiplier;
            GREEN_PWM_REG = breathe_counter * green_multiplier;
            RED_PWM_REG = breathe_counter * red_multiplier;


            if ( breathe_count_up ) breathe_counter++;
            else breathe_counter--;

            if( breathe_counter > breathe_counter_max ) breathe_count_up = 0;
            if( breathe_counter <  1) breathe_count_up = 1;
        }

        // state
        state_old = state;
        _delay_ms(MAIN_LOOP_DELAY);
    }
    return 0;
}
