#ifndef LIGHT_H
#define LIGHT_H

//------------------------------------------------------------------------------
// Pulse Width Modulation
//------------------------------------------------------------------------------
//
// Initialises the PWM Outputs
//
void pwm_initialise()
{
    // set mode of operation to FastPWM
    TCCR0A |= (1<<WGM00 | 1<<WGM01);
    TCCR2A |= (1<<WGM20 | 1<<WGM21);

    // enable clock source with /8 pre-scaler
    TCCR0B |= (1<<CS01);
    TCCR2B |= (1<<CS21);

    // initialise with 50% duty cycle
    OCR0A = 0x80;
    OCR0B = 0x80;
    OCR2A = 0x80;
    OCR2B = 0x80;

    // 4 PWM channel outputs
    DDRB |= 1<<PB3; // OC2A
    DDRD |= 1<<PD3; // OC2B
    DDRD |= 1<<PD5; // OC0B
    DDRD |= 1<<PD6; // OC0A
}

//
// Enable All PWM Channels
//
void pwm_enable_all()
{
    TCCR0A |= 1<<COM0A1;
    TCCR0A |= 1<<COM0B1;
    TCCR2A |= 1<<COM2A1;
    TCCR2A |= 1<<COM2B1;
}

//
// Disable All PWM Channels
//
void pwm_disable_all()
{
    TCCR0A &= ~(1<<COM0A1);
    TCCR0A &= ~(1<<COM0B1);
    TCCR2A &= ~(1<<COM2A1);
    TCCR2A &= ~(1<<COM2B1);
}



//------------------------------------------------------------------------------
// Counter
//------------------------------------------------------------------------------
//
// Initialises the Counter
//
void counter_initialise()
{
    // enable clock with 1024 pre-scaler
    TCCR1B |=  _BV(CS12);
    TCCR1B &= ~_BV(CS11);
    TCCR1B |=  _BV(CS10);
}

//
// Get the Value of the Counter
//
uint16_t counter_get()
{
    uint16_t count;
    count  = TCNT1L;
    count |= TCNT1H << 8;
    return count;
}



//------------------------------------------------------------------------------
// Button
//------------------------------------------------------------------------------
//
// Initialises the Button
//
void button_initialise()
{
    // set pin D2 as output
    DDRD &= ~(1 << PD2);

    // setup pull up resistor on pin D2
    PORTD |= (1 << PD2);

    // setup external interrupt 0
    EICRA |= _BV(ISC01) | _BV(ISC00);
    EIMSK |= _BV(INT0);
}



//------------------------------------------------------------------------------
// Rotary Encoder
//------------------------------------------------------------------------------
//
// Initialises the Rotary Encoder
//
void rotary_encoder_initialise()
{
    // set pins output
    DDRC  &= ~_BV(PC4);
    DDRC  &= ~_BV(PC5);

    // setup pull up resistor on pins
    PORTC |=  _BV(PC4);
    PORTC |=  _BV(PC5);

    // enable pin change interrupt 1
    PCICR  |= _BV(PCIE1);

    // mask the two used pins through
    PCMSK1  = _BV(PCINT12);
    PCMSK1 |= _BV(PCINT13);
}

#endif
