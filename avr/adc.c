#include <avr/io.h>
#include <avr/interrupt.h>

ISR(ADC_vect)
{
    return;
}


void ADCInit(void)
{
    ADMUX = (1 << ADLAR) | (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
}


void ADCStart(char channel)
{   
    // Set input
    ADMUX &= 0xF0;
    ADMUX |= (channel & 0x0F);

    // Activate conversion
    ADCSRA |=  (1 << ADSC);
}

uint16_t ADCRead(void)
{    // Wait for the conversion if single shot
    if (!(ADCSRA & (1 << ADATE))) {
        while (ADCSRA & (1 << ADSC));
    }

    return ADCH;
}
