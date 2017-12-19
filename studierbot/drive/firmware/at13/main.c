#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 9600000UL
#include <util/delay.h>

#define _SETPB2   (1<<PB2)
#define _MASKPB2 ~(1<<PB2)

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

int _dirTable[4][4] = {{0,-1,1,0},
                       {1,0,0,-1},
                       {-1,0,0,1},
                       {0,1,-1,0}};

int _statePrev = 0;

ISR(PCINT0_vect)
{
  char portBNext = PORTB;
  int pb3high = ((PINB & (1 << PB3))!=0);
  int pb4high = ((PINB & (1 << PB4))!=0);
  int state = ((pb3high << 1) | pb4high);
  int dir = _dirTable[_statePrev][state];
  if(dir==1)
    portBNext |= _SETPB2;
  else if(dir==-1)
    portBNext &= _MASKPB2;

  _statePrev = state;

  portBNext ^= (1 << PB0);

  PORTB = portBNext;
}

int main (void)
{
  asm volatile("push %0"::"r" ("__sreg__"):);
  asm volatile("cli"::);
  CLKPR = (1<<CLKPCE);
  CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
  asm volatile("pop %0"::"r" ("__sreg__"):);

  DDRB  |= (1<<DDB0);                 // Port B Data Direction Register -> PB0 is output
  DDRB  |= (1<<DDB2);                 // Port B Data Direction Register -> PB2 is output
  SREG  |= (1<<7);                    // Global Interrupt Enable

  PORTB |= ((1<<PB3) | (1<<PB4));     // Port B Data Register -> Define pull-up for input ports
  _NOP();                             // Sync

  PCMSK |= ((1<<PINB3) | (1<<PINB4)); // Pin Change Mask Register
  GIMSK |= (1<<PCIE);                 // General Interrupt Mask Register -> Pin Change Interrupt Enable
  GIMSK &= ~(1<<INT0);                // General Interrupt Mask Register -> Pin Change Interrupt Enable
  MCUCR |= (0<<ISC01) | (1<<ISC00);   // MCU Control Register

  PORTB |= (1<<PB2) | (1<<PB0);

  sei();

  while(1)
  {
    _NOP();
  }
  return 0;
}	
