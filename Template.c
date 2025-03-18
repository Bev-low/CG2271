#include "MKL25Z4.h"       
#define SWITCH 17


void PORTD_IRQHandler()
{
 // Clear Pending IRQ

 // Updating some variable / flag

 //Clear INT Flag

}

void initSwitch(void)
{
// enable clock for PortD

/* Select GPIO and enable pull-up resistors and interrupts on
falling edges of pin connected to switch*/

	
// Set PORT D Switch bit to input
	
//Enable Interrupts

}

void initLED() {

	
	//CONFIGURE MUX setting 
	//Making it GPIO Pins

	
	//PORT DATA DIRECTION REGISTER
	//MAKE IT OUTPUT PINS

}


void delay(volatile uint32_t count) {
	while (count--) {
		__NOP();
	}
}


int main(void)
{
initSwitch();
initLED();
while(1)
{
}
}