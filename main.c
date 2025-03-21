#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

// Declare task functions
extern void initUART2(uint32_t baud_rate);
extern void tBrain(void *argument);
extern void tMotorControl(void *argument);
extern void tAudio(void *argument);
extern void tLED(void *argument);

osMessageQueueId_t uartQueue;  // Queue for serial data

void initLED() {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Enable clock for Port B

    PORTB->PCR[19] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[19] |= PORT_PCR_MUX(1);   // Set PTB19 as GPIO

    PTB->PDDR |= (1 << 19);  // Set PTB19 as output
}

int main(void) {
    SystemCoreClockUpdate();
    osKernelInitialize();

    // Initialize UART
    initUART2(115200);
	
	  initLED();

    // Create message queue for UART data
    uartQueue = osMessageQueueNew(16, sizeof(uint8_t), NULL);

    // Create threads
    osThreadNew(tBrain, NULL, NULL);
    osThreadNew(tMotorControl, NULL, NULL);
    osThreadNew(tAudio, NULL, NULL);
    osThreadNew(tLED, NULL, NULL);

    osKernelStart();  // Start the RTOS kernel

    for (;;) {}  // Should never reach here
}

