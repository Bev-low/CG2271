/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

// Declare task functions
extern void tBrain(void *argument);
extern void tMotorControl(void *argument);
extern void tLED(void *argument);
extern void tAudio(void *argument);

// Create RTOS resources
osMessageQueueId_t uartQueue;
osMessageQueueId_t motorQueue;
osMessageQueueId_t audioQueue;
osEventFlagsId_t ledEvent;

int main(void) {
    SystemCoreClockUpdate();
    osKernelInitialize();

    // Create message queues
    uartQueue = osMessageQueueNew(10, sizeof(uint8_t), NULL);
    motorQueue = osMessageQueueNew(10, sizeof(char), NULL);
    audioQueue = osMessageQueueNew(10, sizeof(char), NULL);

    // Create event flags
    ledEvent = osEventFlagsNew(NULL);

    // Initialize hardware
    initUART2(9600);

    // Create RTOS tasks
    osThreadNew(tBrain, NULL, NULL);
    osThreadNew(tMotorControl, NULL, NULL);
    osThreadNew(tLED, NULL, NULL);
    osThreadNew(tAudio, NULL, NULL);

    osKernelStart();
    for (;;) {}
}

