/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

//tBrain is the main decoder and releases resources for the other areas

#include "MKL25Z4.h"

extern osMessageQueueId_t uartQueue;
osMessageQueueId_t motorQueue;
osMessageQueueId_t audioQueue;
osMessageQueueId_t ledQueue;
osEventFlagsId_t ledEvent;

void tBrain(void *argument) {
	  uint8_t command; 
	
	  audioQueue = osMessageQueueNew(10, sizeof(uint8_t), NULL);
    ledQueue = osMessageQueueNew(10, sizeof(uint8_t), NULL);
    motorQueue = osMessageQueueNew(10, sizeof(uint8_t), NULL);


    for (;;) {
        if (osMessageQueueGet(uartQueue, &command, NULL, osWaitForever) == osOK) {
            switch (command) {
                case 0b00000001: // Move Forward
                case 0b00000010: // Move Backward
                case 0b00000011: // Move Left
                case 0b00000100: // Move Right
                case 0b00000000: // Stop
                    osMessageQueuePut(motorQueue, &command, 0, 0);
                    break;

                case 0b10000000: // Play Final Tune
                    osMessageQueuePut(audioQueue, &command, 0, 0);
                    break;

                case 0b11000000: // Control LEDs //NOT DONEYET
                    osMessageQueuePut(ledQueue, &command, 0, 0);
                    break;
            }
        }
    }
}
