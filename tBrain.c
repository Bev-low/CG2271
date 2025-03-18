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
extern osMessageQueueId_t motorQueue;
extern osMessageQueueId_t audioQueue;
extern osEventFlagsId_t ledEvent;

void tBrain(void *argument) {
    uint8_t receivedData;

    for (;;) {
        if (osMessageQueueGet(uartQueue, &receivedData, NULL, osWaitForever) == osOK) {
            switch (receivedData) {
                case 'F': osMessageQueuePut(motorQueue, "F", 0, 0); break;  // Move Forward
                case 'B': osMessageQueuePut(motorQueue, "B", 0, 0); break;  // Move Backward
                case 'L': osMessageQueuePut(motorQueue, "L", 0, 0); break;  // Turn Left
                case 'R': osMessageQueuePut(motorQueue, "R", 0, 0); break;  // Turn Right
                case 'S': osMessageQueuePut(motorQueue, "S", 0, 0); break;  // Stop

                case 'M': osEventFlagsSet(ledEvent, 0x01); break;  // Moving LED pattern
                case 'T': osEventFlagsSet(ledEvent, 0x02); break;  // Stopped LED pattern

                case 'A': osMessageQueuePut(audioQueue, "A", 0, 0); break;  // Play song
                case 'E': osMessageQueuePut(audioQueue, "E", 0, 0); break;  // End tone
            }
        }
    }
}

 

