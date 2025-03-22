/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 // Play a certian tone until it recieves the command to play continous tone
 /*
#include "MKL25Z4.h"

xtern osMessageQueueId_t audioQueue;

void tAudio(void *argument) {
    char command;

    for (;;) {
        if (osMessageQueueGet(audioQueue, &command, NULL, osWaitForever) == osOK) {
            if (command == 'A') {
                // Play continuous song
            } else if (command == 'E') {
                // Play end tone
            }
        }
    }
}
*/

