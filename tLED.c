/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 //Controls the LED that are connected to Freedom
 
#include "MKL25Z4.h"

extern osEventFlagsId_t ledEvent;

void tLED(void *argument) {
    for (;;) {
        uint32_t flags = osEventFlagsWait(ledEvent, 0x03, osFlagsWaitAny, osWaitForever);

        if (flags & 0x01) {
            // Running LED pattern (moving)
        } 
        if (flags & 0x02) {
            // Solid LED pattern (stopped)
        }
    }
}
