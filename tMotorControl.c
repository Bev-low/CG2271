/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
 
 //This reads in what to do for the motors 
 
#include "MKL25Z4.h"

extern osMessageQueueId_t motorQueue;

void tMotorControl(void *argument) {
    char command;

    for (;;) {
        if (osMessageQueueGet(motorQueue, &command, NULL, osWaitForever) == osOK) {
            switch (command) {
                case 'F': // Set PWM for forward
                    break;
                case 'B': // Set PWM for backward
                    break;
                case 'L': // Set PWM for left turn
                    break;
                case 'R': // Set PWM for right turn
                    break;
                case 'S': // Stop motors
                    break;
            }
        }
    }
}

