#include "cmsis_os2.h"
#include "MKL15Z4.h"                    // Device header

#define UART2_INIT_PRIO 128

//Captures the data and pushes into a Queue for tBrain to decode


// Define the message queue (shared with tBrain)
extern osMessageQueueId_t uartQueue;  // Shared queue

// Interrupt handler for UART receive
void UART2_IRQHandler(void) {
    uint8_t receivedData;
    
    if (UART2->S1 & UART_S1_RDRF_MASK) {  // Check if data received
        receivedData = UART2->D;  // Read UART data
        osMessageQueuePut(uartQueue, &receivedData, 0, 0);  // Push to queue
    }
}


// UART Initialization (unchanged)
void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;

    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK; //Using PORTE 22, would only need to read in
    PORTE->PCR[22] |= PORT_PCR_MUX(4);

    PORTE->PCR[23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[23] |= PORT_PCR_MUX(4);

    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

    bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
    divisor = bus_clock / (baud_rate * 16);
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);

    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;
    
    UART2->C2 |= UART_C2_RIE_MASK;

    NVIC_SetPriority(UART2_IRQn, UART2_INIT_PRIO);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    UART2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
}
