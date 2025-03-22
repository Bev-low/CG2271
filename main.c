#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

// Declare task functions
//extern void tBrain(void *argument);
//extern void tMotorControl(void *argument);
extern void tAudio(void *argument);
extern void tLED(void *argument);

volatile int DATA;
volatile int counter = 0;

osMessageQueueId_t uartQueue;  // Queue for serial data

#define UART2_INIT_PRIO 0
#define FRONTR_F 0 //TPM1_CH0 PTB0
#define FRONTR_B 1 //TPM1_CH1 PTB1
#define FRONTL_F 2 //TPM2_CH0 PTB2
#define FRONTL_B 3 //TPM2_CH1 PTB3

#define BACKR_F 1 //TPM0_CH0 PTC1
#define BACKR_B 2 //TPM0_CH1 PTC2
#define BACKL_F 8 //TPM0_CH4 PTC8
#define BACKL_B 9 //TPM0_CH5 PTC9


//Captures the data and pushes into a Queue for tBrain to decode

/* Delay Function */
static void delay(volatile uint32_t nof) {
while(nof!=0) {
__asm("NOP");
nof--;
}
}


// Define the message queue (shared with tBrain)
//extern osMessageQueueId_t uartQueue;  // Shared queue
osMessageQueueId_t motorQueue;

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

// Interrupt handler for UART receive
void UART2_IRQHandler(void) {
    counter++;
     if (UART2->S1 & UART_S1_RDRF_MASK) {  // Check if data received
        DATA = UART2->D;  // Read UART data
    }
		 
		//osMessageQueuePut(motorQueue, (const void*)&DATA, 0, 0);
        //osMessageQueuePut(uartQueue, &receivedData, 0, 0);  // Push to queue
}

void initLED() {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Enable clock for Port B

    PORTB->PCR[19] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[19] |= PORT_PCR_MUX(1);   // Set PTB19 as GPIO

    PTB->PDDR |= (1 << 19);  // Set PTB19 as output


    PORTB->PCR[18] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[18] |= PORT_PCR_MUX(1);   // Set PTB19 as GPIO

    PTB->PDDR |= (1 << 18);  // Set PTB19 as output
}

/*
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
*/

void InitPWM() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; 
	
	PORTB->PCR[FRONTR_F] &= PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONTR_B] &= PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONTL_F] &= PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONTL_B] &= PORT_PCR_MUX_MASK;
	
	PORTB->PCR[FRONTR_F] |= PORT_PCR_MUX(3);
	PORTB->PCR[FRONTR_B] |= PORT_PCR_MUX(3);
	PORTB->PCR[FRONTL_F] |= PORT_PCR_MUX(3);
	PORTB->PCR[FRONTL_B] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[BACKR_F] &= PORT_PCR_MUX_MASK;
	PORTC->PCR[BACKR_B] &= PORT_PCR_MUX_MASK;
	PORTC->PCR[BACKL_F] &= PORT_PCR_MUX_MASK;
	PORTC->PCR[BACKL_B] &= PORT_PCR_MUX_MASK;
	
	PORTC->PCR[BACKR_F] |= PORT_PCR_MUX(4);
	PORTC->PCR[BACKR_B] |= PORT_PCR_MUX(4);
	PORTC->PCR[BACKL_F] |= PORT_PCR_MUX(3);
	PORTC->PCR[BACKL_B] |= PORT_PCR_MUX(3);
	
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //select clock
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK; //edge align
	TPM2->MOD = 7500; 

  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	TPM1->MOD = 7500; 
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	TPM0->MOD = 7500; 
	
	//TPM1 CH 0 and 1
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	
	//TPM2 CH 0 and 1
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	
	//TPM0 CH 0, 1, 4, 5
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	
	//SET Default DUTY CYCLE IS THIS NEEDED???
	TPM2_C0V = TPM2->MOD / 2;
	TPM2_C1V = TPM2->MOD / 2;
	
	TPM1_C0V = TPM1->MOD / 2;
	TPM1_C1V = TPM1->MOD / 2;
	
	TPM0_C0V = TPM0->MOD / 2;
	TPM0_C1V = TPM0->MOD / 2;
	TPM0_C4V = TPM0->MOD / 2;
	TPM0_C5V = TPM0->MOD / 2;
	
	//START PWM
	TPM2->SC |= TPM_SC_CMOD(1); // Start PWM
	TPM1->SC |= TPM_SC_CMOD(1); // Start PWM
	TPM0->SC |= TPM_SC_CMOD(1); // Start PWM
}

void setMotorSpeed(uint16_t fr_fwd, uint16_t fr_rev, uint16_t fl_fwd, uint16_t fl_rev,
                   uint16_t br_fwd, uint16_t br_rev, uint16_t bl_fwd, uint16_t bl_rev) {
										 
  TPM2_C0V = fr_fwd;//FRONT RIGHT FORWARD
	TPM2_C1V = fr_rev; //FRONT RIGHT BACKWARD
	
	TPM1_C0V = fl_fwd; //FRONT LEFT FORWARD
	TPM1_C1V = fl_rev; //FRONT RIGHT BACKWARD
	
	TPM0_C0V = br_fwd; //BACK RIGHT FORWARD
	TPM0_C1V = br_rev; //BACK RIGHT BACKWARD
	
	TPM0_C4V = bl_fwd; //BACK LEFT FORWARRD
	TPM0_C5V = bl_rev; //BACK LEFT BACKWARD
}

void tMotorControl() {
    //char command;
	
	  switch (DATA) {
                case 0b00000001: // Move Forward
                    setMotorSpeed(5000, 0, 5000, 0, 5000, 0, 5000, 0);
                    break;
                case 0b00000010: // Move Backward
                    setMotorSpeed(0, 5000, 0, 5000, 0, 5000, 0, 5000);
                    break;
                case 0b00000011: // Turn Left
                    setMotorSpeed(0, 3000, 5000, 0, 0, 3000, 5000, 0);
                    break;
                case 0b00000100: // Turn Right
                    setMotorSpeed(5000, 0, 0, 3000, 5000, 0, 0, 3000);
                    break;
                case 0b10000000: // Stop motors
                case 0b00000000:
                    setMotorSpeed(0, 0, 0, 0, 0, 0, 0, 0);
                    break;
            }
	
	/*
    for (;;) {
        if (osMessageQueueGet(motorQueue, &DATA, NULL, osWaitForever) == osOK) {
            switch (command) {
                case 0b00000001: // Move Forward
                    setMotorSpeed(5000, 0, 5000, 0, 5000, 0, 5000, 0);
                    break;
                case 0b00000010: // Move Backward
                    setMotorSpeed(0, 5000, 0, 5000, 0, 5000, 0, 5000);
                    break;
                case 0b00000011: // Turn Left
                    setMotorSpeed(0, 3000, 5000, 0, 0, 3000, 5000, 0);
                    break;
                case 0b00000100: // Turn Right
                    setMotorSpeed(5000, 0, 0, 3000, 5000, 0, 0, 3000);
                    break;
                case 0b10000000: // Stop motors
                case 0b00000000:
                    setMotorSpeed(0, 0, 0, 0, 0, 0, 0, 0);
                    break;
            }
        }
    }
		*/
}

int main(void) {
    SystemCoreClockUpdate();
    //osKernelInitialize();

    // Initialize UART
    initUART2(115200);
	  InitPWM();
	  //motorQueue = osMessageQueueNew(10, sizeof(uint8_t), NULL); 
 
	/*
	// Create message queue for UART data
    uartQueue = osMessageQueueNew(16, sizeof(uint8_t), NULL);
	*/
	 //Create threads
    //osThreadNew(tBrain, NULL, NULL);
    //osThreadNew(tMotorControl, NULL, NULL);
    //osThreadNew(tAudio, NULL, NULL);
    //osThreadNew(tLED, NULL, NULL);

    //osKernelStart();  // Start the RTOS kernel

    //for (;;) {}  // Should never reach here
		
		while (1) {
			tMotorControl();
		}
    
}