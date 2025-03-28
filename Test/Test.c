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

#define CLOCK_FREQ 48000000 // 48 MHz system clock
#define TPM_PRESCALER 128 

osMessageQueueId_t uartQueue;  // Queue for serial data

#define UART2_INIT_PRIO 0
#define RIGHT_B 0 //TPM1_CH0 PTB0 
#define RIGHT_F 1 //TPM1_CH1 PTB1
#define LEFT_B 2 //TPM2_CH0 PTB2
#define LEFT_F 3 //TPM2_CH1 PTB3

#define NOTE_Bb4  466.16
#define NOTE_B4    493.88
#define NOTE_Eb5  622.25
#define NOTE_F5    698.46
#define NOTE_Gb5  739.99
#define NOTE_Ab5  830.61


// Define note durations (in milliseconds)
#define DURATION_QUARTER 500
#define DURATION_HALF    1000
#define DURATION_WHOLE   2000

#define Buzzer 1 //TPM0_CH0 PTC1

//LED 
#define LED_1 20  //PTE20
#define LED_2 21  //PTE21
#define LED_3 22  //PTE22
#define LED_4 29  //PTE29
#define LED_5 30  //PTE30

#define LED_6 4    //PTA4
#define LED_7 2    //PTC2
#define LED_8 6    //PTC6
#define LED_9 7  //PTAC7
#define LED_10 8  //PTAC8
#define LED_BACK 9  //PTC9

#define MASK(x) (1 << (x))



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
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  
	  //SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
    PORTE->PCR[LED_1] &= ~PORT_PCR_MUX_MASK;  
    PORTE->PCR[LED_2] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_3] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_4] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_5] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[LED_6] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_7] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_8] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_9] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_10] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_BACK] &= ~PORT_PCR_MUX_MASK;
  
    PORTE->PCR[LED_1] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_2] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_3] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_4] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_5] |= PORT_PCR_MUX(1);
  
    PORTA->PCR[LED_6] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_7] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_8] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_9] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_10] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_BACK] |= PORT_PCR_MUX(1);
    
    
    //SET AS OUTPUT
    PTE->PDDR |= MASK(LED_1); 
    PTE->PDDR |= MASK(LED_2); 
    PTE->PDDR |= MASK(LED_3); 
    PTE->PDDR |= MASK(LED_4); 
    PTE->PDDR |= MASK(LED_5); 
    
    PTA->PDDR |= MASK(LED_6); 
    PTC->PDDR |= MASK(LED_7); 
    PTC->PDDR |= MASK(LED_8); 
    PTC->PDDR |= MASK(LED_9); 
    PTC->PDDR |= MASK(LED_10); 
    PTC->PDDR |= MASK(LED_BACK); 
} 

void LED_OFF() {
	PTA->PCOR |= MASK(LED_6);
	PTC->PCOR |= MASK(LED_7) | MASK(LED_8) | MASK(LED_9) | MASK(LED_10);
	PTE->PCOR |= MASK(LED_1) | MASK(LED_2) | MASK(LED_3) | MASK(LED_4) | MASK(LED_5);
}

void LED_ON() {
	PTA->PSOR |= MASK(LED_6);
	PTC->PSOR |= MASK(LED_7) | MASK(LED_8) | MASK(LED_9) | MASK(LED_10);
	PTE->PSOR |= MASK(LED_1) | MASK(LED_2) | MASK(LED_3) | MASK(LED_4) | MASK(LED_5);
}

void LED_RUN() {
	PTE->PSOR |= MASK(LED_1);
	delay(0x80000);
	PTE->PCOR |= MASK(LED_1);
	PTE->PSOR |= MASK(LED_2);
	delay(0x80000);
	PTE->PCOR |= MASK(LED_2);
	PTE->PSOR |= MASK(LED_3);
	delay(0x80000);
	PTE->PCOR |= MASK(LED_3);
	PTE->PSOR |= MASK(LED_4);
	delay(0x80000);
	PTE->PCOR |= MASK(LED_4);
	PTE->PSOR |= MASK(LED_5);
	delay(0x80000);
	PTE->PCOR |= MASK(LED_5);
	PTA->PSOR |= MASK(LED_6);
	delay(0x80000);
	PTA->PCOR |= MASK(LED_6);
	PTC->PSOR |= MASK(LED_7);
	delay(0x80000);
	PTC->PCOR |= MASK(LED_7);
	PTC->PSOR |= MASK(LED_8);
	delay(0x80000);
	PTC->PCOR |= MASK(LED_8);
	PTC->PSOR |= MASK(LED_9);
	delay(0x80000);
	PTC->PCOR |= MASK(LED_9);
	PTC->PSOR |= MASK(LED_10);
	delay(0x80000);
	PTC->PCOR |= MASK(LED_10);
}


void LED_SLOW() {
    PTC->PSOR |= MASK(LED_BACK);
    delay(2400000);  // 500 ms ON
    PTC->PCOR |= MASK(LED_BACK);
    delay(2400000);  // 500 ms OFF
}

void LED_FAST() {
    PTC->PSOR |= MASK(LED_BACK);
    delay(1200000);  // 250 ms ON
    PTC->PCOR |= MASK(LED_BACK);
    delay(1200000);  // 250 ms OFF
}



void LedControl() {
	  //TURN ALL OFF
	  switch (DATA) {
			case 0b00000000:
	        LED_OFF();
			break;
			case 0b00000001:
				LED_ON();
			break;
			case 0b00000010:
				LED_SLOW();
			break;
			case 0b00000011:
				LED_FAST();
			break;
			case 0b00000100: 
				LED_RUN();
		  break;
			default: 
				LED_OFF();
			break;
		}
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


void InitBuzzer() {
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
  PORTC->PCR[Buzzer] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[Buzzer] |= PORT_PCR_MUX(4);
	
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
  
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 
	TPM0->SC = TPM_SC_PS(7);
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
	
	TPM0->SC |= TPM_SC_CMOD(1);
	
}


void InitPWM() {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; 
	
	PORTB->PCR[RIGHT_B] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RIGHT_F] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_B] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_F] &= ~PORT_PCR_MUX_MASK;
	
	PORTB->PCR[RIGHT_B] |= PORT_PCR_MUX(3);
	PORTB->PCR[RIGHT_F] |= PORT_PCR_MUX(3);
	PORTB->PCR[LEFT_B] |= PORT_PCR_MUX(3);
	PORTB->PCR[LEFT_F] |= PORT_PCR_MUX(3);
	
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //select clock
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK; //edge align
	TPM2->MOD = 7500; 

  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~TPM_SC_CPWMS_MASK;
	TPM1->MOD = 7500; 

	
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

	
	
	//START PWM
	TPM2->SC |= TPM_SC_CMOD(1); // Start PWM
	TPM1->SC |= TPM_SC_CMOD(1); // Start PWM
}

void PlayNote(double frequency, uint16_t duration) {
    if (frequency == 0) { // Rest
        TPM0->MOD = 0;
        TPM0->CONTROLS[0].CnV = 0;
    } else {
        TPM0->MOD = (CLOCK_FREQ / (frequency * TPM_PRESCALER)) - 1;
        TPM0->CONTROLS[0].CnV = TPM0->MOD / 2; // 50% Duty Cycle
    }
    for (volatile int i = 0; i < duration * 1000; i++); // Delay
}

void PlayChorus() {
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_QUARTER);
    
    PlayNote(0, 500);
    PlayNote(NOTE_B4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_QUARTER);
  PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
  PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
  
  PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_B4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_B4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_F5, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_F5, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Ab5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Gb5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_F5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_HALF);
    
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_B4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_B4, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Eb5, DURATION_QUARTER);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
    PlayNote(0, 500);
    PlayNote(NOTE_Bb4, DURATION_HALF);
		PlayNote(0, 500);
}


void setMotorSpeed(uint16_t R_B, uint16_t R_F, uint16_t L_B, uint16_t L_F) {
										 
  TPM2_C0V = R_B;//FRONT RIGHT FORWARD
	TPM2_C1V = R_F; //FRONT RIGHT BACKWARD
	
	TPM1_C0V = L_B; //FRONT LEFT FORWARD
	TPM1_C1V = L_F; //FRONT RIGHT BACKWARD
	
}

void tMotorControl() {
    //char command;
	
	  switch (DATA) {
                case 0b00000010: // Move Forward
                    setMotorSpeed(5000, 0, 5000, 0);
                    break;
                case 0b00000001: // Move Backward
                    setMotorSpeed(0, 5000, 0, 5000);
                    break;
                case 0b00000100: // Turn Left
                    setMotorSpeed(0, 5000, 5000, 0);
                    break;
                case 0b00000011: // Turn Right
                    setMotorSpeed(5000, 0, 0, 5000);
                    break;
								case 0b00000111: 
									  PlayChorus();
								    break;
                case 0b10000000: // Stop motors
                case 0b00000000:
                    setMotorSpeed(0, 0, 0, 0);
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
	int count = 0;

    // Initialize UART
    initUART2(115200);
	  InitPWM();
	  InitBuzzer();
	  initLED();
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
			count++;
			//tMotorControl();
			LedControl();
		}
    
	}
