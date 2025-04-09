#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"


volatile int DATA;
volatile int IRQCounter = 0;
volatile int motorControlCounter = 0;
volatile int motorSpdCounter = 0;
volatile int LEDCounter = 0;
volatile int DEBUGCOUNTER = 0;
volatile int audioCounter = 0;

#define UART2_INIT_PRIO 3

//LED

#define LED_1 8  //PTC8
#define LED_2 4  //PTA4
#define LED_3 7  //PTC7
#define LED_4 6  //PTC6
#define LED_5 20  //PTE20

#define LED_6 21    //PTE21
#define LED_7 22    //PTE22
#define LED_8 29    //PTE29
#define LED_9 30  //PTE30
#define LED_10 1  //PTC1
#define LED_BACK 9  //PTC9

#define MASK(x) (1 << (x))

//MOTOR

#define RIGHT_B 0 //TPM1_CH0 PTB0 
#define RIGHT_F 1 //TPM1_CH1 PTB1
#define LEFT_B 2 //TPM2_CH0 PTB2  
#define LEFT_F 3 //TPM2_CH1 PTB3

//AUDIO

#define CLOCK_FREQ 48000000 // 48 MHz system clock
#define TPM_PRESCALER 128 

#define NOTE_Bb4  466.16
#define NOTE_B4    493.88
#define NOTE_Eb5  622.25
#define NOTE_F5    698.46
#define NOTE_Gb5  739.99
#define NOTE_Ab5  830.61

#define NOTE_A3  220
#define NOTE_E4  329.63
#define NOTE_C4  261.63
#define NOTE_B3  246.94


// Define note durations (in milliseconds)
#define DURATION_QUARTER 500
#define DURATION_HALF    1000
#define DURATION_WHOLE   2000

#define Buzzer 2 //TPM0_CH1 PTC2

osMessageQueueId_t brainMsg, motorMsg, ledMsg_Front, audioMsg, ledMsg_Back;
osThreadId_t tMotor_Id, tBrain_Id;
uint8_t MSG_COUNT = 1;

uint8_t myDataPkt;

/* Delay Function */
static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}


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
    IRQCounter++;
    if (UART2->S1 & UART_S1_RDRF_MASK) {  // Check if data received
        DATA = UART2->D;  // Read UART data
      myDataPkt = DATA;
      osMessageQueuePut(motorMsg, &myDataPkt, NULL, 0);
    }
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
  TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(4));
  TPM2->SC &= ~TPM_SC_CPWMS_MASK; //edge align
  TPM2->MOD = 6000; 

  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); // Clear CMOD and Prescaler
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(4));
  TPM1->SC &= ~TPM_SC_CPWMS_MASK;
  TPM1->MOD = 6000;
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

void setMotorSpeed(uint16_t R_B, uint16_t R_F, uint16_t L_B, uint16_t L_F) {
  
  motorSpdCounter++;
  
  TPM2->MOD = 6000;
  TPM2_C0V = R_B;//FRONT RIGHT FORWARD
  TPM2_C1V = R_F; //FRONT RIGHT BACKWARD
  
  TPM1->MOD = 6000;
  TPM1_C0V = L_B; //FRONT LEFT FORWARD
  TPM1_C1V = L_F; //FRONT RIGHT BACKWARD
  
  
  
  
}

void tMotorControl(void *argument) {
    uint8_t CMD; 
	uint8_t CMD_LED, CMD_AUD; 
    
    for(;;) {
        if (osMessageQueueGet(motorMsg, &CMD, NULL, osWaitForever) == osOK) {
        motorControlCounter++;
					CMD_LED = CMD;
					osMessageQueuePut(ledMsg_Front, &CMD_LED, NULL, 0); //Put inside 
					osMessageQueuePut(ledMsg_Back, &CMD_LED, NULL, 0); //Put inside 
					CMD_AUD = CMD;
					osMessageQueuePut(audioMsg, &CMD_AUD, NULL, 0); //Put inside 
      
					
				//JOYSTICK COMMANDS	
				if (CMD == 0b00000001) { // Move FORWARD, 
            setMotorSpeed(5400, 0, 0, 5400);
					  DEBUGCOUNTER++;
        }
        else if (CMD == 0b00000010) { // Move BACKWARD
            setMotorSpeed(0, 6000, 6000, 0);
        }
        else if (CMD == 0b00000100) { // Turn Left
            setMotorSpeed(6000, 0, 6000, 0);
        }
        else if (CMD == 0b00000011) { // Turn Right
            setMotorSpeed(0, 6000, 0, 6000);
        }
        else if (CMD == 0b00000000) { // STOP!
            setMotorSpeed(0, 0, 0, 0);
        }
				
				//JOYSTICK DIAGONALS
				
				else if (CMD == 0b00000101) { //UP-LEFT, 
            setMotorSpeed(0, 0, 0, 6000);
        }
        else if (CMD == 0b00000110) { //UP-RIGHT
            setMotorSpeed(6000, 0, 0, 0);
        }
        else if (CMD == 0b00000111) { //DOWN-LEFT
            setMotorSpeed(0, 0, 6000, 0);
        }
        else if (CMD == 0b00001000) { //DOWN-RIGHT
            setMotorSpeed(0, 6000, 0, 0);
        }
				
				//D-PAD COMMANDS
        else if (CMD == 0b10000001) {		//SLOW FORWARDS
            setMotorSpeed(6000, 0, 0, 6000);
        }
				else if (CMD == 0b10000010) {		//SLOW BACKWARDS
            setMotorSpeed(0, 6000, 6000, 0);
        }
				else if (CMD == 0b10000100) {		//SLOW LEFT
            setMotorSpeed(5250, 0, 5250, 0);
        }
				else if (CMD == 0b10000011) {		//SLOW RIGHT
            setMotorSpeed(0, 5250, 0, 5250);
        }
        //osDelay(10);
      }
      }
    }
      
void initLED() {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    //PORT E ALR CLOCK GATED IN INITAURT

    PORTC->PCR[LED_1] &= ~PORT_PCR_MUX_MASK;  
    PORTA->PCR[LED_2] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_3] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_4] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_5] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_6] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_7] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_8] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_9] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_10] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[LED_BACK] &= ~PORT_PCR_MUX_MASK;
  
    PORTC->PCR[LED_1] |= PORT_PCR_MUX(1);
    PORTA->PCR[LED_2] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_3] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_4] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_5] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_6] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_7] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_8] |= PORT_PCR_MUX(1);
    PORTE->PCR[LED_9] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_10] |= PORT_PCR_MUX(1);
    PORTC->PCR[LED_BACK] |= PORT_PCR_MUX(1);
    
    
    //SET AS OUTPUT
    PTC->PDDR |= MASK(LED_1); 
    PTA->PDDR |= MASK(LED_2); 
    PTC->PDDR |= MASK(LED_3); 
    PTC->PDDR |= MASK(LED_4); 
    PTE->PDDR |= MASK(LED_5); 
    PTE->PDDR |= MASK(LED_6); 
    PTE->PDDR |= MASK(LED_7); 
    PTE->PDDR |= MASK(LED_8); 
    PTE->PDDR |= MASK(LED_9); 
    PTC->PDDR |= MASK(LED_10);
    PTC->PDDR |= MASK(LED_BACK); 
}

void LED_OFF() {
  PTA->PCOR |= MASK(LED_2) ;
  PTC->PCOR |= MASK(LED_1) | MASK(LED_3)  | MASK(LED_4) | MASK(LED_10);
  PTE->PCOR |= MASK(LED_5) | MASK (LED_6) | MASK(LED_7) | MASK(LED_8) | MASK(LED_9);
}

void LED_ON() {
  PTA->PSOR |= MASK(LED_2) ;
  PTC->PSOR |= MASK(LED_1) | MASK(LED_3)  | MASK(LED_4) | MASK(LED_10);
  PTE->PSOR |= MASK(LED_5) | MASK (LED_6) | MASK(LED_7) | MASK(LED_8) | MASK(LED_9);
}

void LED_RUN() {
  PTC->PSOR |= MASK(LED_1);
  osDelay(150);
  PTC->PCOR |= MASK(LED_1);
  PTC->PSOR |= MASK(LED_4);
  osDelay(150);
  PTC->PCOR |= MASK(LED_4);
  PTE->PSOR |= MASK(LED_5);
  osDelay(150);
  PTE->PCOR |= MASK(LED_5);
  PTE->PSOR |= MASK(LED_6);
  osDelay(150);
  PTE->PCOR |= MASK(LED_6);
  PTE->PSOR |= MASK(LED_7);
  osDelay(150);
  PTE->PCOR |= MASK(LED_7);
  PTE->PSOR |= MASK(LED_8);
  osDelay(150);
  PTE->PCOR |= MASK(LED_8);
  PTE->PSOR |= MASK(LED_9);
  osDelay(150);
  PTE->PCOR |= MASK(LED_9);
  PTC->PSOR |= MASK(LED_10);
  osDelay(150);
  PTC->PCOR |= MASK(LED_10);
}


void LED_SLOW() {
    PTC->PSOR |= MASK(LED_BACK);
	  LEDCounter++;
    //delay(2400000);  // 500 ms ON
	  osDelay(500);
    PTC->PCOR |= MASK(LED_BACK);
    //delay(2400000);  // 500 ms OFF
	  osDelay(500);
}

void LED_FAST() {
    PTC->PSOR |= MASK(LED_BACK);
    //delay(1200000);  // 250 ms ON
	  osDelay(250);
    PTC->PCOR |= MASK(LED_BACK);
    //delay(1200000);  // 250 ms OFF
	  osDelay(250);
}
void tLED_Back (void *argument) {
	  

  uint8_t CMD; 
  
    for (;;) { 
      
      if (osMessageQueueGet(ledMsg_Back, &CMD, NULL, osWaitForever) == osOK) {
        
        
    if (CMD == 0b00000000) {
            LED_FAST(); //FRONT
        }
        else { 
            LED_SLOW(); //BACK
        }
		}
  }
}
void tLED_Front(void *argument) {
	  

  uint8_t CMD; 
  
    for (;;) { 
      
      if (osMessageQueueGet(ledMsg_Front, &CMD, NULL, osWaitForever) == osOK) {
        
        
    if (CMD == 0b00000000) {
            LED_ON(); //FRONT
        }
        else { 
            LED_RUN(); //FRONT
        }
		}
  }
}

void InitBuzzer() {
	
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
  //SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //In tLED
	
  PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[2] |= PORT_PCR_MUX(4);
	
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |  (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); // Clear mode bits
  TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Set Edge-Aligned PWM with High-True pulses
  
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); 
	TPM0->SC = TPM_SC_PS(7);
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;
	
	
	TPM0->SC |= TPM_SC_CMOD(1);
	
}

void PlayNote(double frequency, uint16_t duration) {
    if (frequency == 0) { // Rest
        TPM0->MOD = 0;
        TPM0->CONTROLS[1].CnV = 0;
    } else {
        TPM0->MOD = (CLOCK_FREQ / (frequency * TPM_PRESCALER)) - 1;
        TPM0->CONTROLS[1].CnV = TPM0->MOD / 2; // 50% Duty Cycle
    }
    for (volatile int i = 0; i < duration * 1000; i++); // Delay
}

void PlayEnding() {
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

void PlayChorus() {				//MAX VERSTAPPEN
    PlayNote(0, 500);
    PlayNote(NOTE_A3, DURATION_QUARTER * 1.5);
    PlayNote(0, 500);
    PlayNote(NOTE_A3, DURATION_QUARTER * 1.5);
    PlayNote(0, 500);
    PlayNote(NOTE_A3, DURATION_QUARTER );
    PlayNote(0, 500);
    PlayNote(NOTE_E4, DURATION_WHOLE );
    PlayNote(0, DURATION_HALF * 2.5);
		PlayNote(NOTE_E4, DURATION_QUARTER * 2);
    PlayNote(0, 500);
		PlayNote(NOTE_E4, DURATION_QUARTER);
    PlayNote(0, 500);
		PlayNote(NOTE_C4, DURATION_QUARTER * 2);
    PlayNote(0, 500);
		PlayNote(NOTE_B3, DURATION_QUARTER * 2);
    PlayNote(0, 500);
}

void tAudio(void *argument) {
	  uint8_t CMD; 
	
	  for (;;) { 

			if (osMessageQueueGet(audioMsg, &CMD, NULL, osWaitForever) == osOK) {			
				audioCounter++;
				if (CMD == 0b11111111) {
					PlayEnding();
				} else {
					PlayChorus();
				}
	}
}
		}

  

int main(void) {
	SystemCoreClockUpdate();
    initUART2(115200);
    InitPWM();
    initLED();
	  InitBuzzer();
	 DEBUGCOUNTER++;
  
    osKernelInitialize();

    motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
  	ledMsg_Front = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	  ledMsg_Back = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	  audioMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);

    //Create threads
    osThreadNew(tMotorControl, NULL, NULL);
	  osThreadNew(tLED_Front, NULL, NULL);
	  osThreadNew(tLED_Back, NULL, NULL);
	  osThreadNew(tAudio, NULL, NULL);
  
    osKernelStart();  // Start the RTOS kernel

    for (;;) {}  // Should never reach here
    
}


