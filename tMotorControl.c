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
#define FRONTR_F 0 //TPM1_CH0 PTB0
#define FRONTR_B 1 //TPM1_CH1 PTB1
#define FRONTL_F 2 //TPM2_CH0 PTB2
#define FRONTL_B 3 //TPM2_CH1 PTB3

#define BACKR_F 1 //TPM0_CH0 PTC1
#define BACKR_B 2 //TPM0_CH1 PTC2
#define BACKL_F 8 //TPM0_CH4 PTC8
#define BACKL_B 9 //TPM0_CH5 PTC9


extern osMessageQueueId_t motorQueue;

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
	TPM2_C0V = TPM1->MOD / 2;
	TPM2_C1V = TPM1->MOD / 2;
	
	TPM1_C0V = TPM1->MOD / 2;
	TPM1_C1V = TPM1->MOD / 2;
	
	TPM0_C0V = TPM1->MOD / 2;
	TPM0_C1V = TPM1->MOD / 2;
	TPM0_C4V = TPM1->MOD / 2;
	TPM0_C5V = TPM1->MOD / 2;
	
	//START PWM
	TPM2->SC |= TPM_SC_CMOD(1); // Start PWM
	TPM1->SC |= TPM_SC_CMOD(1); // Start PWM
	TPM0->SC |= TPM_SC_CMOD(1); // Start PWM
}

void setMotorSpeed(uint16_t fr_fwd, uint16_t fr_rev, uint16_t fl_fwd, uint16_t fl_rev,
                   uint16_t br_fwd, uint16_t br_rev, uint16_t bl_fwd, uint16_t bl_rev) {
										 
  TPM2_C0V = TPM1->fr_fwd//FRONT RIGHT FORWARD
	TPM2_C1V = TPM1->fr_rev; //FRONT RIGHT BACKWARD
	
	TPM1_C0V = TPM1->fl_fwd; //FRONT LEFT FORWARD
	TPM1_C1V = TPM1->fl_rev; //FRONT RIGHT BACKWARD
	
	TPM0_C0V = TPM1->br_fwd; //BACK RIGHT FORWARD
	TPM0_C1V = TPM1->br_rev; //BACK RIGHT BACKWARD
	
	TPM0_C4V = TPM1->bl_fwd; //BACK LEFT FORWARRD
	TPM0_C5V = TPM1->bl_rev; //BACK LEFT BACKWARD
}

void tMotorControl(void *argument) {
    InitPWM();
    char command;

    for (;;) {
        if (osMessageQueueGet(motorQueue, &command, NULL, osWaitForever) == osOK) {
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
}


