/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <stdbool.h>

#define DATA_STATIONARY 0
#define DATA_BLUETOOTH_SUCCESS 1
#define DATA_BLUETOOTH_DISCONNECT 2
#define DATA_UP 3
#define DATA_DOWN 4
#define DATA_LEFT 5 //curved left
#define DATA_RIGHT 6 //curve right
#define DATA_END 7

#define PTB0_Pin 0
#define PTB1_Pin 1
#define CORE_CLK 48000000
#define FREQ_C 262
#define FREQ_D 294
#define FREQ_E 330
#define FREQ_F 349
#define FREQ_G 392
#define FREQ_A 440
#define FREQ_B 494
#define PRESCALAR 128

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTD2 2
#define UART2_INT_PRIO PRESCALAR

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))
#define SW_POS 6

#define RED_LED_1 5 // PortE Pin 5
#define RED_LED_2 4 // PortE Pin 4
#define RED_LED_3 3 // PortE Pin 3
#define RED_LED_4 2 // PortE Pin 2
#define RED_LED_5 11 // PortB Pin 11
#define RED_LED_6 10 // PortB Pin 10
#define RED_LED_7 9 // PortB Pin 9
#define RED_LED_8 8 // PortB Pin 8
#define GREEN_LED_1 7 // PortC Pin 7
#define GREEN_LED_2 0 // PortC Pin 0
#define GREEN_LED_3 3 // PortC Pin 3
#define GREEN_LED_4 4 // PortC Pin 4
#define GREEN_LED_5 5 // PortC Pin 5
#define GREEN_LED_6 6 // PortC Pin 6
#define GREEN_LED_7 10 // PortC Pin 10
#define GREEN_LED_8 11 // PortC Pin 11

#define FRONT_LEFT_MOTOR 30	 // PortE Pin 30
#define FRONT_RIGHT_MOTOR 29 // PortE Pin 29
#define BACK_LEFT_MOTOR 23 // PortE Pin 23
#define BACK_RIGHT_MOTOR 22 // PortE Pin 22


enum color_t{red, green, blue};
enum led_state{led_on, led_off};
enum motion_state{stationary, up, down, left, right};

volatile uint8_t rx_data;
volatile bool is_connected = false;
volatile bool has_started = false;
volatile enum motion_state curr_motion_state = up;
volatile bool has_completed = false;


/* Init UART2 */
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //supply power to UART2 module
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; //supply power to PORTE 
	
	//PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK; //clear MUX bits for PORT E pin 22
	//PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4); //set PORT E pin 22 to use UART2_TX module
	
	PORTD->PCR[UART_RX_PORTD2] &= ~PORT_PCR_MUX_MASK; //clear MUX bits for PORT E pin 23
	PORTD->PCR[UART_RX_PORTD2] |= PORT_PCR_MUX(3); //set PORT E pin 23 to use UART2_RX module
	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //clear TE and RE bits for initialization
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2; 
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0; 
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	NVIC_SetPriority(UART2_IRQn, 2);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
	
	
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// can send another character
		UART2->C2 &= ~UART_C2_TIE_MASK; //turn off transmit interrupt because not using for now
	}
		
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		
		rx_data = UART2->D;
		
	}
	
	if (UART2->S1 & (UART_S1_OR_MASK |
			UART_S1_NF_MASK |
			UART_S1_FE_MASK |
			UART_S1_PF_MASK)) {
// handle the error
// clear the flag
	}
	
}


/* UART2 Transmit Poll */
void UART2_Transmit_Poll(uint8_t data) {
	while(!(UART2->S1 & UART_S1_TDRE_MASK));
	UART2->D = data;
}

/* UART2 Receive Poll */
uint8_t UART2_Receive_Poll(void) {
	while(!(UART2->S1 & UART_S1_RDRF_MASK));
	return (UART2->D);
}

/*-------- LED CODE -----------------------------------------------------------
------------------------------------------------------------------------------*/

void initLED(void) {
	
	// Enable Clock to PORTB, PORTC and PORTE
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTE_MASK));
	
	// Configure MUX settings to make all 16 pins GPIO
	PORTE->PCR[RED_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED_1] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[RED_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED_2] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[RED_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED_3] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[RED_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED_4] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[RED_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_5] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[RED_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_6] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[RED_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_7] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[RED_LED_8] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_8] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED_5) | MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8));
	PTE->PDDR |= (MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4));
	PTC->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
}

void single_green_led_on(int led_pos) {
	switch (led_pos) {
		case 1:
			PTC->PCOR |= (MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_1));
			break;
		case 2:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_2));
			break;
		case 3:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_3));
			break;
		case 4:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_4));
			break;
		case 5:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_5));
			break;
		case 6:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_6));
			break;
		case 7:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_8));
			PTC->PSOR |= (MASK(GREEN_LED_7));
			break;
		case 8:
			PTC->PCOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7));
			PTC->PSOR |= (MASK(GREEN_LED_8));
			break;	
	}
}

void all_green_led_on() {
	PTC->PSOR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8));
}

void green_led_flashing() {
	int green_led_pos = 1;
	while (curr_motion_state != stationary) {
		single_green_led_on(green_led_pos);
		osDelay(250);
		green_led_pos++;
		if (green_led_pos >= 9) {
			break;
		}
	}
}

void all_red_led_flashing_stationary() {
	PTB->PSOR |= (MASK(RED_LED_5) | MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8));
	PTE->PSOR |= (MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4));
	osDelay(250);
	PTB->PCOR |= (MASK(RED_LED_5) | MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8));
	PTE->PCOR |= (MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4));
	osDelay(250);
}

void all_red_led_flashing_running() {
	PTB->PSOR |= (MASK(RED_LED_5) | MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8));
	PTE->PSOR |= (MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4));
	osDelay(500);
	PTB->PCOR |= (MASK(RED_LED_5) | MASK(RED_LED_6) | MASK(RED_LED_7) | MASK(RED_LED_8));
	PTE->PCOR |= (MASK(RED_LED_1) | MASK(RED_LED_2) | MASK(RED_LED_3) | MASK(RED_LED_4));
	osDelay(500);
}	


int calc_MOD(int freq, int prescaler) {
	int reduced_freq = CORE_CLK / prescaler;
	float period_reduced_freq = 1.0 / reduced_freq;
	float period_PWM = 1.0/freq;
	int num_clk_cycles = (period_PWM / period_reduced_freq) - 1;
	return num_clk_cycles;
}

void off_sound() {
	TPM1->MOD = calc_MOD(0, 128);
	TPM1_C0V = calc_MOD(0, 128);
}

void sound_break() {
	off_sound();
	osDelay(50);
}

void play_C(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_C, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_C, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_D(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_D, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_D, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_E(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_E, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_E, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_F(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_F, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_F, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_G(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_G, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_G, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_A(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_A, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_A, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_B(int time_delay) {
	TPM1->MOD = calc_MOD(FREQ_B, PRESCALAR);
	TPM1_C0V = calc_MOD(FREQ_B, PRESCALAR) / 2;
	osDelay(time_delay);
	sound_break();
}

void play_start_sound() {
	play_C(1000);
	play_G(1000);
}

void play_end_sound() {
	
}

void play_constant_song() {
	play_E(500);
	play_D(500);
	play_C(500);
	play_D(500);
	play_E(500);
	play_E(500);
	play_E(1000);
	play_D(500);
	play_D(500);
	play_D(1000);
	play_E(500);
	play_E(500);
	play_E(1000);
	play_E(500);
	play_D(500);
	play_C(500);
	play_D(500);
	play_E(500);
	play_E(500);
	play_E(1000);
	play_D(500);
	play_D(500);
	play_E(500);
	play_D(500);
	play_C(2000);
}

/* initPWM() */
void initPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //enable clock gate for port b
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK; //clear MUX 
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); //setting PTB0 pin to be PWN signal (TPM1_CH0)
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK; //clear MUX bits
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3); //setting PTB1 pin to be PWN signal (TPM1_CH1)
	
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; //enable clock gate for TPM1 module
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //clear TPMSRC bits
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //enable TPMSRC MCGFLLCLK clock or Multipurpose Clock Generator Frequency-Locked Loop Clock
	
	//TPM1->MOD = 0b0001110101001100;
	//TPM1_C0V = 0b0001110101001100;
	TPM1->MOD = calc_MOD(50, PRESCALAR);
	TPM1_C0V = calc_MOD(50, PRESCALAR);
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //clear PS(prescalar) and CMOD(counter mode) bits
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //PS7 is set prescalar to 128, CMOD set LPTPM counter to increase on every LPTPM counter clock
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK); //clear CPWMS bit = LPTPM counter set to up-counting mode
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //clear ELSB ELSA MSB MSA bits (M - mode, EL - Edge/Level)
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Mode = Edge-aligned PWM, Configuration = High-true pulses
}


//----------------------------------------------------------------------------

void initMotor(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //supply power to PORTE 
	
	PORTE->PCR[FRONT_LEFT_MOTOR] &= ~PORT_PCR_MUX_MASK; //clear MUX 
	PORTE->PCR[FRONT_LEFT_MOTOR] |= PORT_PCR_MUX(3); //setting PortE pin30 to be PWN signal (TPM0_CH0)
	
	PORTE->PCR[FRONT_RIGHT_MOTOR] &= ~PORT_PCR_MUX_MASK; //clear MUX 
	PORTB->PCR[FRONT_RIGHT_MOTOR] |= PORT_PCR_MUX(3); //setting PortE pin29 to be PWN signal (TPM0_CH0)
	
	PORTE->PCR[BACK_LEFT_MOTOR] &= ~PORT_PCR_MUX_MASK; //clear MUX 
	PORTE->PCR[BACK_LEFT_MOTOR] |= PORT_PCR_MUX(3); //setting PortE pin23 to be PWN signal (TPM2_CH0)
	
	PORTE->PCR[BACK_RIGHT_MOTOR] &= ~PORT_PCR_MUX_MASK; //clear MUX 
	PORTE->PCR[BACK_RIGHT_MOTOR] |= PORT_PCR_MUX(3); //setting PortE pin22 to be PWN signal (TPM2_CH0)
	
	PTE->PDDR |= (MASK(FRONT_LEFT_MOTOR) | MASK(FRONT_RIGHT_MOTOR) | MASK(BACK_LEFT_MOTOR) | MASK(BACK_RIGHT_MOTOR));
	
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; //enable clock gate for TPM2 module
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; //enable clock gate for TPM0 module
	
	TPM0->MOD = calc_MOD(50, PRESCALAR);
	TPM0_C0V = calc_MOD(50, PRESCALAR);
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //clear PS(prescalar) and CMOD(counter mode) bits
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //PS7 is set prescalar to 128, CMOD set LPTPM counter to increase on every LPTPM counter clock
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //clear CPWMS bit = LPTPM counter set to up-counting mode
	
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //clear ELSB ELSA MSB MSA bits (M - mode, EL - Edge/Level)
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Mode = Edge-aligned PWM, Configuration = High-true pulses
	
	TPM2->MOD = calc_MOD(50, PRESCALAR);
	TPM2_C0V = calc_MOD(50, PRESCALAR);
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //clear PS(prescalar) and CMOD(counter mode) bits
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); //PS7 is set prescalar to 128, CMOD set LPTPM counter to increase on every LPTPM counter clock
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK); //clear CPWMS bit = LPTPM counter set to up-counting mode
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //clear ELSB ELSA MSB MSA bits (M - mode, EL - Edge/Level)
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // Mode = Edge-aligned PWM, Configuration = High-true pulses
}


void stop_moving() {
	
}

void move_up() {
	PTE->PDDR |= (MASK(FRONT_LEFT_MOTOR) | MASK(FRONT_RIGHT_MOTOR) | MASK(BACK_LEFT_MOTOR) | MASK(BACK_RIGHT_MOTOR));
	osDelay(500);
	PTE->PDDR &= ~(MASK(FRONT_LEFT_MOTOR) | MASK(FRONT_RIGHT_MOTOR) | MASK(BACK_LEFT_MOTOR) | MASK(BACK_RIGHT_MOTOR));
}

void move_down() {
	
}

void move_left() {
	
}

void move_right() {
	
}
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
	for (;;) {		
		/* Rx and Tx */
		
		if (rx_data == DATA_BLUETOOTH_SUCCESS) {
			is_connected = true;
		//delay(0x80000);
		} else if (rx_data == DATA_BLUETOOTH_DISCONNECT) {
			is_connected = false;
		} else if (rx_data == DATA_STATIONARY) {
			//curr_motion_state = stationary;
		} else if (rx_data == DATA_UP) {
			curr_motion_state = up;
		} else if (rx_data == DATA_DOWN) {
			curr_motion_state = down;
		} else if (rx_data == DATA_LEFT) {
			curr_motion_state = left;
		} else if (rx_data == DATA_RIGHT) {
			curr_motion_state = right;
		} else if (rx_data == DATA_END) {
			has_completed = true;
		}
	}  
}

void tMotorControl (void *argument) {
	for (;;) {
		if (curr_motion_state == stationary) {
			stop_moving();
		} else if (curr_motion_state == up) {
			move_up();
		} else if (curr_motion_state == down) {
			move_down();
		} else if (curr_motion_state == left) {
			move_left();
		} else if (curr_motion_state == right) {
			move_right();
		}
	}
}

void tGreenLED (void *argument) {
	for (;;) {
		if (curr_motion_state == stationary) {
			all_green_led_on();
		} else if (curr_motion_state != stationary) {
			green_led_flashing();
		}
	}
}

void tRedLED (void *argument) {
	for (;;) {
		if (curr_motion_state == stationary) {
			all_red_led_flashing_stationary();
		} else if (curr_motion_state != stationary) {
			all_red_led_flashing_running(); 
		}
	}
}

void tAudio (void *argument) {
	for (;;) {
		if (is_connected == true) {
			if (has_started == false) {
				play_start_sound();
				has_started = true;
			}
			play_constant_song();			
		} 
		if (is_connected == false) {
			has_started = false;
		}
	}
}
 
int main (void) {
 
  // System Initialization
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initLED();
	initPWM();
	initMotor();

  // ...
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(tBrain, NULL, NULL);    // Create application main thread
	osThreadNew(tMotorControl, NULL, NULL);    
	osThreadNew(tGreenLED, NULL, NULL);    
	osThreadNew(tRedLED, NULL, NULL);    
	osThreadNew(tAudio, NULL, NULL);    
  osKernelStart();                      // Start thread execution
	
  for (;;) {}
}
