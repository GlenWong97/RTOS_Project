/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
#include "MKL25Z4.h"                    // Device header
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include <stdbool.h>
 
#define DATA_BLUETOOTH_SUCCESS 1
#define DATA_ALL_LED_OFF 2
#define DATA_RED_LED_ON 3
#define DATA_RED_LED_OFF 4
#define DATA_BLUE_LED_ON 5
#define DATA_BLUE_LED_OFF 6
#define DATA_GREEN_LED_ON 7
#define DATA_GREEN_LED_OFF 8

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
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO PRESCALAR

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define MASK(x) (1 << (x))
#define SW_POS 6

enum color_t{red, green, blue};
enum led_state{led_on, led_off};
volatile uint8_t rx_data;

volatile bool is_connected = false;
volatile bool has_started = false;

/* Init UART2 */
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //supply power to UART2 module
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //supply power to PORTE 
	
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK; //clear MUX bits for PORT E pin 22
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4); //set PORT E pin 22 to use UART2_TX module
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK; //clear MUX bits for PORT E pin 23
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4); //set PORT E pin 23 to use UART2_RX module
	
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

/* Delay routine */
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
		nof--;
	}
}

/*-------- LED CODE -----------------------------------------------------------
------------------------------------------------------------------------------*/

void initLED(void) {
	
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

void on_red() {
	PTB->PCOR = MASK(RED_LED);
}

void on_blue() {
	PTD->PCOR = MASK(BLUE_LED);
}

void on_green() {
	PTB->PCOR = MASK(GREEN_LED);
}

void off_red() {
	PTB->PSOR = MASK(RED_LED);
}

void off_blue() {
	PTD->PSOR = MASK(BLUE_LED);
}

void off_green() {
	PTB->PSOR = MASK(GREEN_LED);
}

void led_control(enum color_t color, enum led_state state) {
	if (color == red) {
		if (state == led_on) {
			on_red();
		} else {
			off_red();
		}
	} else if (color == green) {
		if (state == led_on) {
			on_green();
		} else {
			off_green();
		}
		
	} else if (color == blue) {
		if (state == led_on) {
			on_blue();
		} else {
			off_blue();
		}
	}
}

void offRGB() {
	off_red();
	off_blue();
	off_green();
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


 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
	for (;;) {		
		/* Rx and Tx */
		if (rx_data == DATA_RED_LED_ON) {
			led_control(red, led_on);
		} else if (rx_data == DATA_ALL_LED_OFF) {
			offRGB();
		} else if (rx_data == DATA_RED_LED_OFF) {
			led_control(red, led_off);
		} else if (rx_data == DATA_BLUETOOTH_SUCCESS) {
			
		//delay(0x80000);
		}
	}  
}

void tMotorControl (void *argument) {
	
}

void tLED (void *argument) {
	
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
	offRGB();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(tBrain, NULL, NULL);    // Create application main thread
	osThreadNew(tMotorControl, NULL, NULL);    
	osThreadNew(tLED, NULL, NULL);    
	osThreadNew(tAudio, NULL, NULL);    
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
