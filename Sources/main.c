/* p4_4.c UART2 echo
 * This program receives a character from UART2 receiver
 * then sends it back through UART2.
 * The bus clock is set to 13.98 MHz in SystemInit().
 * Baud rate = bus clock / BDH:BDL / 16 = 9600
 * A terminal emulator (TeraTerm) should be launched
 * on the host PC. Typing on the PC keyboard sends
 * characters to the FRDM board. The FRDM board echoes
 * the character back to the terminal emulator.
 * The UART2 transmit line is connected to PTD5.
 * The UART2 receive line is connected to PTD4.
 */
#include "derivative.h" /* include peripheral declarations */
#include "I2C.h"
#include "MMA8451Q.h"
#include <string.h>

typedef int bool;

#define true 1
#define false 0
#define BUFFER_SIZE 100
#define CYCLES_SEC 500000
#define	LED_RED_ON	 	GPIOB_PCOR = (1 << 18);
#define	LED_RED_OFF		GPIOB_PSOR |= (1 << 18);
#define	LED_BLUE_ON    	GPIOD_PCOR = (1 << 1);
#define	LED_BLUE_OFF  	GPIOD_PSOR |= (1 << 1);
#define	LED_PURPLE_ON  	LED_RED_ON; LED_BLUE_ON;
#define	LED_PURPLE_OFF	LED_RED_OFF; LED_BLUE_OFF;

#define STARTUP 0u 
#define IDLE 1u 
#define SIGNAL 2u
#define CALL	3u

bool response_ready = false;
bool command_sent = false;
unsigned int state_step = 0u;
unsigned int delay_state_counter = 0u;
char expectedResponse[BUFFER_SIZE] = "";
unsigned int state = STARTUP;


unsigned char PULSE_SRC_val = 0;
char sendBuffer[BUFFER_SIZE] = "";
int sendBufferSize = 0;
int sendBufferPos = 0;

char recvBuffer[BUFFER_SIZE] = "";
int recvBufferSize = 0;
int recvBufferPos = 0;

// Interrupt enabling and disabling
static inline void enable_irq(int n) {
	NVIC_ICPR |= 1 << (n - 16);
	NVIC_ISER |= 1 << (n - 16);
}
// TODO:  IRQ disable

static inline void __enable_irq(void) {
	asm volatile ("cpsie i");
}
static inline void __disable_irq(void) {
	asm volatile ("cpsid i");
}

void cleanBuffer(char* buffer, unsigned int buff_size) {
	int i;
	for (i = 0; i < buff_size; i++) {
		buffer[i] = ' ';
	}
}
void UART2_IRQHandler() {
	int status;
	status = UART2_S1;
	// If there is received data, read it into the receive buffer.  If the
	// buffer is full, disable the receive interrupt.
	if ((status & UART_S1_RDRF_MASK) && recvBufferSize != BUFFER_SIZE) {
		recvBuffer[recvBufferPos] = UART2_D;
		recvBufferPos++;
		recvBufferSize++;
		if (strstr(recvBuffer, expectedResponse) != NULL) {
			response_ready = true;
		}
	}
}

void sendAtCommand(char* message, unsigned int len) {
	unsigned int i;
	for (i = 0; i < len; i++) {
		while (!(UART2_S1 & 0x80)) {
		} /* wait for transmit buffer empty */
		UART2_D = message[i]; /* send a char */
	}
}


/* initialize UART2 to transmit and receive at 9600 Baud */
void UART2_init(void) {
	SIM_SCGC4 |= 0x1000; /* enable clock to UART2 */
	UART2_C2 = 0; /* disable UART during configuration */

	//24 000 000/(16*9600) = 156 | 156 to Hex = 009C
	UART2_BDH = 0x00;
	UART2_BDL = 0x9C; /* 9600 Baud */
	UART2_C1 = 0x00; /* normal 8-bit, no parity */
	UART2_C3 = 0x00; /* no fault interrupt */
	UART2_C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK | UART_C2_RIE_MASK; /* enable transmit and receive */
	SIM_SCGC5 |= 0x1000; /* enable clock to PORTD */
	PORTD_PCR5 = 0x300; /* PTD5 for UART2 transmit */
	PORTD_PCR4 = 0x300; /* PTD5 for UART2 receive */

	/*PORTD_PCR4 |= (0|PORT_PCR_ISF_MASK|	// Clear the interrupt flag 
	 PORT_PCR_MUX(0x3)|// PTA14 is configured as GPIO 
	 PORT_PCR_IRQC(0xA));// PTA14 is configured for falling edge interrupts */
	//enable_irq(INT_PORTD);
	enable_irq(INT_UART2);
}

void initRoutine() {
	SIM_COPC = 0x00U;
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; /* Enable clock gate for ports to enable pin routing */
	/* SIM_CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */SIM_CLKDIV1 =
			SIM_CLKDIV1_OUTDIV4(0x01); /* Update system prescalers */
	/* SIM_SOPT2: PLLFLLSEL=0 */SIM_SOPT2 &=
			(uint32_t) ~(uint32_t) (SIM_SOPT2_PLLFLLSEL_MASK); /* Select FLL as a clock source for various peripherals */
	/* SIM_SOPT1: OSC32KSEL=3 */SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03); /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
	/* SIM_SOPT2: TPMSRC=1 */SIM_SOPT2 = (uint32_t) ((SIM_SOPT2
			& (uint32_t) ~(uint32_t) (SIM_SOPT2_TPMSRC(0x02)))| (uint32_t)(
			SIM_SOPT2_TPMSRC(0x01)
	)); /* Set the TPM clock */
	/* Switch to FEI Mode */
	/* MCG_C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
	MCG_C1 = (MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK);
	/* MCG_C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */MCG_C2 =
			0x00U;
	/* MCG_C4: DMX32=1,DRST_DRS=1 */MCG_C4 = (uint8_t) ((MCG_C4
			& (uint8_t) ~(uint8_t) (MCG_C4_DRST_DRS(0x02)))| (uint8_t)(
			MCG_C4_DMX32_MASK |
			MCG_C4_DRST_DRS(0x01)
	));
	/* OSC0_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	OSC0_CR = OSC_CR_ERCLKEN_MASK;
	/* MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0 */MCG_C5 = 0x00U;
	/* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */MCG_C6 = 0x00U;
	while ((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	}
	while ((MCG_S & 0x0CU) != 0x00U) { /* Wait until output of the FLL is selected */
	}
}

/******************************************************************************
 * MCU initialization function
 ******************************************************************************/
void Led_and_Accel_Init(void) {
	//I2C0 module initialization
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;		// Turn on clock to I2C0 module 
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;		// Turn on clock to Port E module 
	PORTE_PCR24 = PORT_PCR_MUX(5);			// PTE24 pin is I2C0 SCL line 
	PORTE_PCR25 = PORT_PCR_MUX(5);			// PTE25 pin is I2C0 SDA line 
	I2C0_F = 0x14; // SDA hold time = 2.125us, SCL start hold time = 4.25us, SCL stop hold time = 5.125us *
	I2C0_C1 = I2C_C1_IICEN_MASK;    		// Enable I2C0 module 

	//Configure the PTA14 pin (connected to the INT1 of the MMA8451Q) for falling edge interrupts
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Turn on clock to Port A module 
	PORTA_PCR14 |= (0|PORT_PCR_ISF_MASK|	// Clear the interrupt flag 
			PORT_PCR_MUX(0x1)|// PTA14 is configured as GPIO 
			PORT_PCR_IRQC(0xA));// PTA14 is configured for falling edge interrupts 

	//Enable PORTA interrupt on NVIC
	NVIC_ICPR |= 1 << ((INT_PORTA - 16) % 32);
	NVIC_ISER |= 1 << ((INT_PORTA - 16) % 32);

	//Configure PTB18, PTB19 and PTD1 as output for the RGB LED
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;		// Turn on clock to Port B module
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;		// Turn on clock to Port D module	

	PORTB_PCR18 |= PORT_PCR_MUX(0x1);	    // PTB18 is configured as GPIO
	PORTD_PCR1 |= PORT_PCR_MUX(0x1);

	GPIOB_PDDR |= (1 << 18);		//Port Data Direction Register (GPIOx_PDDR)
	GPIOD_PDDR |= (1 << 1);
}

/******************************************************************************
 * Accelerometer initialization function
 ******************************************************************************/
void Accel_Config(void) {
	unsigned char reg_val = 0, CTRL_REG1_val = 0;

	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x40);// Reset all registers to POR values

	do		// Wait for the RST bit to clear 
	{
		reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG2) & 0x40;
	} while (reg_val);

	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x0C);// ODR = 400Hz, Reduced noise, Standby mode	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, XYZ_DATA_CFG_REG, 0x00);	// +/-2g range -> 1g = 16384/4 = 4096 counts 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x02);// High Resolution mode

	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_CFG_REG, 0x15); //Enable X, Y and Z Single Pulse
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSX_REG, 0x10); //Set X Threshold to 2.016g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSY_REG, 0x10); //Set Y Threshold to 2.016g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSZ_REG, 0x10); //Set Z Threshold to 2.646g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_TMLT_REG, 0xB0); //Set Time Limit for Tap Detection to 25 ms
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_LTCY_REG, 0xC0); //Set Latency Time to 50 ms
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x08); //Pulse detection interrupt enabled
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x08); //Route INT1 to system interrupt

	CTRL_REG1_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG1); //Active Mode
	CTRL_REG1_val |= 0x01;
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, CTRL_REG1_val);
}

/******************************************************************************
 * PORT A Interrupt handler
 ******************************************************************************/
void PORTA_IRQHandler() {
	PORTA_PCR14 |= PORT_PCR_ISF_MASK;			// Clear the interrupt flag
	state = CALL;
}


void transitionNextStateStep(unsigned int next_state, unsigned int step) {
	state = next_state;
	command_sent = false;
	response_ready = false;
	state_step = step;
	recvBufferPos = 0;
	recvBufferSize = 0;
	cleanBuffer(recvBuffer, BUFFER_SIZE);
}

void getSignalQualityStateMachine() {
	/* Not waiting for a command response  */
	if (!command_sent) {
		switch (state_step) {
		case 0:
			/* Change LED Color */
			LED_PURPLE_OFF
			LED_RED_ON
			response_ready = false;
			sendAtCommand("AT+CSQ\r\n", strlen("AT+CSQ\r\n"));
			command_sent = true;
			break;
		}
		/*Sent Test AT */

	}
	/* Response available */
	if (response_ready) {
		switch (state_step) {
		/* Send Call AT */
		case 0:
			/*Return to IDLE */
			transitionNextStateStep(IDLE, 0);
			LED_RED_OFF
			LED_PURPLE_ON
			break;

		}
	}
}
void callPhoneStateMachine() {
	/* Not waiting for a command response  */
	if (!command_sent) {
		switch (state_step) {
		case 0:
			/* Change LED Color */
			LED_PURPLE_OFF
			LED_RED_ON
			response_ready = false;
			sendAtCommand("AT\r\n", strlen("AT\r\n"));
			command_sent = true;
			break;
		case 1:
			response_ready = false;
			sendAtCommand("ATD627272355;\r\n", strlen("ATD627272355;\r\n"));
			delay_state_counter = CYCLES_SEC * 10; /*10 secs */
			command_sent = true;
			break;
		case 2:
			response_ready = false;
			sendAtCommand("ATH\r\n", strlen("ATH\r\n"));
			command_sent = true;
			break;
		}
		/*Sent Test AT */

	}
	/* Response available */
	if (response_ready) {
		switch (state_step) {
		/* Send Call AT */
		case 0:
			transitionNextStateStep(CALL, 1);
			break;
		case 1:
			/* Delay */
			if (delay_state_counter == 0) {
				transitionNextStateStep(CALL, 2);
			} else {
				delay_state_counter--;
			}
			break;
		case 2:
			/*Return to IDLE */
			transitionNextStateStep(IDLE, 0);
			LED_RED_OFF
			LED_PURPLE_ON
			break;

		}
	}
}

void startupStateMachine()
{
	/* Not waiting for a command response  */
		if (!command_sent) {
			switch (state_step) {
			case 0:
				/* Change LED Color */
				LED_PURPLE_OFF
				LED_BLUE_ON
				response_ready = false;
				sendAtCommand("ATE0\r\n", strlen("ATE0\r\n"));
				command_sent = true;
				break;
			}
			/*Sent Test AT */

		}
		/* Response available */
		if (response_ready) {
			switch (state_step) {
			/* Send Call AT */
			case 0:
				/*Return to IDLE */
				transitionNextStateStep(IDLE, 0);
				LED_BLUE_OFF
				LED_PURPLE_ON
				break;

			}
		}
}
int main(void) {
	/*Init I2C and LED */
	Led_and_Accel_Init();
	Accel_Config();
	initRoutine();
	UART2_init();
	/* String searched in each response. 
	 * Can be changed for example by > when you have to write a SMS.*/
	strcpy(expectedResponse, "\r\nOK\r\n");
	
	while (1) {
		switch (state) {
		case STARTUP:
			startupStateMachine();
			break;
		case SIGNAL:
			getSignalQualityStateMachine();
			break;
		case CALL:
			callPhoneStateMachine();
			break;
		case IDLE:
		default:
			/*Do nothing*/
			break;
		}
	}
	return 0;
}

