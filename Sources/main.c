/******************************************************************************
 * Embedded System Program made as an assignment for Microprocessors and
 * Peripherals subject.
 * 
 * The system baptized as PurpleBox TCU is programmed as a state machine.
 * 
 * The components used for the project are a SIM808 development platform and
 * the NXP FRDM-KL25z development platform.
 * 
 * PurpleBox TCU sends the GPS Location every x seconds and calls a predefined
 * phone number if an abrupt movement is detected.
 * 
 * This project was created by Steven Macías and  Lorenzo Hidalgo,
 * students at the Autonomous University of Barcelona, and is available at
 * the following repository: https://github.com/StevenMacias/PurpleBoxTCU
 * 
 * For more information visit www.purplebox.tk 
 ******************************************************************************/

#include "derivative.h" /* include peripheral declarations */
#include "I2C.h"
#include "MMA8451Q.h"
#include <string.h>

typedef int bool;

/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
#define true 			1
#define false 			0
#define BUFFER_SIZE 	255
#define RECV_BUFFER_SIZE 2048
#define COORD_SIZE		16
#define CYCLES_SEC 		500000


/******************************************************************************
 * LED CONFIGURATION MACROS
 ******************************************************************************/
//RGB MACROS
#define	LED_RED_ON	 	GPIOB_PCOR = (1 << 18);
#define	LED_RED_OFF		GPIOB_PSOR |= (1 << 18);
#define	LED_GREEN_ON	GPIOB_PCOR = (1 << 19);
#define	LED_GREEN_OFF	GPIOB_PSOR |= (1 << 19);
#define	LED_BLUE_ON    	GPIOD_PCOR = (1 << 1);
#define	LED_BLUE_OFF  	GPIOD_PSOR |= (1 << 1);

//COMBINED COLORS MACROS
#define LED_YELLOW_ON	LED_RED_ON; LED_GREEN_ON;
#define LED_YELLOW_OFF	LED_RED_OFF; LED_GREEN_OFF;
#define	LED_PURPLE_ON  	LED_RED_ON; LED_BLUE_ON;
#define	LED_PURPLE_OFF	LED_RED_OFF; LED_BLUE_OFF;
#define LED_CIAN_ON		LED_GREEN_ON; LED_BLUE_ON;
#define LED_CIAN_OFF	LED_GREEN_OFF; LED_BLUE_OFF;
#define LED_WHITE_ON	LED_RED_ON; LED_GREEN_ON; LED_BLUE_ON;
#define LED_WHITE_OFF	LED_RED_OFF; LED_GREEN_OFF; LED_BLUE_OFF;

//LED ALL COLORS OFF
#define LED_ALL_OFF		LED_RED_OFF; LED_GREEN_OFF; LED_BLUE_OFF;


/******************************************************************************
 * SM GLOBAL VARIABLES
 ******************************************************************************/
//SM STATES
#define STARTUP 	0u 
#define IDLE 		1u 
#define SIGNAL 		2u
#define CALL		3u
#define GPS			4u
#define HTTP_SEND	5u

//SM VARIABLES
bool response_ready = false;
bool command_sent = false;
unsigned int state_step = 0u;
unsigned int delay_state_counter = 0u;
unsigned int gps_state_counter = CYCLES_SEC * 5;
char expectedResponse[BUFFER_SIZE] = "";
unsigned int state = STARTUP;


/******************************************************************************
 * COMMUNICATION BUFFERS
 ******************************************************************************/
//SEND BUFFER
unsigned char PULSE_SRC_val = 0;
char sendBuffer[BUFFER_SIZE] = "";
int sendBufferSize = 0;
int sendBufferPos = 0;

//RECIEVE BUFFER
char recvBuffer[RECV_BUFFER_SIZE] = "";
int recvBufferSize = 0;
int recvBufferPos = 0;

//JSON BUFFER FOR HTTP REQUEST
char jsonBuffer[BUFFER_SIZE] = ""; 
int jsonBuffer_size = 0;

//GPS GLOBAL VARIABLES
bool new_gps_data = false;
char *token;
char latitude[COORD_SIZE] = "", longitude[COORD_SIZE] = "";


/******************************************************************************
 * INTERRUPTION HANDLERS
 ******************************************************************************/
// INTERRUPTION ENABLE AND DISABLE
static inline void enable_irq(int n) {
	NVIC_ICPR |= 1 << (n - 16);
	NVIC_ISER |= 1 << (n - 16);
}

//ENABLE INTERRUPTIONS
static inline void __enable_irq(void) {
	asm volatile ("cpsie i");
}

//DISABLE INTERRUPTIONS
static inline void __disable_irq(void) {
	asm volatile ("cpsid i");
}


/******************************************************************************
 * FUNCTION TO SET ALL POSITIONS OF A BUFFER TO \0
 ******************************************************************************/
void cleanBuffer(char* buffer, unsigned int buff_size) {
	int i;
	for (i = 0; i < buff_size; i++) {
		buffer[i] = '\0';
	}
}


/******************************************************************************
 * UART2 INTERRUPTION HANDLER
 ******************************************************************************/
void UART2_IRQHandler() {
	int status;
	status = UART2_S1;
	// If there is received data, read it into the receive buffer.  If the
	// buffer is full, disable the receive interrupt.
	if ((status & UART_S1_RDRF_MASK) && recvBufferSize != RECV_BUFFER_SIZE) {
		recvBuffer[recvBufferPos] = UART2_D;
		recvBufferPos++;
		recvBufferSize++;

		if (strstr(recvBuffer, expectedResponse) != NULL) {
			response_ready = true;
		}
	}
}


/******************************************************************************
 * FUNCTION TO SEND AT COMMANDS VIA UART2
 ******************************************************************************/
void sendAtCommand(char* message, unsigned int len) {
	unsigned int i;
	strcpy(sendBuffer, message);
	for (i = 0; i < len; i++) {
		while (!(UART2_S1 & 0x80)) {
		} /* wait for transmit buffer empty */
		UART2_D = message[i]; /* send a char */
	}
}


/******************************************************************************
 * UART2 INITIALIZATION FUNCTION - 9600 BAUD
 ******************************************************************************/
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

	//enable_irq(INT_PORTD);
	enable_irq(INT_UART2);
}


/******************************************************************************
 * FUNCTION TO INITIALIZE THE SIM808 DEVELOPMENT PLATFORM
 ******************************************************************************/
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
 * FUNCTION TO INITIALIZE RGB LED AND ACCELEROMETER
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
	PORTB_PCR19 |= PORT_PCR_MUX(0x1);
	PORTD_PCR1 |= PORT_PCR_MUX(0x1);

	GPIOB_PDDR |= (1 << 18);		//Port Data Direction Register (GPIOx_PDDR)
	GPIOB_PDDR |= (1 << 19);
	GPIOD_PDDR |= (1 << 1);

	LED_ALL_OFF
	;
}


/******************************************************************************
 * PORT A INTERRUPTION HANDLER
 ******************************************************************************/
void PORTA_IRQHandler() {
	PORTA_PCR14 |= PORT_PCR_ISF_MASK;			// Clear the interrupt flag
	state = CALL;
}


/******************************************************************************
 * FUNCTION TO CONFIGURE THE MMA845x ACCELEROMETER
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
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSX_REG, 0x60); //Set X Threshold to 2.016g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSY_REG, 0x60); //Set Y Threshold to 2.016g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_THSZ_REG, 0x60); //Set Z Threshold to 2.646g
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_TMLT_REG, 0xB0); //Set Time Limit for Tap Detection to 25 ms
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, PULSE_LTCY_REG, 0xC0); //Set Latency Time to 50 ms
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x08); //Pulse detection interrupt enabled
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x08); //Route INT1 to system interrupt

	CTRL_REG1_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG1); //Active Mode
	CTRL_REG1_val |= 0x01;
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, CTRL_REG1_val);
}


/******************************************************************************
 * SM STATE TRANSITION FUNCTION
 ******************************************************************************/
void transitionNextStateStep(unsigned int next_state, unsigned int step) {
	state = next_state;
	command_sent = false;
	response_ready = false;
	state_step = step;
	recvBufferPos = 0;
	recvBufferSize = 0;
	cleanBuffer(recvBuffer, RECV_BUFFER_SIZE);
	strcpy(expectedResponse, "\r\nOK\r\n");
}


/******************************************************************************
 * SM STATE - SIGNAL QUALITY HANDLER
 ******************************************************************************/
void getSignalQualityStateMachine() {
	// ENTERS IF THE SM ISN'T WAITING FOR A RESPONSE
	if (!command_sent) {
		switch (state_step) {
		case 0:
			// SIGNAL QUALITY REQUEST
			LED_PURPLE_OFF
			LED_RED_ON
			response_ready = false;
			sendAtCommand("AT+CSQ\r\n", strlen("AT+CSQ\r\n"));
			command_sent = true;
			break;
		}
	}
	
	// ENTERS IF A RESPONSE IS AVAILABLE - UART2 INTERRUPTION
	if (response_ready) {
		switch (state_step) {
		// TRANSITION HANDLER BETWEEN SM FUNCTIONS
		case 0:
			// RETURNS TO IDLE STATE - LED PURPLE
			transitionNextStateStep(IDLE, 0);
			LED_RED_OFF
			LED_PURPLE_ON
			break;

		}
	}
}


/******************************************************************************
 * SM STATE - GPS SIGNAL HANDLING - LED YELLOW
 ******************************************************************************/
void GPSrequestStateMachine() {
	// ENTERS IF THE SM ISN'T WAITING FOR A RESPONSE
	if (!command_sent) {
		switch (state_step) {
		case 0:
			// GPS POSITION REQUEST - LED COLOR YELLOW
			LED_ALL_OFF
			LED_YELLOW_ON
			response_ready = false;
			sendAtCommand("AT+CGNSINF\r\n", strlen("AT+CGNSINF\r\n"));
			command_sent = true;
			break;
		}
	}
	
	// ENTERS IF A RESPONSE IS AVAILABLE - UART2 INTERRUPTION
	if (response_ready) {
		switch (state_step) {
		// TRANSITION HANDLER BETWEEN SM FUNCTIONS
		case 0:
			token = strtok(recvBuffer, ",");
			token = strtok(NULL, ",");

			if (atoi(token)) {
				// POSITION FIXED - SEND HTTP REQ
				token = strtok(NULL, ",");
				token = strtok(NULL, ",");
				strcpy(latitude, token);
				token = strtok(NULL, ",");
				strcpy(longitude, token);
				new_gps_data = true;
				transitionNextStateStep(HTTP_SEND, 0);
			} else {
				// POSITION NOT FIXED - BACK TO IDLE
				transitionNextStateStep(IDLE, 0);
			}
			// LED COLOR PURPLE
			LED_ALL_OFF
			LED_PURPLE_ON
			break;

		}
	}
}

/******************************************************************************
 * SM STATE - HTTP COMMUNICATION HANDLING - LED CYAN
 ******************************************************************************/
void HTTPrequestStateMachine() {
	// ENTERS IF THE SM ISN'T WAITING FOR A RESPONSE
	if (!command_sent) {
		switch (state_step) {
		case 0:
			// HTTP REQUEST PARAMETERS - CID - LED COLOR CYAN
			LED_ALL_OFF
			LED_CIAN_ON
			response_ready = false;
			sendAtCommand("AT+HTTPPARA=\"CID\",1\r\n",
					strlen("AT+HTTPPARA=\"CID\",1\r\n"));
			command_sent = true;
			break;
		case 1:
			// HTTP REQUEST PARAMETERS - URL
			response_ready = false;
			sendAtCommand(
					"AT+HTTPPARA=\"URL\",\"http://purplebox.000webhostapp.com/saveGpsFrame\"\r\n",
					strlen(
							"AT+HTTPPARA=\"URL\",\"http://purplebox.000webhostapp.com/saveGpsFrame\"\r\n"));
			command_sent = true;
			break;
		case 2:
			// HTTP REQUEST PARAMETERS - CONTENT
			response_ready = false;
			sendAtCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n",
					strlen("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n"));
			command_sent = true;
			break;
		case 3:
			// PREPARES THE JSON BUFFER
			response_ready = false;
			cleanBuffer(sendBuffer, BUFFER_SIZE);
			cleanBuffer(jsonBuffer, BUFFER_SIZE);
			sprintf(jsonBuffer, "{\"latitude\":%s,\"longitude\":%s}", latitude, longitude);
			jsonBuffer_size = strlen(jsonBuffer);
			sprintf(sendBuffer, "AT+HTTPDATA=%d,10000\r\n",
					jsonBuffer_size);
			sendAtCommand(sendBuffer, strlen(sendBuffer));
			command_sent = true;
			strcpy(expectedResponse, "DOWNLOAD");
			break;
		case 4:
			// SENDS THE JSON BUFFER
			response_ready = false;
			sendAtCommand(jsonBuffer, jsonBuffer_size);
			command_sent = true;
			strcpy(expectedResponse, "\r\nOK\r\n");
			break;
		case 5:
			// SENDS HTTP REQUEST
			response_ready = false;
			sendAtCommand("AT+HTTPACTION=1\r\n", strlen("AT+HTTPACTION=1\r\n"));
			command_sent = true;
			strcpy(expectedResponse, "\r\nOK\r\n\r\n+HTTPACTION: 1,");
			break;
		case 6:
			// READS HTTP RESPONSE
			sendAtCommand("AT+HTTPREAD\r\n", strlen("AT+HTTPREAD\r\n"));
			command_sent = true;
			strcpy(expectedResponse, "\r\nOK\r\n");
			break;
		}
	}
	
	// ENTERS IF A RESPONSE IS AVAILABLE - UART2 INTERRUPTION
	if (response_ready) {
		switch (state_step) {
		// TRANSITION HANDLER BETWEEN SM FUNCTIONS
		case 0:
			transitionNextStateStep(HTTP_SEND, 1);
			break;
		case 1:
			transitionNextStateStep(HTTP_SEND, 2);
			break;
		case 2:
			transitionNextStateStep(HTTP_SEND, 3);
			break;
		case 3:
			transitionNextStateStep(HTTP_SEND, 4);
			break;
		case 4:
			transitionNextStateStep(HTTP_SEND, 5);
			break;
		case 5:
			transitionNextStateStep(HTTP_SEND, 6);
			break;
		case 6:
			// RETURNS TO IDLE STATE - LED PURPLE
			transitionNextStateStep(IDLE, 0);
			LED_ALL_OFF
			LED_PURPLE_ON
			break;

		}
	}
}


/******************************************************************************
 * SM STATE - CELLPHONE CALL HANDLING - LED RED
 ******************************************************************************/
void callPhoneStateMachine() {
	// ENTERS IF THE SM ISN'T WAITING FOR A RESPONSE
	if (!command_sent) {
		switch (state_step) {
		case 0:
			// CHECKS SIM808 AVAILABILITY - LED COLOR RED
			LED_PURPLE_OFF
			LED_RED_ON
			response_ready = false;
			sendAtCommand("AT\r\n", strlen("AT\r\n"));
			command_sent = true;
			break;
		case 1:
			// CALLS A NUMBER AND WAITS FOR 10 SECONDS
			response_ready = false;
			sendAtCommand("ATDXXXXXXXXX;\r\n", strlen("ATDXXXXXXXXX;\r\n"));
			delay_state_counter = CYCLES_SEC * 10; /*10 secs */
			command_sent = true;
			break;
		case 2:
			// HANG UP THE CALL
			response_ready = false;
			sendAtCommand("ATH\r\n", strlen("ATH\r\n"));
			command_sent = true;
			break;
		}
	}
	
	// ENTERS IF A RESPONSE IS AVAILABLE - UART2 INTERRUPTION
	if (response_ready) {
		switch (state_step) {
		// TRANSITION HANDLER BETWEEN SM FUNCTIONS
		case 0:
			transitionNextStateStep(CALL, 1);
			break;
		case 1:
			// DELAY HANDLER
			if (delay_state_counter == 0) {
				transitionNextStateStep(CALL, 2);
			} else {
				delay_state_counter--;
			}
			break;
		case 2:
			// RETURNS TO IDLE STATE - LED PURPLE
			transitionNextStateStep(IDLE, 0);
			LED_RED_OFF
			LED_PURPLE_ON
			break;

		}
	}
}


/******************************************************************************
 * SM STATE - STARTUP HANDLING - LED BLUE
 ******************************************************************************/
void startupStateMachine() {
	// ENTERS IF THE SM ISN'T WAITING FOR A RESPONSE
	if (!command_sent) {
		switch (state_step) {
		case 0:
			// SET LED COLOR TO BLUE
			LED_ALL_OFF
			LED_BLUE_ON
			response_ready = false;
			sendAtCommand("ATE0\r\n", strlen("ATE0\r\n"));
			command_sent = true;
			break;
		case 1:
			// GNS POWERING
			response_ready = false;
			sendAtCommand("AT+CGNSPWR=1\r\n", strlen("AT+CGNSPWR=1\r\n"));
			command_sent = true;
			break;
		case 2:
			// BEARER SETTINGS FOR IP BASED APPLICATIONS
			response_ready = false;
			sendAtCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n",
					strlen("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n"));
			command_sent = true;
			break;
		case 3:
			// BEARER SETTINGS FOR IP BASED APPLICATIONS
			response_ready = false;
			sendAtCommand("AT+SAPBR=3,1,\"APN\",\"orangeworld\"\r\n",
					strlen("AT+SAPBR=3,1,\"APN\",\"orangeworld\"\r\n"));
			command_sent = true;
			break;
		case 4:
			// BEARER SETTINGS FOR IP BASED APPLICATIONS
			response_ready = false;
			sendAtCommand("AT+SAPBR=1,1\r\n", strlen("AT+SAPBR=1,1\r\n"));
			command_sent = true;
			break;
		case 5:
			// BEARER SETTINGS FOR IP BASED APPLICATIONS
			response_ready = false;
			sendAtCommand("AT+SAPBR=2,1\r\n", strlen("AT+SAPBR=2,1\r\n"));
			command_sent = true;
			break;
		case 6:
			// HTTP MODULE INITIALIZATION
			response_ready = false;
			sendAtCommand("AT+HTTPINIT\r\n", strlen("AT+HTTPINIT\r\n"));
			command_sent = true;
			break;

		}

	}
	
	// ENTERS IF A RESPONSE IS AVAILABLE - UART2 INTERRUPTION
	if (response_ready) {
		switch (state_step) {
		// TRANSITION HANDLER BETWEEN SM FUNCTIONS
		case 0:
			transitionNextStateStep(STARTUP, 1);
			break;
		case 1:
			transitionNextStateStep(STARTUP, 2);
			break;
		case 2:
			transitionNextStateStep(STARTUP, 3);
			break;
		case 3:
			transitionNextStateStep(STARTUP, 4);
			break;
		case 4:
			transitionNextStateStep(STARTUP, 5);
			break;
		case 5:
			transitionNextStateStep(STARTUP, 6);
			break;
		case 6:
			// RETURNS TO IDLE STATE - LED PURPLE
			transitionNextStateStep(IDLE, 0);
			LED_ALL_OFF
			LED_PURPLE_ON
			break;

		}
	}
}


/******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
int main(void) {
	// COMPONENT INITIALIZATIONS
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
			// SM STARTUP PROCEDURE
			startupStateMachine();
			break;
		case SIGNAL:
			// SIGNAL QUALITY 
			getSignalQualityStateMachine();
			break;
		case CALL:
			// CALL
			callPhoneStateMachine();
			break;
		case GPS:
			// REQUEST GPS POSITION
			GPSrequestStateMachine();
			break;
		case HTTP_SEND:
			// SEND HTTP REQUEST
			HTTPrequestStateMachine();
			break;
		case IDLE:
			// COUNTDOWN FOR NEW GPS REQUEST
			if (gps_state_counter == 0) {
				transitionNextStateStep(GPS, 0);
				gps_state_counter = CYCLES_SEC * 5;
			} else {
				gps_state_counter--;
			}
		default:
			break;
		}
	}
	return 0;
}

