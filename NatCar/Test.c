#include "MKL25Z4.h"
#include "uart.h"
#include "main.h"
#include "adc16.h"
#include "timers.h"


char strNewLine[] = "\r\n";

float servoPConstant = 22;
float servoIConstant;
float servoDConstant = 6;
unsigned short PWCenter = 4500; 
unsigned short tempPW = 0;

volatile float midpointError;

//Buffers
volatile uint8_t bufferPing1[128]; 
//volatile uint8_t prevbufferPing1[128];

volatile uint8_t bufferPong1[128];
//volatile uint8_t prevbufferPong1[128];

//Counters
volatile uint8_t i = 0; 		//counter for buffers
volatile uint8_t counterCLK = 0;			//counter for the CLK

//Flags
volatile uint8_t flagDone = 0; 				//flag for leaving the infinite loop in Main
volatile uint8_t flagBuffer = 0; 			//sorts which buffer is active/passive, determines when to swap Buffers
volatile uint8_t flagSwap = 0;				//flag for switching buffers to analyze

//PW
unsigned short PW1 = 0; 
unsigned short PW2 = 0; 
volatile unsigned short PW3 = 4500; //Neutral Position of Servo

const uint32_t led_mask[] = {1UL << 18, 1UL << 19, 1UL << 1, 1UL << 0};

volatile float prevMidPoint = 63.5; 
volatile uint8_t black = 0;

void put(char *ptr_str)
{
	while(*ptr_str)
	uart0_putchar(*ptr_str++);
}

void init_ADC0(void);

void LED_Initialize(void) {
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
		| SIM_SCGC5_PORTB_MASK
		| SIM_SCGC5_PORTC_MASK
		| SIM_SCGC5_PORTD_MASK
		| SIM_SCGC5_PORTE_MASK ); //Enables Clock to Port A, B, C, D, and E
	
	
	 /*UART*/
	PORTB->PCR[0] = (1UL << 8);
	FPTB->PDDR |= (1UL << 0);
	
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	
	PORTA->PCR[1] = PORT_PCR_MUX(0x2);		// Enable the UART0_RX function on PTA1
	PORTA->PCR[2] = PORT_PCR_MUX(0x2);		// Enable the UART0_TX function on PTA2
	
	/*UART END*/
	
	// LEDS Red, Blue, Green
	PORTB->PCR[18] = (1UL << 8); 			// Pin PTB18 is GPIO
  	PORTB->PCR[19] = (1UL <<  8);                   /* Pin PTB19 is GPIO */
  	FPTB->PDOR  = ((1UL << 18) | (1UL << 19));	/* switch Red/Green LED off  */
  	FPTB->PDDR |= ((1UL << 18) | (1UL << 19));	/* enable PTB18/19 as Output */
	 
	FPTB->PDOR  = ( led_mask[0] | led_mask[1] );			/* switch Red/Green LED off  */
  	FPTB->PDDR |= ( led_mask[0] | led_mask[1] | 1UL << 0 );		/* enable PTB18/19 as Output */

	
	// H-BRIDGE 
	PORTC->PCR[4] = (1UL << 8);			// H-BRIDGE_A-IN2 is GPIO
	PORTC->PCR[2] = (1UL << 8);			// H-BRIDGE_B-IN2 is GPIO
	PORTE->PCR[20] = (1UL << 8);			// H-BRIDGE-FAULT is GPIO
	PORTE -> PCR[21] = (1UL << 8);			// set H-BRIDGE_ENABLE as GPIO

	FPTC->PDDR |= (1UL << 2);			// SET PTC2 OUTPUT
	FPTC->PDDR |= (1UL << 4);			// SET PTC4 OUTPUT
	FPTE->PDDR |= (1UL << 21);			// SET PTE21 OUTPUT
		
	FPTE -> PCOR = (1UL << 21); 			// SET PTE 21 LOW ( H-BRIDGE_ENABLE) Disabled
	
	FPTC -> PCOR = (1UL << 4);			// TURNS PTC4 A-IN2 LOW, LEFT WHEEL 
	
	FPTC -> PCOR = (1UL << 2); 			// TURNS PTC2 B-IN2 IS LOW, RIGHT WHEEL
	
	// Enable AO, SI, CLK
		PORTD->PCR[5] = (1UL << 8);
		PORTD->PCR[7] = (1UL << 8);
		PORTE->PCR[1] = (1UL << 8);
		
		FPTD->PDDR |= (1UL << 7);
		FPTE->PDDR |= (1UL << 1);
		
	// SWITCHES AND POTS
	//PORTC->PCR[13] = (1UL << 8); // SW1 IS GPIO
	//PORTC->PCR[17] = (1UL <<8); // SW2 IS GPIO
}


void printBuffer(uint8_t * a)
{
	
	int k;
	char tempchar[] = "a ";
	
	for (k = 0; k < 128 ; k++) 
	{
		tempchar[0] = a[k] + '0';
		put(tempchar); 
	} 
	put(strNewLine);
}

void getDir(uint8_t * a)
{
	uint8_t b;
	float steeringPError = 0;
	//float steeringIError = 0;
	float steeringDError = 0;
	
	float midpoint;
	uint8_t leftpoint = 2;
	uint8_t rightpoint = 125;
	
	for( b = 1; b < 126; b++){ //left to right
		if(((a[b + 1] - a[b - 1])/2) > 0.25){
			leftpoint = b;
			black = 0;
			break;
		}
		else{
			black = 1;
		}
	}
	
	for( b = 126; b > 1; b--){ //right to left
		if(((a[b - 1] - a[b + 1])/2) > 0.25){
			rightpoint = b;
			black = 0;
			break;
		}
		else{
			black = 1;
		}
			
	}
	
	if(black == 0){
		midpoint = (leftpoint + rightpoint) / 2;
		//midpoint = (midpoint * 4 / 5) + (prevMidPoint / 5);
		prevMidPoint = midpoint;
	}
	else{
		midpoint = prevMidPoint;
	}
	
	
	//midpoint is less than 63.5 when the line is at the left 
	//midpoint is greater than 63.5 when the line is at the right 
	
		//if steeringDError is positive then the current midpoint is greater than the previous. Then Right Turn
		//if steeringDError is negative then the current midpoint is less than the previous. Then Left Turn
		steeringDError = prevMidPoint - midpoint;
		//steeringPError is negative when midpoint is at the left i.e. right turn
		//steeringPError is postive when midpoint is at the right i.e. left turn
		steeringPError =  midpoint - 63.5;
	
		PW3 = (steeringPError * servoPConstant) + (steeringDError * servoDConstant) + PWCenter;		
	
	if(steeringPError > 20){ //left Turn
	TPM0->CONTROLS[0].CnV = tempPW / ((steeringPError * .03) + 1);
	TPM0->CONTROLS[2].CnV = tempPW * ((steeringPError * .003) + 1);
	}
	else if(steeringPError < -20){ //right Turn
	TPM0->CONTROLS[0].CnV = tempPW * ((steeringPError * .003) + 1);
	TPM0->CONTROLS[2].CnV = tempPW / ((-steeringPError * .03) + 1);
	}
	else{
	TPM0->CONTROLS[0].CnV = tempPW;
	TPM0->CONTROLS[2].CnV = tempPW;
	}
}
/*----------------------------------------------------------------------------
  Interrupts Begin
 *----------------------------------------------------------------------------*/

void TPM1_IRQHandler(void) {
	//clear pending IRQ
	NVIC_ClearPendingIRQ(TPM1_IRQn);
	
	if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK) 		//channel flag  	//clear flag when CnV = PW3
		TPM1->CONTROLS[0].CnSC|= TPM_CnSC_CHF_MASK;

	// modify pulse width for TPM1_CH1
	TPM1->CONTROLS[0].CnV = PW3; 	
}

void ADC0_IRQHandler(void)
{	
	// variable declaration 
    uint16_t tempVal; //temporary value to hold A/D value
    int pseudoDelay = 0;
    
    // Condition 1
    FPTE->PSOR = (1UL << 1); //Assert CLK 
    
    // Condition 2
    tempVal = ADC0->R[0]; //Read A/D Value
    NVIC_ClearPendingIRQ(ADC0_IRQn); // clear interrupt request flag
    
    // Condition 3
	if (flagBuffer == 0)
		{
				bufferPing1[i] = tempVal;
				//bufferPing1[i] = (bufferPong1[i] / 3) + (tempVal * 2 / 3);
		}
	else 
		{
				bufferPong1[i] = tempVal;
				//bufferPong1[i] = (bufferPing1[i] / 3) + (tempVal * 2 / 3);
		} //Store A/D value in specific buffer
			
			
	i++;//update index
	
	// Condition 4
    counterCLK++; // increment CLK counter
  
    
    // Condition 5
        if(counterCLK < 129) // Using  clk cycles for 256 pixels
        {
						for(pseudoDelay = 0; pseudoDelay < 6; pseudoDelay++){}// Accounts for 60ns Delay
							
						ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; // configure adc
						ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6); // start conversion on channel SE6b (PTD5)

        }
        else
        {
            flagDone = 1; // set flags and reset counter/index
            i = 0;
            counterCLK = 0;
            if(flagBuffer == 1)
                            flagBuffer = 0;
            else 
                            flagBuffer = 1;

            FPTB->PTOR = (1UL << 0); //Deassert GPIOsignal for measuring conversion time of the entire line
        }
    
    // Condition 6
	FPTE->PCOR = (1UL << 1); //Deassert CLK
}
	
void PIT_IRQHandler() 
	{
		int pseudoDelay;
                //Condition 1
        FPTD->PSOR = (1UL << 7); // Assert SI Signal

                //Condition 2
        FPTB->PSOR = (1UL << 0); //assert GPIO signal on arbitrary pin 
    
                //Condition 3
        PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;    //CLEAR PIT INTERRUPT REQUEST SIGNAL.
        
                //Delay 
        pseudoDelay = 0; // Accounts for 20ns delay 
    
                //Condition 4
        FPTE->PSOR = (1UL << 1);// Assert CLK signal
    
                //Condition 5
       for(pseudoDelay = 0; pseudoDelay < 6; pseudoDelay++){}// Accounts for 120ns Delay
                
                //Condition 6
        PIT_interrupt_counter++;
        
                //Condition 7
        FPTD->PCOR = (1UL << 7);//Deassert SI signal
				FPTE->PSOR = (1UL << 1);//Deassert CLK 
                
                //Condition 8
        ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
        ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6); //THIS STARTS CONVERSION ON CHANNEL SE6B (PTD5)
                
                //Condition 9
	}


void Init_PWM(void) {

	// Set up the clock source for MCGPLLCLK/2. 
	// See p. 124 and 195-196 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	// TPM clock will be 48.0 MHz if CLOCK_SETUP is 1 in system_MKL25Z4.c.
	
		SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	
	// See p. 207 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
		SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Turn on clock to TPM0
		SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK; // Turn on clock to TPM1


	// See p. 163 and p. 183-184 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	
		PORTC->PCR[1] = PORT_PCR_MUX(4); // Configure PTC1 as TPM0_CH1		
		PORTC->PCR[3] = PORT_PCR_MUX(4); // CONFIGURE PTC3 AS TPM0_CH2
		PORTB->PCR[0] = PORT_PCR_MUX(3); // Configure PTB0 as TPM1_CH0		//changed
	
	// Set channel TPM1_CH0 to edge-aligned, high-true PWM
	
		TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
		TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
		TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_CHIE_MASK;
	
	// Set period and pulse widths
	
		TPM0->MOD = 3000-1;		// Freq. = (48 MHz / 16) / 3000 = 1 kHz  // 60K = 50HZ		
		TPM0->CONTROLS[0].CnV = PW1; 	
		TPM0->CONTROLS[2].CnV = PW2;

	// Set period and pulse widths
	
		TPM1->MOD = 60000;		// Freq. = (48 MHz / 16) / 3000 = 1 kHz  // 60K = 50HZ			//changed
		TPM1->CONTROLS[0].CnV = PW3; 	
		
	// set TPM0 to up-counter, divide by 16 prescaler and clock mode
	
		TPM0->SC = ( TPM_SC_CMOD(1) | TPM_SC_PS(4));
		
	// set TPM1 to up-counter, divide by 16 prescaler and clock mode
	
		TPM1->SC = ( TPM_SC_CMOD(1) | TPM_SC_PS(4));
	
	// clear the overflow mask by writing 1 to CHF

		if (TPM0->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK){
			TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK ;	
		}
		if (TPM0->CONTROLS[2].CnSC & TPM_CnSC_CHF_MASK){
		TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK ;	
		}

	// clear the overflow mask by writing 1 to CHF

		if (TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK)
			TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK ;			//changed
}

/*----------------------------------------------------------------------------
  Interrupts End
 *----------------------------------------------------------------------------*/

void Init_ADC(void) {

		init_ADC0();			// initialize and calibrate ADC0
		ADC0->CFG1 = (ADLPC_LOW | ADIV_1 | ADLSMP_LONG | MODE_8 | ADICLK_BUS_2);
		ADC0->SC2 = 0; // ADTRG=0 (software trigger mode)
		NVIC_SetPriority(ADC0_IRQn, 128);
		NVIC_ClearPendingIRQ(ADC0_IRQn);
		NVIC_EnableIRQ(ADC0_IRQn);
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
	
	//char key; //temporary
	int uart0_clk_khz;
	
	LED_Initialize();           /* Initialize the LEDs */
	uart0_clk_khz = (48000000 / 1000); // UART0 clock frequency will equal half the PLL frequency    ***
          

	uart0_init (uart0_clk_khz, TERMINAL_BAUD);
	Init_ADC();
	Init_PWM();

	Init_PIT(12000);
	
	FPTE-> PSOR = (1UL << 21); // SET PTE21 HIGH ( H_BRIDGE ENABLE) //temporary
	tempPW = 1300; //temporary
	
	Start_PIT();
		
		// Enable Interrupts

	NVIC_SetPriority(TPM1_IRQn, 64); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM1_IRQn); 
	NVIC_EnableIRQ(TPM1_IRQn);	
	
	while(1){
		while (flagDone == 0){} //standy by until buffer is filled
		flagDone =  0; //reset flag
		
		if(flagBuffer == 0){
			getDir((uint8_t *)bufferPong1);
			//printBuffer((uint8_t *)bufferPong1);
		}
		else{
			getDir((uint8_t *)bufferPing1);
			//printBuffer((uint8_t *)bufferPing1);
		}
		 
	}
}//main
