#include <stdio.h>
#include <stdlib.h>
#include <at89lp51rd2.h>
#include <math.h>

// ~C51~ 

#define CURSOR_ON "\x1b[?25h"
#define CURSOR_OFF "\x1b[?25l"
#define CLEAR_SCREEN "\x1b[2J"
#define GOTO_YX "\x1B[%d;%dH"
#define CLR_TO_END_LINE "\x1B[K"
#define BKF_WTB "\x1B[0;30;47m"
#define FORE_BACK "\x1B[0;3%d;4%dm"
#define	COLOR_BLACK		0
#define	COLOR_WHITE		7

// The AT89lp51 soft processor clock is 22.1184 MHz 
#define CLK 22118400L
#define BAUD 115200L
#define BRG_VAL (0x100-(CLK/(32L*BAUD)))

//((1/10000Hz)=100 us)
#define BAUD 115200L
#define FREQ 10000L
#define TIMER0_RELOAD_VALUE (65536L-(CLK/(12L*FREQ)))
#define HCTL_FREQ 10000L
#define TIMER1_RELOAD_VALUE (65536L-(CLK/(12L*FREQ)))
#define MSB_reload_value 0xD4 /* Timer 2 msb reload value exemple */
#define LSB_reload_value 0xCC /* Timer 2 lsb reload value exemple */

//HCTL
#define RST1 P1_1	 //[PCB] P1_1
#define OE1 P3_6	 //[PCB] P3_6 [PROTO} P3_7
#define UD1 P3_2	 //[PCB] P3_2
#define RST2 P1_2	 //[PCB] P1_2
#define OE2 P3_7	 //[PCB] P3_7
#define UD2 P3_3	 //[PCB] P3_3
#define SEL1 P4_3
#define SEL2 P4_4	

// MCP3008
#define CE	  P1_4	 //[PCB] P1_4  [PROTO] P0_3

//Function Prototypes
void waitms (void);
void resetHCTL(int);
unsigned int decode (int);
unsigned int readBits(int);
void SPIWrite(unsigned char);
unsigned int GetADC(unsigned char);
int PIDcalculation(int);
int fivePointMovingAvg(int);

//MOTOR PWMs
#define M1 P3_4		 //[PCB] P3_4
#define M2 P3_5      //[PCB] P3_5

// PID GAIN VALUES
#define kp  1.000F //6.7334    
#define ki  0.187F //1.1851
#define kd  0.069F //0.4368
#define dT  0.00083333F
volatile int position = 1111;
volatile int linposition = 2222;
volatile int angSetPoint = 808;
volatile int linSetPoint = 909;
volatile int error = 0;
volatile int prevError = 0;
volatile int past5[5]; //Array of previous errors
volatile int errSum = 0;
volatile int dErr = 0;
volatile int pwm_temp = 0;


unsigned char _c51_external_startup(void)
{
	// Configure ports as a bidirectional with internal pull-ups.
	P0M0=0;	P0M1=0;
	P1M0=0;	P1M1=0;
	P2M0=0;	P2M1=0;
	P3M0=0;	P3M1=0;
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O
	P4M0=0;	P4M1=0;
    
    // Initialize the serial port and baud rate generator
    PCON|=0x80;
	SCON = 0x52;
    BDRCON=0;
    BRL=BRG_VAL;
    BDRCON=BRR|TBCK|RBCK|SPD;
    
    //Initialize Timer 1 for ISR
    TMOD=0B_0001_0001; // 0001 is 16-bit mode (Enhanced Timer 1, p77 in docs)
    //TCONB=0B_01000000; //TCONB(7) = P3_5 PWM and TCONB(6) = P3_4 for PWM on Timer0
    TCONB=0B_11000000; //P3_5 PWM ENABLED 
    

    TR0=0;
    TR0=1;
   	TR1=1;
 	
 	RL1=250;
 	RL0=250; 
 	
	RH1=128; //Pin3.5
	RH0=128; //Pin3.4
	

	/* TIMER 2 initializations */
	T2MOD &= 0xFC;			      /* T2OE=0;DCEN=1; */
	T2MOD |= 0x01;			      

	EXF2=0;                    /* reset flag */
	TCLK=0;RCLK=0;             /* disable baud rate generator */
	EXEN2=0;                   /* ignore events on T2EX */ 
	TH2=MSB_reload_value;		/* Init msb_value */
	TL2=LSB_reload_value;		/* Init lsb_value */
	RCAP2H=MSB_reload_value;	/* reload msb_value */
	RCAP2L=LSB_reload_value;	/* reload lsb_value */
	C_T2=0;                    /* timer mode */
	CP_RL2=0;                  /* reload mode */
	EA=1;                      /* interupt enable */
	ET2=1;                     /* enable timer2 interrupt */
	TR2=1;                     /* timer2 run */



    return 0;
}

/**
 * FUNCTION_PURPOSE: timer2 interrupt
 * FUNCTION_INPUTS: void
 * FUNCTION_OUTPUTS: P1.3 toggle period = 2 * (65536-reload_value) cycles 
 */
void it_timer2(void) interrupt 5 /* interrupt address is 0x002b */
{
   P1_3 = ~P1_3;					/* P1.3 toggle when interrupt. */
	TF2 = 0;							/* reset interrupt flag */
}


void main (void)
{	
	int temp_pwm = 88;
	
	resetHCTL(1);
	resetHCTL(2);
	
	//Reset everything
	printf( FORE_BACK, COLOR_BLACK, COLOR_WHITE );
	printf( CLEAR_SCREEN );
	printf( GOTO_YX, 1, 1 );
	printf("Motor Angle      :::");
	printf("\nLin Motor Angle  :::");
	printf("\nSetpoint Angle   ::: ");
	printf("\nLinear Position  ::: ");
	
	while(1)
	{		
		printf( GOTO_YX, 1, 22 );
		position = decode(1);          //HCTL1 NEEDS CONVERSION---(Sensor Resolution*4/360)*(Gear Ratio)
		printf("%i     ", position);
		
		printf( GOTO_YX, 2, 22 );
		linposition = decode(2);       //HCTL2 (Sensor Resolution*4/360)*(360degrees/cm)*(Gear Ratio)   33.33;
		printf("%i     ", linposition);
		
		angSetPoint = GetADC(1)/3.196;	   //Angular Pot Reading in degrees TODO: Not reading for like 20 degrees around 0, measure actual angle to ensure precision.
		printf( GOTO_YX, 3, 22 );
		if(angSetPoint > 300)resetHCTL(2);
		printf("%i    ", angSetPoint);
		
		linSetPoint = GetADC(0)/(9.3);	//Linear Pot Reading out of 100
		printf( GOTO_YX, 4, 22 );
		if(linSetPoint>80)resetHCTL(1);
		printf("%i    ", linSetPoint);
		
		error = angSetPoint - position;
		
		temp_pwm = PIDcalculation(error); //WILL CHANGE WITH dT
			
			printf("\nPWM ::: %i    ", temp_pwm);
			
		//RH0 = temp_pwm;
		RH0 = linSetPoint + 28;
	}	
}

void resetHCTL (int select)
{
	if(select == 1){
		RST1 = 0; //Clear HCTL
	    waitms;
	    RST1 = 1;
	}
	else if(select == 2){
		RST2 = 0;
		waitms;
		RST2 = 1;
	}
}

int PIDcalculation (int error){
	int output;
	
	/*Control for HCTL Overflow*/
	if(error<-1700)
		error=50;
	
	/*Compute working error variables*/
	errSum += error * dT;
	dErr = (error - fivePointMovingAvg(prevError)); //Dividing by dT makes this huge
	
	/*Compute PID Output*/
	output = kp * error + ki * errSum + kd * dErr;
	
	/* Limit error */
	if(output > 128) output = 128;
	else if (output < -128) output = -127; //OVERFLOWS AT 255
	output = 128 - output;
	
	pwm_temp = output;
	
	prevError = error;
	
	//return output;
	return pwm_temp;
}

int fivePointMovingAvg (int prevError){
	int i = 0;
	int avgError, sum = 0;
	
	/* Shift Error values down, append newest to front */
	for(i=5; i>0; i--)
		past5[i] = past5[i-1];
	past5[0] = prevError;
	
	/* Calculate the average */
	for(i=6; i>0; i--)
		sum += past5[i];
	avgError = sum / 5;
	
	return avgError;
}


//         LP51B    HCTL
//---------------------------
// 5V   -  	VCC    - pin 1
// 0V   -  	GND    - pin 12 
// INDEX-  	PX.X   - pin 11
// CLK  -          - pin 3		
// SEL1 -  	P4.3   - pin 4
// SEL2 -  	P4.4   - pin 17
// OE   -  	P3.7   - pin 5
// U/D	-	P3.6   - pin 6
// RST	-	P1.1   - pin 8
// Y	-		   - pin 9
// G	-	  	   - pin 10
// D0   -	P2.0   - pin 2
// D1   -          - pin 20
// D2   -          - pin 19
// D3   -          - pin 18
// D4	-	P2.4   - pin 15
// D5	-		   - pin 14
// D6	-		   - pin 13
// D7	-	P2.7   - pin 7

//RETURN 32-BIT MEASUREMENT, read from encoder
unsigned int decode (int OE)
{
	unsigned int result = 0;
	int mult = 0;
	unsigned int temp = 0;
	unsigned int Result_lo = 0;
	unsigned int Result_2nd = 0;
	unsigned int Result_3rd = 0;
	unsigned int Result_hi = 0;
	
	if(OE==1)P3_6 = 1; //Disable OE
	else if(OE==2)P3_7 = 1; //Disable OE
	
			P4_3 = 0; //Sel1 TEST
			P4_4 = 1; //SEL2 = 1 (MSB)
			
	if(OE==1)P3_6 = 0; //Enable OE
	else if(OE==2)P3_7 = 0; //Enable OE	
	
		Result_hi = readBits(OE); // Get MSB
			P4_3 = 1; //SEL1 = 1 (2nd Byte)
			P4_4 = 1; //SEL2 = 1 (2nd Byte)
		Result_2nd  = readBits(OE);
			P4_3 = 0; //SEL1 = 0 (3rd Byte)
			P4_4 = 0; //SEL2 = 0 (3rd Byte)
		Result_3rd = readBits(OE);
			P4_3 = 1; //SEL1 = 1 (LSB)
			P4_4 = 0; //SEL2 = 0 (LSB)
		Result_lo = readBits(OE); // Get LSB
		
	if(OE==1)P3_6 = 1; //Disable OE
	else if(OE==2)P3_7 = 1; //Disable OE
	
		mult = 1;
		temp = Result_lo*mult; //Assign LSB
		result = temp;
	
		mult = mult*256; 		//Shift register up a byte
		temp = Result_3rd*mult; //Assign 3rd Byte
		result = result + temp;
		
		mult = mult*256;
		temp = Result_2nd*mult; //Assign 2nd Byte
		result = result + temp;
		
		mult = mult*256;
		temp = Result_hi*mult; //Assign MSB
		
		result = result + temp; // Final 32-bit result
	return result;
}

//READ BYTE FROM OPTICAL ENCODER
unsigned int readBits(int select)
{
	unsigned int num = 0;
	
	if(select == 1){
		num = P2_7*128 + P2_6*64 + P2_5*32 + P2_4*16 + P2_3*8 + P2_2*4 + P2_1*2 + P2_0*1;
	}
	
	else if(select == 2){
		num = P0_7*128 + P0_6*64 + P0_5*32 + P0_4*16 + P0_3*8 + P0_2*4 + P0_1*2 + P0_0*1;
	}
	
	return num;
}


void waitms (void){
_asm
mov R2, #1
L3: mov R1, #1
L2: mov R0, #184
L1: djnz R0, L1 ; 2 machine cycles-> 0.5425347us*184=100us
djnz R1, L2 ; 100us*10=1ms
djnz R2, L3 ; 1ms*25= 25ms
ret
_endasm;
}

//-------------------------------------------------------------Potentiometer + MCP3008------------------------------------------------------
//         LP51B    MCP3008
//---------------------------
// MISO  -  P1.5  -	pin 12
// SCK   -  P1.6  - pin 13		
// MOSI  -  P1.7  - pin 11		
// CE*   -  P0.3  - pin 10		
// 4.8V  -  VCC   - pins 15, 16  
// 0V    -  GND   - pins 9, 14   
// CH0   -        - pin 1
// CH1   -        - pin 2
// CH2   -        - pin 3
// CH3   -        - pin 4

void SPIWrite(unsigned char value)
{	SPSTA&=(~SPIF); // Clear the SPIF flag in SPSTA
	SPDAT=value;
	while((SPSTA & SPIF)!=SPIF); //Wait for transmission to end
}

// Read 10 bits from the MCP3008 ADC converter
unsigned int GetADC(unsigned char channel)
{	
	unsigned int adc;

	// initialize the SPI port to read the MCP3004 ADC attached to it.
	SPCON&=(~SPEN); // Disable SPI
	SPCON=MSTR|CPOL|CPHA|SPR1|SPR0|SSDIS;
	SPCON|=SPEN; // Enable SPI
	
	CE=0; // Activate the MCP3004 ADC.
	SPIWrite(channel|0x18);	// Send start bit, single/diff* bit, D2, D1, and D0 bits.
	for(adc=0; adc<10; adc++); // Wait for S/H to setup
	SPIWrite(0x55); // Read bits 9 down to 4
	adc=((SPDAT&0x3f)*0x100); 					
	SPIWrite(0x55);// Read bits 3 down to 0
	CE=1; // Deactivate the MCP3004 ADC.
	adc+=(SPDAT&0xf0); // SPDR contains the low part of the result. 
	adc>>=4;
		
	return adc;
}