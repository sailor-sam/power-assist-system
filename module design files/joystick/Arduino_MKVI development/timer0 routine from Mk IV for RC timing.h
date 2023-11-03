#include <16c74b.h>
#fuses HS,WDT,NOPROTECT,PUT,NOBROWNOUT
#use delay(clock=20000000, restart_wdt)
#include <stdlib.h>
#include <pins.h> 		// defines descriptive names for PIC pin numbers

/*******************
* LESKIW - defines *
*******************/
enum {
	BLINK_TIMER,		// mode lamp timer
	HEARTBEAT_TIMER, 	// Controls how the flashing works
	MODE_TIMER,     	// "NO ACTIVITY" TIMER, SWITCHES BACK TO WINCH MODE AFTER FIVE SECONDS
	NO_RC_PULSE_TIMER,	// IF RC PULSE NOT RECEIVED WITHIN "X" MSECS, RESETS RC PARAMETERS TO NEUTRAL
	ACK_TIMER,
	STUCK_KEY_TIMER, 	// THIS INDICATES THAT ONE OR MORE KEYS IS STUCK "DOWN" FOR MORE THAN 10 SECONDS
	BATTERY_TEST_TIMER, 	// BATTERY CONDITION IS CHECKED EVERY TEN SECONDS
	low_battery_timer,	// WHEN BATTERY REACHES CRITICAL LEVEL, THIS TIMER INTEGRATES CONDITION FOR 3 MINUTES
	NUM_TIMERS    		// this must be the last in the list
};

#define MODE_TIME 88		// initial value for mode timer (approx 5 seconds)

#define NUM_MODES  3		// FABES - max number of modes handled by this version of firmware
#define STEER_MODE 0		// rudder mode
#define WINCH_MODE 1		// winch mode
#define RIG_MODE   2		// not used in Mk IV
//#define OUTH_MODE  3		// not used in Mk IV
//#define BOOM_MODE  4		// not used in Mk IV

#define dssaDevice 0x10		// (NOT USED) make autohelm address 0x10

#BYTE	TMR0	= 0X01		// DEFINE TIMER0 REGISTER (DIRECT ACCESS)
#BYTE 	OPTION 	= 0X81 		// Port B Option register (for setting port B pullup option)
#BYTE 	INTCON 	= 0X8B 		// Interrupt control register (used to force Interrupt mask set-up)

#BYTE	PR2 	= 0X92		// DEFINE PWM CONTROL REGISTERS (DIRECT ACCESS)
#BYTE	CCPR1L 	= 0X15
#BYTE 	CCPR2L 	= 0X1B
#BYTE	TMR2 	= 0X11
#BYTE	T2CON 	= 0X12
#BYTE 	CCP1CON = 0X17
#BYTE 	CCP2CON = 0X1D

/**********************
* Function Prototypes *		// ALVEY - THESE ALLOW SUBROUTINE CODE SEGMENTS TO BE RECOGNIZED WHEREVER THEY ARE IN FILE
**********************/
void allOff(void);
void RCOff(void);
void initialize(void);
void buildModeTable(void);
char checkInputs(void);
void handleInputCommand(void);
void linkRemote(void);
void check_battery_level(void);
void beep(char numBeeps);
void batteryMeterFlashRoutine(void);
void ledsOff(void);
void restoreModeLEDs(void);
void changeMode(char newMode);
void checkJoy(void);
void checkRC(void);
void checkRF(void);
void rudstbd(void);
void rudPort(void);
void winchIn(void);
void winchOut(void);
void rigIn(void);
void rigOut(void);
void r_output_high(char sense);
void r_output_low(char sense);
void handleBeeps(void);

/*******************
* LESKIW - globals *
*******************/
char INPUT_FLAGS 	= 0;			// various control flags
#BIT STCR 		= INPUT_FLAGS.0		// 1 = sea talk command received
#BIT JOY_ENABLED 	= INPUT_FLAGS.1		// 1 = joystick is enabled
#BIT KEY_DOWN 		= INPUT_FLAGS.2		// 1 = not used in MAIN 2.0
#BIT BEEP_SENSE 	= INPUT_FLAGS.3		// 1 = identifies that sound is on
#BIT OVERCURRENT_FLAG 	= INPUT_FLAGS.4		// 1 = (not used) identifies overcurrent condition detected
#BIT PWM2_MODE 		= INPUT_FLAGS.5		// 1 = identifies PWM2 MODE (uses WINCH_IN and WINCH_OUT to control second PWM driver)
#BIT RF_MODE 		= INPUT_FLAGS.6		// 1 = identifies RF keyfob in use

signed int J_REF;				// joystick reference value, read from A/D input pin RA3
int joystick_UDvalue=0;				// calculated joystick UD PWM1 value
int joystick_RLvalue=0;				// calculated joystick RL PWM2 value

signed long RC_NEUTRAL = 117;			// RC reference value, equivalent to 1.5msec pulsewidth (calc = 117)
signed long max_pulse_length = 156;		// longest valid pulse length is 2.0 msec (approx 156 Timer0 ticks)
signed long min_pulse_length = 78;		// shortest valid pulse length is 1.0 msec (approx 78 Timer0 ticks)
signed long RC_RL;
signed long RC_UD;

char RC_FLAGS = 0;
#BIT RC_input_received 	= RC_FLAGS.0		// 1 = one (or more) RC input pulse measured to be handled in MAIN
#BIT bad_RC1_pulse 	= RC_FLAGS.1
#BIT bad_RC2_pulse 	= RC_FLAGS.2
#BIT RC_MODE 		= RC_FLAGS.3		// 1 = identifies RC MODE (uses WINCH_IN and WINCH_OUT to generate RC PWM pulses)
#BIT RF_input_received 	= RC_FLAGS.4		// 1 = one (or more) RC input pulse measured to be handled in MAIN

char JOY_POS 		= 0;			// contains joystick position flags
#BIT JUP 		= JOY_POS.0		// 
#BIT JDOWN 		= JOY_POS.1		//
#BIT JRIGHT 		= JOY_POS.2		//
#BIT JLEFT 		= JOY_POS.3		//

int debug = 0;					// holds debug flag, to control location of debug "beeps" in code
char debounce = 0;				// holds debounce flag for debug routine "button" presses

char mode = 0;					// holds current operating mode
char NUM_BEEPS = 0;				// holds number of time to beep depending on current mode
char BEEP_TIMER = 0;				// time between beeps
signed int JOY_RL_LIMIT = 20;			// joystick RL dead band
signed int JOY_UD_LIMIT = 30;			// joystick UD dead band

int timerList[NUM_TIMERS];			// define timer list
char modeList[NUM_MODES];			// define (operating) mode list

char RIG_IN = RIG_IND;				// define rig_in as pin number associated with Rig In Direction
char RIG_OUT = RIG_OUTD;			// NOTE; direction of RIG_IN can be changed at any time by pressing AUX and 
						// RIGHT FUNCTION button at the same time

char global_flags = 0;				// used to store global boolean values
#BIT watchDogAlarm = global_flags.0		// if set, the main software is stuck in a loop
#BIT rightFuncDisabled = global_flags.1		// IF SET, RIGHT FUNCTION KEY IS DISABLED
#BIT leftFuncDisabled = global_flags.2		// IF SET, LEFT FUNCTION KEY IS DISABLED
#BIT auxDisabled = global_flags.4		// IF SET, AUX KEY DISABLED
#BIT use_pwm_ramp = global_flags.5 		// IF SET, means accelerate PWM to full speed using a ramp
#BIT beepPriority = global_flags.6 		// IF SET, beep() will ignore new beeps until the current set is done
#BIT timer1_overflow = global_flags.7 		// IF SET, RC2 pulse boundary condition; ignore current pulse timing

/**************************************************
* FABES - INITIALIZE MK IV BATTERY METER FUNCTION *
**************************************************/
char battery_level = 5; 			// initialize battery to be full, EF
char read_battery = 1; 				// initialize to not read the battery level, EF
char battery_too_low = 0; 			// initialize to battery is fine, EF
char beep_flag = 0; 				// beep when battery level is low, EF


/**********************************************
* LESKIW - initialization routine (MAIN LOOP) *
**********************************************/
void initialize(void)
{
	int i;

	#USE FAST_IO(A)
	#USE FAST_IO(B)
	#USE FAST_IO(C)
	#USE FAST_IO(D)
	#USE FAST_IO(E)
	
	disable_interrupts(global);	// disable interrupts during initialization

	set_tris_a(0x1f);  		// MK IV hardware
	set_tris_b(0xc5);   		// ALLOW RC INPUTS ON PIN 33 AND 40
	set_tris_c(0x80);		
	set_tris_d(0xf3);		// 
	set_tris_e(0x06);

	allOff();			// TURN ALL OUTPUTS OFF
	LEDSoFF();			// TURN ALL LEDS OFF

	output_high(BUZZER);		// TEST BUZZER

	output_low(AUX2_RC_LED);	// LED TEST SEQUENCE
	output_low(WINDLASS_LMP);
	delay_ms(200);

	output_high(AUX2_RC_LED);
	output_low(AUX1_RELAY_LED);
	output_high(WINDLASS_LMP);
	delay_ms(200);

	output_high(AUX1_RELAY_LED);
	output_low(WINCH_LED);		
	output_low(WINDLASS_LMP);
	delay_ms(200);

	output_high(WINCH_LED);		
	output_low(HELM_LED);
	output_high(WINDLASS_LMP);
	delay_ms(200);

	output_high(HELM_LED);
	output_low(HEARTBEAT_LED);	
	output_low(WINDLASS_LMP);
	delay_ms(500);

	output_high(WINDLASS_LMP);

	output_low(BUZZER);		// end LED test

	for(i = 0; i < NUM_TIMERS; i++)
		timerList[i] = 0;

	for(i = 0; i < NUM_MODES; i++)
		modeList[i] = 0;

	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_64);	// COMPILER DOES NOT SEEM TO WORK; BELOW IS DIRECT REGISTER SET-UP OF TIMER 0

//	OPTION = 0XC5;					// timer0 is loaded on RC1 pulse edge and used to time RC pulses (as a simple counter)
							// INTERNAL SOURCE, PRESCALE = 64; NO PORTB PULL-UPS
	disable_interrupts(int_timer0);			// we do NOT want timer0 interrupts to be generated

	setup_timer_1(T1_INTERNAL|T1_DIV_BY_4); 	/* timer1 is primary real time clock; interrupts every 52 msec */
	enable_interrupts(INT_TIMER1);			// timer1 is also used in relative mode to time RC pulsewidths

	ext_int_edge(L_to_H);				// SET RB0 INTERUPT TO TRIGGER ON RC1 RECEIVER INPUT 
	enable_interrupts(INT_EXT); 			// interupt EXT is used to synchronize with rising edge of RC1 pulse
	enable_interrupts(INT_RB); 			// interupt RB is used to synchronize with rising edge of RC2 pulse
							// **** interupts are enabled alternately, so that each pulse gets exclusive use of Timer0

	PR2 = 0XFF;				// SET TMR2 COMPARE VALUE TO "255" STEPS (max PWM resolution)
	CCPR1L = 0;				// INITIALIZE PWM1 PULSE WIDTH TO 0
	CCPR2L = 0;				// INITIALIZE PWM2 PULSE WIDTH TO 0
	TMR2 = 0X10;				// INITIALIZE PWM FREQUENCY AT 20 KHZ (20 MHZ / 4/ 256)
	T2CON = T2CON | 0X04;			// TURN ON PWM2 CONTROL REGISTER, WITH NO PRE/POST SCALE VALUES

	setup_adc(ADC_CLOCK_DIV_8);
	setup_adc_ports(RA0_RA1_RA3_ANALOG);	// SET UP A/D PORTS FOR JOYSTICK AND BATTERY METER
	set_adc_channel(3);
	delay_ms(100);
	J_REF = read_adc();			// SET JOYSTICK REFERENCE VALUE (ONCE, AT POWER-UP)

	if (INPUT(PUMP_SW))			// JOYSTICK DISABLED IF LOWER LEFT SWITCH PRESSED (0) ON POWER-UP
		JOY_ENABLED = 1;
		BEEP (3);

	enable_interrupts(global);		// enable all defined interrupts

//	INTCON = 0XD8;				// enable external and Port B interrupts (forced)
}








/****************************************
* RRRRR    CCCC   1  I N     N  TTTTTTT * THIS INTERUPT SYNCHRONIZES WITH RISING EDGE OF THE
* R    R  C    C  1  I N     N     T    * RC1 PULSE, AND MEASURES WIDTH FROM 1.0 - 2.0 MSECS
* R    R  C       1  I N N   N     T    *	
* RRRRR   C       1  I N  N  N     T    * TIMER1 IS USED IN RELATIVE MODE TO TIME PULSES 
* R  R    C       1  I N   N N     T    *   
* R   R   C    C  1  I N    NN     T    * TWO TIMER1 READINGS ARE TRANSLATED TO A PWM VALUE AND A DIRECTION
* R    R   CCCC   1  I N     N     T    * NOTE THAT INT_EXT AND INT_RB ARE ENABLED ALTERNATELY, SO THAT EACH ROUTINE GETS TO USE TIMER0
****************************************/

#int_ext
void timeRC1pulse(void)
{
	signed long RC1_stop = 0;				// MUST INITIALIZE RC1_STOP TO 0

//	disable_interrupts(int_ext);
//	disable_interrupts(int_rb);

	set_timer0(0);						// reset Timer0 to start pulse timing count
	timerList[NO_RC_PULSE_TIMER] = 2;			// reset NO_RC_PULSE_TIMER to 100 msec
	delay_us(100);						// debounce RC1 pulse START

	while (input(RC1_INPUT))				// debounce, and stay here for 1 - 2 msec to time RC1 pulse
	{
		restart_wdt();
		RC1_stop = get_timer0();			// AMPLIFY RESULT BY ADDING "8" COUNTS
		if (RC1_stop > max_pulse_length)
		{
			bad_RC1_pulse = 1;			// validate pulse length to be < 2.0 msec
			beep(1);				// BEEP if bad pulse detected
			break; 
		}
	}
	if (RC1_stop < min_pulse_length) 
	{
		bad_RC1_pulse = 1;				// validate pulse length to be > 1.0 msec
		beep(1);					// BEEP if bad pulse detected
	}
	else
	{
		RC_RL = ((signed long) RC_NEUTRAL - RC1_stop);	// calculate RC1 offset time and return
		RC_RL <<=1;					// amplify RC_RL by factor of 2 (same scale as joystick)
	}
//	enable_interrupts(int_rb);
}



/*********************************************
* RRRRR    CCCC    2222   I N     N  TTTTTTT * THIS INTERUPT SYNCHRONIZES WITH RISING EDGE OF 
* R    R  C    C  2    2  I N     N     T    * THE RC2 PULSE, AND MEASURES WIDTH FROM 1.0 - 2.0 MSECS
* R    R  C           2   I N N   N     T    *	
* RRRRR   C          2    I N  N  N     T    * TIMER1 IS USED IN RELATIVE MODE TO TIME PULSES 
* R  R    C         2     I N   N N     T    *   
* R   R   C    C   2      I N    NN     T    * TWO TIMER1 READINGS ARE TRANSLATED TO A PWM VALUE AND A DIRECTION
* R    R   CCCC   222222  I N     N     T    * NOTE THAT INT_EXT AND INT_RB ARE ENABLED ALTERNATELY, SO THAT EACH ROUTINE GETS TO USE TIMER0	
*********************************************/

#int_rb								// 01/07/05 this interrupt WORKS...
void timeRC2pulse(void)
{
	signed long RC2_stop = 0;				// MUST INITIALIZE RC2_STOP TO 0

//	disable_interrupts(int_ext);
//	disable_interrupts(int_rb);

	set_timer0(0);						// reset Timer0 to start pulse timing count
	timerList[NO_RC_PULSE_TIMER] = 2;			// reset NO_RC_PULSE_TIMER to 100 msec
	delay_us(100);						// debounce RC2 pulse START

	while (input(RC2_INPUT))				// debounce, and stay here for 1 - 2 msec to time RC2 pulse
	{
		restart_wdt();
		RC2_stop = get_timer0();			// AMPLIFY RESULT BY ADDING "8" COUNTS
		if (RC2_stop > max_pulse_length)
		{
			bad_RC2_pulse = 1;			// validate pulse length to be < 2.0 msec
			beep(1);				// BEEP if bad pulse detected
			break; 
		}
	}
	if (RC2_stop < min_pulse_length) 
	{
		bad_RC2_pulse = 1;				// validate pulse length to be > 1.0 msec
		beep(1);					// BEEP if bad pulse detected
	}
	else
	{
		RC_UD = ((signed long) RC_NEUTRAL - RC2_stop);	// calculate RC2 offset time and return
		RC_UD <<=1;					// amplify RC_UD by factor of 2 (same scale as joystick)
	}
//	enable_interrupts(int_ext);








	/*******************************************
	* LESKIW - CHECK JOYSTICK COMMAND RECEIVED *	// PRIORITY ONE INPUT
	*******************************************/
	if (!retVal && JOY_ENABLED) 			// R DALLAIRE BUILD - JOY_ENABLED FLAG ADDED TO DISABLE JOYSTICK
	{
		checkJoy();
		if (JOY_POS)
			return 1;			// IF RC SCREWS UP, WILL ALWAYS GIVE PRIORITY TO JOYSTICK INPUT
	}


	/************************************
	* ALVEY - CHECK RC COMMAND RECEIVED *		// PRIORITY TWO INPUT
	************************************/
	if (!retVal) 
	{
		checkRC();
		if (JOY_POS)
			return 1;			// if RC input present, retVal = 1
	}	

	/*****************************************************
	* check inputs 4 (or 5) SWITCH IMBEDDED RF RECEIVER * 	// PRIORITY TWO INPUT
	*****************************************************/

	if (!retVal) 
	{

#if 0	/******  BITE SWITCH (FUNCTION) IS NOT USED IN RF VERSION, BUT CAN BE IMPLEMENTED ON PIN B2 ********************/

		if(input(BITE_SW))
		{
			retVal = 0;
			delay_ms(200);
			if(input(BITE_SW))			// IF BITE SWITCH - TOGGLE MODE
			{
				if(!EXTENDED_MODES)		// SET FLAG, CHANGE TO EXTENDED MODE 2
				{
					EXTENDED_MODES = 1;
					changeMode(2);
				}
				else 
				{
					EXTENDED_MODES = 0;
					changeMode(WINCH_MODE);
				}
			}
		}
#endif	
		if(input(STEERPUFF_SW))		
		{
			JLEFT = 1;
			use_pwm_ramp =1;
			retVal = 1;
		}
	
		if(input(STEERSIP_SW))
		{
			JRIGHT = 1;
			use_pwm_ramp =1;
			retVal = 1;
		}

		if(input(WINCHPUFF_SW))		
		{
			JUP = 1;
			retVal = 1;
		}
	
		if(input(WINCHSIP_SW))
		{
			JDOWN = 1;
			retVal = 1;
		}
	}
	return retVal;
}

/**************************
* LESKIW - check joystick *	verified same as MAIN19	// READ JOYSTICK DEFLECTION AND STORE DIRECTION AND ABSOLUTE VALUE
**************************/
void checkJoy(void)
{
	int sampleUDVal;		  
	int sampleRLVal;
	signed int joy_UD = 0;
	signed int joy_RL = 0;

	JOY_POS =0;
	{
		set_adc_channel(1);
		delay_ms(20);
		sampleRLVal = read_adc();				// READ THE JOYSTICK RL VALUE 
		joy_RL = (signed int) J_REF - (signed int) sampleRLVal;

		if (joy_RL > JOY_RL_LIMIT)
		{
			JRIGHT = 1;
			use_pwm_ramp=0;
			joystick_RLvalue = abs(joy_RL);			// save Right Left joystick offset value
		}
		else if (joy_RL < -(JOY_RL_LIMIT))
		{
			JLEFT = 1;
			use_pwm_ramp=0;
			joystick_RLvalue = abs(joy_RL);			// save Right Left joystick offset value
		}
	}
	if(!JOY_POS)			
	{
		set_adc_channel(0);
		delay_ms(20);
		sampleUDVal = read_adc();		 		// READ THE JOYSTICK UD VALUE 
		joy_UD = (signed int) J_REF - (signed int) sampleUDVal;

		if (joy_UD > JOY_UD_LIMIT)
		{
			JDOWN = 1;
			joystick_UDvalue = abs(joy_UD);			// save Up Down joystick offset value
		}
		else if (joy_UD < -(JOY_UD_LIMIT))
		{
			JUP = 1;
			joystick_UDvalue = abs(joy_UD);			// save Up Down joystick offset value
		}
	}

}


/****************************************		Program uses Timer0 to time RC1 or RC2 pulses 
* 			 		*		Program "waits" in this routine during timing of either pulse
*  ALVEY - check RC INPUTS (MAIN)	*		in order to provide timing as accurately as possible
*					*		
****************************************/		

void checkRC(void)
{
	JOY_POS =0;

	if (timerList[NO_RC_PULSE_TIMER] == 0)
	{
		RCOff();				// if there are no RC pulses, then RC must be turned off or out of range 
		return; 				// reset RC offset values and return to calling program
	}
	if (bad_RC1_pulse)
	{	
		bad_RC1_pulse = 0;
	}
	else
	{
		if (RC_RL > JOY_RL_LIMIT)
		{
			JRIGHT = 1;
			use_pwm_ramp=0;
			joystick_RLvalue = abs(RC_RL+8);	// "amplify" and save RC CH1 offset value
		}
		else if (RC_RL < -(JOY_RL_LIMIT))
		{
			JLEFT = 1;
			use_pwm_ramp=0;
			joystick_RLvalue = abs(RC_RL-8);	// "amplify" and save RC CH1 offset value
		}
	}

	if(!JOY_POS)						// if no RL input, then check for UD input
	{
		if (bad_RC2_pulse)	
		{
			bad_RC2_pulse = 0;
		}
		else
		{
			if (RC_UD > JOY_UD_LIMIT)
			{
				JUP = 1;
				joystick_UDvalue = abs(RC_UD+8);  // "amplify" and save RC CH2 offset value
			}
			else if (RC_UD < -(JOY_UD_LIMIT))
			{
				JDOWN = 1;
				joystick_UDvalue = abs(RC_UD-8);  // "amplify" and save RC CH2 offset value
			}
		}
	}
}
/***********************************
* handle input command (MAIN LOOP) *
***********************************/

void handleInputCommand()
{
	if( timerList[ACK_TIMER] == 0)		// IF INPUT COMMAND PRESENT, BEEP ONCE
	{
		beep(1);
		timerList[ACK_TIMER] = 20;
	}

	if(JRIGHT | JLEFT)			// ACTIVATE JOYSTICK LEFT/RIGHT
	{
		if(JRIGHT)
			rudStbd();
		else
			rudPort();
	}
	
	else if(JUP | JDOWN)
	{
		switch(mode)			// ACTIVATE JOYSTICK UP/DOWN
		{
			case(STEER_MODE):
			case(WINCH_MODE):
				if(JUP)
					winchOut();
				else
					winchIn();
				break;

			case(RIG_MODE):
				if(JUP)
					rigOut();
				else
					rigIn();
				break;
			default:
		}
	}
//	joy_pos = 0;				// acknowledge that joystick input has been handled
	timerList[MODE_TIMER] = MODE_TIME;	// reset MODE_TIMER to five seconds
}


/************************************************
*						*
* LESKIW - OUTPUT CONTROL ROUTINES (MAIN LOOP) 	*
*					    	*
************************************************/


/*****************************
* rudder to port (MAIN LOOP) *
*****************************/
void rudPort(void)
{
	if (!input(OVERCURRENT))		// first check for OVERCURRENT condition (TRUE = LOW) and clear if present
	{
		CCPR2L = 0;			// if OVERCURRENT, reduce PWM duty cycle
		delay_ms(100);
		output_low(PWM1_DIRECTION);	// then toggle direction bit to clear condition
		beep(3);
	}

	if (!(CCP2CON & 0X0C))			// IF NOT IN PWM MODE...
	{
		CCP2CON = CCP2CON | 0X0C;	// TURN ON PWM MODE
	}
	if(use_pwm_ramp)
	{					// test if "digital" or "analog" input
		if (CCPR2L < 223)
		{
			CCPR2L = CCPR2L + 32;	// either 1) "ramp" PWM
		}
		else
		{
			CCPR2L = 255;		// or 2) go full-speed...
		}
	}

//*************** 072607 something wrong with joystick response - temp change full speed threshold from "70" to "50" to debug ************

	else if (joystick_RLvalue < 50)		// test deflection of joystick...
	{
		CCPR2L = joystick_RLvalue*3;  	// either 1) use (amplified) joystick value for PWM
	}
	else
	{	
		CCPR2L = 255;			// or 2) go full-speed...
	}

	output_high(PWM1_DIRECTION);
	delay_us(50);
	output_high(PWM1);			
}


/*****************************
* rudder to stbd (MAIN LOOP) *
*****************************/
void rudstbd(void)
{
	if (!input(OVERCURRENT))		// first check for OVERCURRENT condition (TRUE = LOW) and clear if present
	{
		CCPR2L = 0;			// if OVERCURRENT, reduce PWM duty cycle
		delay_ms(100);
		output_high(PWM1_DIRECTION);	// then toggle direction bit to clear condition
		beep(3);
	}

	if (!(CCP2CON & 0X0C))			// IF NOT IN PWM MODE...
	{
		CCP2CON = CCP2CON | 0X0C;	// TURN ON PWM MODE
	}
	if(use_pwm_ramp)
	{					// test if "digital" or "analog" input
		if (CCPR2L < 223)
		{
			CCPR2L = CCPR2L + 32;	// either 1) "ramp" PWM
		}
		else
		{
			CCPR2L = 255;		// or 2) go full-speed...
		}
	}

//*************** 072607 something wrong with joystick response - temp change full speed threshold from "70" to "50" to debug ************

	else if (joystick_RLvalue < 50)		// test deflection of joystick...
	{
		CCPR2L = joystick_RLvalue*3;  	// either 1) use (amplified) joystick value for PWM
	}
	else
	{	
		CCPR2L = 255;			// or 2) go full-speed...
	}

	output_low(PWM1_DIRECTION);
	delay_us(50);
	output_high(PWM1);		
}

/***********************
* winch in (MAIN LOOP) *
***********************/
void winchIn(void)
{
	output_high(WINCH_IN);
}

/************************
* winch out (MAIN LOOP) *
************************/
void winchOut(void)
{
	output_high(WINCH_OUT);
}

/*********************
* rig in (MAIN LOOP) *
*********************/
void rigIn(void)
{
//	r_output_high(RIG_IN);			// 2/9/05 DISABLE "TOGGLE" FUNCTION; DOES NOT WORK WITH RC INPUTS
	output_high(RIG_IND);
}

/**********************
* rig out (MAIN LOOP) *
**********************/
void rigOut(void)
{
//	r_output_high(RIG_OUT);			// 2/9/05 DISABLE "TOGGLE" FUNCTION; DOES NOT WORK WITH RC INPUTS
	output_high(RIG_OUTD);
}

/*****************************************
* Auxialliary output control (MAIN LOOP) *	
*****************************************/
void r_output_high(char sense)
{
	if (sense == RIG_IND)
	{
		output_high(RIG_IND);
	}
	else
	{
		output_high(RIG_OUTD);
	}
}

void r_output_low(char sense)
{
	if (sense == RIG_IND)
	{
		output_high(RIG_IND);
	}
	else
	{
		output_high(RIG_OUTD);
	}
}


/**************************************************
* LESKIW - buildModeTable (MAIN LOOP) *
**************************************************/
void buildModeTable(void)
{
	int i;
	modeList[STEER_MODE] = 1;
	modeList[WINCH_MODE] = 1;
	for(i = 2; i < NUM_MODES; i++)
		modeList[i] = 0;

	modeList[RIG_MODE] = 1;
}

/***************************************
* ALVEY - turn RC off (MULTIPLE CALLS) *	// ALVEY - IF RC PULSES ARE "LOST" FOR MORE THAN 100 MSEC, TURN RC OFF
***************************************/
void RCOff(void)
{
	RC_UD = 0;				// reset (Up Down) PWM value to Neutral
	RC_RL = 0;				// reset (Right Left) PWM value to Neutral
}

/*****************************************
* LESKIW - turn all off (MULTIPLE CALLS) *	// ALVEY - BEFORE HANDLING ANY NEW INPUT, TURN "ALL OUTPUTS OFF"
*****************************************/
void allOff(void)
{
//	JOY_POS = 0;				// 2/9/05  SHOULD NOT CLEAR JOY_POS HERE ...

	output_low(PWM1);
	output_low(WINCH_IN);	
	output_low(WINCH_OUT);
	output_low(RIG_IND);
	output_low(RIG_OUTD);

	CCPR1L = 0;
	CCPR2L = 0;				// PWM OFFSET COUNTERS ARE SET TO "0"

	timerList[ACK_TIMER] = 0;

	if (timerList[MODE_TIMER] == 0 && mode !=0)
	{
		changeMode(0);
	}
}

/************************************
* ALVEY - LEDS OFF (MULTIPLE CALLS) *
************************************/
void ledsOff(void)
{
	output_high(HEARTBEAT_LED);
	output_high(HELM_LED);
	output_high(AUX2_RC_LED);
	output_high(AUX1_RELAY_LED);
	output_high(WINCH_LED);
}

}
