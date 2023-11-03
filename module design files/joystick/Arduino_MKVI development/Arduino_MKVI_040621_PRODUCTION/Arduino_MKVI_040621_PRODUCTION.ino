/*******************************************************************************************************************************************************************
*
* POWER-ASSIST SYSTEM - MK VI SOFTWARE  ************** ARDUINO UNO (ATMEGA328)  MARCH 2021 *************************************************************************
*
* BY:         Project Manager:  Steve Alvey 403-870-7210
*             SW developer:     Steve Alvey 403-870-7210
*
* VERSION:    1.0RC POWER-ASSIST MK VI  
*             Compatible with Mk VI REVC PCB (2021)
*
* PURPOSE:    The POWER-ASSIST System provides control of HELM and WINCH 12V DC servo motors, to
*             control sails and steering system of small sailboats.  Sailors with limited
*             hand function may control the Autohelm System via a wheelchair style joystick, 
*             Sip & Puff interface.   
*             For safety, an (optional) RC Remote joystick is supported. When in use, commands received from the RC Remote Control 
*             are given priority over operator joysitck or sip and puff commands.  Once ANY RC input command is received, 
*             Joystick and sip and puff commands are ignored for ONE SECOND.  After ONE SECOND, joystick and sip and puff commands
*             will be handled.
*             See www.accessiblesailingtechnology.com for pictures and 
*             Power Assist System Operator Manual.
*
* LAST CHANGED: 4/06/21 S ALVEY - inital PRODUCTION BUILD tested and certified
*               Outstanding issues:  NONE
*
* SPECIAL NOTES:    
*               1. N.O. pushbutton is connected to pin 1, to be used for future programming of MODES (programming LED present)
*               2. pin 28 has tri-state input jumper, for use as hardware MODE input
*               3. I2C interface may be added to pins A4 and A5 in future
* 
* 
* GENERAL DESCRIPTION OF S/W:
* 
* This software performs five functions continuously, four in the MAIN loop and the RC inputs via Interupt Service Routines (ISR): 
*
* readbattery();          Battery voltage is polled continuously.  When less than 12.V, Battery Low LED is flashed.  
*                         When less than 11.5V, Battery Low LED is ON and buzzer is ON as a warning to the Operator.  
*                         If battery voltage remains below 11.5V for more than ONE MINUTE, Helm and Winch outputs are disabled. Battery Low LED and buzzer remain ON.
*                          
* checkInputs();          The Arduino receives inputs from one of THREE SOURCES: A joystick on the Module; Sip & Puff sensors; or a remote control RC Joystick
*                         Input commands are polled continuously and executed (one at a time), in priority:  RC joystick commands are PRIORITY ONE and 
*                         will override conflicting joystick or sipp & puff commands received within ONE SECOND. 
*                         When a valid input command is received, that command must be released before any NEW command will be accepted.
*                         Although the sip&puff and RC joystick inputs are analog (proportional), all inputs are sensed and decoded as BINARY, 
*                         once they exceed a "deadband" threshold.  The deadband varies for each INPUT to establish an ergonomic sensitivity for that input device. 
*                         (i.e. joystick sensitivity is NOMINAL; sip & puff sensitivity is INCREASED;  RC joystick sensitivity is DECREASED).  
*                         
* handleInputCommand();   When an input command is received, it is decoded (joystick position) and then passed to be handled by handleInputCommand routine.  
*                         The HELM or WINCH actuator is activated in the appropriate direction for the period the INPUT is present.  Only ONE INPUT will be handled at a time.
*                         The HELM output is a proportional PWM drive for brushed DC motor, generating PWM (ANALOG) and PWM DIRECTION (DIGITAL) outputs.  
*                         Since all inputs are decoded as BINARY, the use of PWM proportional output is limited to the implementation of a "soft start" for the DC motor (PWM 'ramp').
*                         The WINCH output actuates a DC Relay, located remotely.  Two active HIGH signals: WINCHIN and WINCHOUT are generated on DIGITAL outputs, and 
*                         then inverted by a ULN2003 driver to actuate the DC Relay.
*                         
* heartbeat_LED();        The heartbear LED 'blinks' at 1 Hz, indicating the Arduino is running.  When an INPUT is present, the buzzer is activated at a 1 Hz rate, 
*                         providing audio feedback to the Operator.  The buzzer is also used to warn the Operator that the battery is LOW.  
* 
* 
* RC1/2_Input;            A remote control RC Joystick is an (optional) 'plug-n-play' device.  When installed on the Mk VI PCB, the RC Receiver will generate two independent RC channels 
*                         (HELM = RC Receiver CH1, WINCH = RC Receiver CH3), each channel producing an PWM signal (RC standard:  1 - 2ms pulse at 50 Hz rate; 1.5ms nominal).  
*                         Interupt Service Routines timeRC1Pulse and timeRC2Pulse provide critical timing of the PWM pulse duration by synchronizing 
*                         with the rising and falling edges of each PWM pulse.  These values are passed to the checkInputs routine for decoding.
* 
***************************************************************************************************************************************************************************/



/***********************************************
/  define ATMega328 pins
/**********************************************/

#define   interrupts()
#define   USB_TXD         0     // O  PD0
#define   PROGRAM         0     // O  PD0
#define   USB_RXD         1     // I  PD1
#define   RC1_INPUT       2     // I  PB0   
#define   RC2_INPUT       3     // I  PB1   
#define   BUZZER          4     // O  PD4
#define   WINDLASS_LMP    5     // O  PD5
#define   WINCH_OUT       6     // O  PD6
#define   WINCH_IN        7     // O  PD7
#define   PWM1_DIRECTION  8     // O  PD4
#define   PWM1            9     // O  PD3
#define   BAT_LOW_LED     10    // O  PB2
#define   WINCH_LED       11    // O  PB3
#define   HELM_LED        12    // O  PB4
#define   HEARTBEAT_LED   13    // O  PB5
#define   JOYSTICK_HELM   A0    // I  PC0
#define   JOYSTICK_WIND   A1    // I  PC1
#define   SIPPUFF_HELM    A2    // I  PC2
#define   SIPPUFF_WIND    A3    // I  PC3
#define   BATTERY         A4    // I  PC4
#define   MODE            A5    // I  PC4

/***********************************************
/  C++ function declarations
/**********************************************/
void readbattery(void);
void allOff(void);
byte checkInputs(void);
void handleInputCommand(void);
void heartbeat_LED(void);

/***********************************************
/   constants/variables for heartbeat timer routine
/**********************************************/
int ledState = LOW;                     // 'state' variable used to set the LED
bool beep = false;                      // boolean 'state' variable to activate BUZZER
unsigned long currentMicros = 0;        // read the current time...
unsigned long previousMillis = 0;       // store last time LED 'state' was changed
const long blink_interval = 500;        // set LED 'blink' interval (milliseconds)   

/***********************************************
/    constants/variables for joystick routine
/**********************************************/
byte JOY_POS = 5;                       // joystick position:  joy CENTER (no input) = 5; joy RIGHT = 4; joy LEFT = 3; joy UP = 2; joy DOWN = 1
int joy_LR = 0;                         // variable for joystick helm value
int snp_LR = 0;                         // variable for sip and puff helm value
int joy_UD = 0;                         // variable for joystick winch value
int snp_UD = 0;                         // variable for sip and puff winch value
const int J_REF = 512;                  // set the centerpoint of joystick ADC 
const int deadband = 100;               // set joystick ADC deadband (512 +/- 100)   

/**********************************************************************
/   constants/variables for battery meter routine
/*********************************************************************/
int battery_V = 0;                      // variable for battery voltage
byte beepState = LOW;                   // 'state' variable used to set the BUZZER
byte deadBatteryState = LOW;            // 'state' variable used for dead battery voltage (less than 11.5V)
const int batterylow = 770;             // set battery low constant @ 12.2V
const int batterydead = 725;            // set battery dead constant @ 11.5V
unsigned long Battery_State_timer = 0;  // timer used to measure duration of "dead battery" state       
unsigned long Dead_Battery_detected = 0;// record the time when "dead battery" state (< 11.5V) is first detected;  RESET this time whenever battery voltage recovers to ABOVE "dead battery" state (< 11.5V)

/***********************************************
/   constants/variables for RC input routine
/**********************************************/
int RC1_Pin = 0;                        // variable for reading RC1 pin
int RC2_Pin = 0;                        // variable for reading RC2 pin
int RC1_Value = 512;                    // variable for RC joystick winch value (512 = neutral)
int RC2_Value = 512;                    // variable for RC sip and puff winch value (512 = neutral)
unsigned long RC_NEUTRAL = 1500;        // RC reference value, equivalent to 1.5msec pulsewidth
const long max_pulse_length = 2000;     // longest valid pulse length is 2.0 msec
const long min_pulse_length = 500;      // shortest valid pulse length is 500 millisec
bool startflag_RC1_Pulse = true;
bool startflag_RC2_Pulse = true;
byte bad_RC1_pulse = 0;                 // declared, but not used 040321
byte bad_RC2_pulse = 0;                 // declared, but not used 040321
unsigned long startofPulse = 0;         // record the time of the 'start' of an RC pulse        
unsigned long RC_priority_interval = 1000;    // RC Commands are given PRIORITY.  Once a command is received, joystick and s&p commnads are disabled for RC_priority_interval (ONE second = 1,000 milliseconds)
unsigned long RC_Command_received = 0;        // initialized (0) each time RC command is received

/******************************************************
/    constants/variables for PWM output generation
/******************************************************/
byte pwm_value = 0;                     // value from 0 - 255 for PWM speed      
byte pwm_dir = 0;                       // value 1 or 0 for direction
const byte ramp_inc = 2;                // set joystick pwm ramp increment to implement 'soft start' of HELM actuator




/******************************************************************************************************************************/
void setup()                            //execute setup ONCE at start up, to initialize system states
{                                    
    pinMode(PROGRAM, INPUT);            // button switch for programming/debugging
    pinMode(JOYSTICK_HELM, INPUT);      // joystick helm
    pinMode(JOYSTICK_WIND, INPUT);      // joystick windlass
    pinMode(RC1_INPUT, INPUT);          // joystick helm
    pinMode(RC2_INPUT, INPUT);          // joystick windlass
    pinMode(SIPPUFF_HELM, INPUT);       // joystick helm
    pinMode(SIPPUFF_WIND, INPUT);       // joystick windlass
    pinMode(BATTERY, INPUT);            // Battery voltage sense
    pinMode(MODE, INPUT);               // mode sense (tri-state)      
    pinMode(HEARTBEAT_LED, OUTPUT);     // Heartbeat LED
    pinMode(BAT_LOW_LED, OUTPUT);       // Bat Low LED
    pinMode(WINCH_LED, OUTPUT);         // Winch LED
    pinMode(HELM_LED, OUTPUT);          // Helm LED
    pinMode(WINDLASS_LMP, OUTPUT);      // Remote LED on Windlass
    pinMode(BUZZER, OUTPUT);            // Piezo Buzzer
    pinMode(PWM1, OUTPUT);              // PWM1 helm
    pinMode(PWM1_DIRECTION, OUTPUT);    // PWM1_direction helm

    Serial.begin(115200);
    while(!Serial);                     // wait until SERIAL link is established....

    setPwmFrequency(PWM1, 8);           // increase Arduino PWM frequency to 3.9 KHz
    
    attachInterrupt(digitalPinToInterrupt(RC1_INPUT),timeRC1pulse,CHANGE);     // attach interrupt handler for RC1_INPUT.  ISR called on both RISING and FALLING edge of PWM pulse
    attachInterrupt(digitalPinToInterrupt(RC2_INPUT),timeRC2pulse,CHANGE);     // attach interrupt handler for RC2_INPUT.  ISR called on both RISING and FALLING edge of PWM pulse

    allOff();                           // initialize all outputs to OFF
}                                       // END of setup()
/*************************************************************************************************************************/




  

/********************************************  
 *        CHANGE ARDUINO PWM FREQUENCY      *   Allows Arduino PWM frequency to be raised from 488 Hz to 3.9 KHz for higher motor efficiency
 ********************************************/  
void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
} 

/*****************************************
*          turn ALL off (MULTIPLE CALLS) *  
*****************************************/
void allOff()
{
  JOY_POS = 5;                              // turn helm drive OFF
  pwm_value = 0;                            // turn helm drive OFF
  RC1_Value = J_REF;
  RC2_Value = J_REF;
  digitalWrite(PWM1_DIRECTION, pwm_dir);
  analogWrite(PWM1, pwm_value);
  digitalWrite(HELM_LED, LOW);

  digitalWrite(WINCH_OUT, LOW);             // TURN OFF WINCH AND WINCH LED
  digitalWrite(WINCH_IN, LOW);
  digitalWrite(WINCH_LED, LOW);

  beep = false;
}

/********************************************   check battery voltage continuously
 *        BATTERY METER ROUTINE             *   when battery voltage drops below 12.2V, indicate with Battery Low LED flashing
 ********************************************   when battery volatge drops below 11.5V, indicate with Battery Low LED ON, and buzzer ON
                                                If battery voltage remians below 11.5V for more than ONE MINUTE, all outputs will be turned OFF to protect battery */
void readbattery()
{
     unsigned long Battery_State_timer = millis();          // update Battery_State_timer 
     battery_V = analogRead(BATTERY);
     if (battery_V < batterylow && battery_V > batterydead) // if battery V greater than 11.5V but less than 12.2V:  Battery Low LED flashing as "low battery" warning
     {
      digitalWrite(BAT_LOW_LED, ledState);                  
      Dead_Battery_detected = millis();                     //reset Dead_Battery_detected timer
     }
     else if (battery_V < batterydead)                      // if battery V less than 11.5V (dead): Battery Low LED on full, BUZZER on as "dead battery" warning.  Then, if 'dead' voltage condition (< 11.5V) remains for ONE MINUTE, turn all outputs OFF (allOFF).  
     {
      digitalWrite(BAT_LOW_LED, HIGH);
      digitalWrite(BUZZER, HIGH);  
      
      if ((Battery_State_timer - Dead_Battery_detected) >= 60000)   // if 'dead' voltage condition (< 11.5V) remains for ONE MINUTE (60,000 milliseconds), turn all outputs OFF (allOFF).
      {
        allOff();                                       
      }
     }
     else
     {
      digitalWrite(BAT_LOW_LED, LOW);                       // if battery V more than 12.2V:  Battery OK
      Dead_Battery_detected = millis();                     //reset Dead_Battery_detected timer
     }
}

/********************************************    check for joystick, RC, Sip & Puff input command
 *        CHECK INPUT COMMANDS              *    RC Joystick is offered as an optional SAFETY device, for Disabled Sailing Program Operator.  
 ********************************************    if RC Joystick (option) is present and turned ON, these commands will be given PRIORITY over a sailor's joystick or sip & puff commands  */

byte checkInputs()
{
  unsigned long RC_Priority_timer = millis();                                           // RC Commands are given PRIORITY ONE.  Once an RC command is received, joystick and s&p commands are disabled for ONE second
   joy_LR = analogRead(JOYSTICK_HELM);
   snp_LR = analogRead(SIPPUFF_HELM);                            
   joy_UD = (1024 - analogRead(JOYSTICK_WIND));                                         //  NOTE: reverse polarity of joystick up/down
   snp_UD = (analogRead(SIPPUFF_WIND));          
    
    if (RC1_Value > (J_REF + (deadband*2)))                                             // RC joystick deflection (PRIOIRTY ONE) greater than deadband = HELM PORT COMMAND
    {
     JOY_POS = 3;
     RC_Command_received = millis();
     return;
    }
    else if (RC1_Value < (J_REF - (deadband*2)))                                        // RC joystick deflection (PRIOIRTY ONE) less than deadband = HELM STARBOARD COMMAND
    {
     JOY_POS = 4;
     RC_Command_received = millis();
     return;
    }
    else if (RC2_Value > (J_REF + (deadband*3)))                                        // RC joystick deflection (PRIOIRTY ONE) greater than deadband = WINCH OUT COMMAND
    {
     JOY_POS = 1;
     RC_Command_received = millis();
     return;
    }
    else if (RC2_Value < (J_REF - (deadband*3)))                                        // RC joystick deflection (PRIOIRTY ONE) less than (neg) deadband = WINCH IN COMMAND
    {
     JOY_POS = 2;
     RC_Command_received = millis();
     return;
    }


  if ((RC_Priority_timer - RC_Command_received) >= RC_priority_interval)                // RC Commands are given PRIORITY ONE.  Once an RC command is received, joystick and s&p commnads are disabled for RC_priority_interval (ONE second)
  {
    if ((joy_LR > (J_REF + deadband)) || (snp_LR > (J_REF + (deadband/2))))             // joystick deflection (or sip/puff) (PRIORITY TWO) greater than deadband = HELM PORT COMMAND
    {
     JOY_POS = 3;
     return;
    }
    else if ((joy_LR < (J_REF - deadband)) || (snp_LR < (J_REF - (deadband/2))))        // joystick deflection (or sip/puff) (PRIORITY TWO) less than deadband = HELM STARBOARD COMMAND
    {
     JOY_POS = 4;
     return;
    }
    else if ((joy_UD > (J_REF + deadband)) || (snp_UD > (J_REF + (deadband/2))))        // joystick deflection (or sip/puff) (PRIORITY TWO) greater than deadband = WINCH OUT COMMAND
    {
     JOY_POS = 1;
     return;
    }
    else if ((joy_UD < (J_REF - deadband)) || (snp_UD < (J_REF - (deadband/2))))        // joystick deflection (or sip/puff) (PRIORITY TWO) less than (neg) deadband = WINCH IN COMMAND
    {
     JOY_POS = 2;
     return;
    }   
  }
}

/********************************************    when input command received, execute requested OUTPUT
 *        HANDLE INPUT COMMANDS              *   for safety, only one OUTPUT command can be executed at one time
 *******************************************/  
  
void handleInputCommand()
{
//Serial.println(JOY_POS);
 switch(JOY_POS)
 {
  case 1:
  {      
     digitalWrite(WINCH_OUT, HIGH);                                   // if Joystick position = 1, actuate WINCH OUT 
     digitalWrite(WINCH_LED, HIGH); 
     beep = true;
     JOY_POS = 5;                                                     // command complete; RESET joystick position flag
     break;
  }
  case 2:
  {  
    digitalWrite(WINCH_IN, HIGH);                                     // if Joystick position = 2, actuate WINCH IN 
    digitalWrite(WINCH_LED, HIGH);
    beep = true;
    JOY_POS = 5;                                                      // command complete; RESET joystick position flag
    break;
  }
  case 3:
  {
    pwm_dir = 1;                                                      // if Joystick position = 3, actuate HELM to port
    if (pwm_value < 250)                                              // pwm output value is 'ramped' from 0 - 255 to implement 'soft start' for HELM output
    {
      pwm_value = (pwm_value + ramp_inc);                             // increment 'pwm ramp' value
      beep = true;
      delay(1);
    }
    else
    {
      pwm_value = 255;                                                // go FULL SPEED
      beep = true;
    }
      digitalWrite(PWM1_DIRECTION, pwm_dir);
      analogWrite(PWM1, pwm_value);
      digitalWrite(HELM_LED, HIGH);
      JOY_POS = 5;                                                     // command complete; RESET joystick position flag
      break;
  }
  case 4:
  {
    pwm_dir = 0;                                                       // if Joystick position = 4, actuate HELM to starboard
    if (pwm_value < 250)                                               // pwm output value is 'ramped' from 0 - 255 to implement 'soft start' for HELM output
    {
     pwm_value = (pwm_value + ramp_inc);                               // increment 'pwm ramp' value
     beep = true;
     delay(1);
    }
    else
    {
      pwm_value = 255;                                                // go FULL SPEED
      beep = true;
    }
      digitalWrite(PWM1_DIRECTION, pwm_dir);
      analogWrite(PWM1, pwm_value);
      digitalWrite(HELM_LED, HIGH);
      JOY_POS = 5;                                                    // command complete; RESET joystick position flag
      break;
  }
  case 5:                                                             // if Joystick position = 5; no INPUT is present
  {
    allOff();                                                         // turn all outputs OFF                                 
    break;
  }
 }
}


/********************************************   check to see if it's time to blink the LED: that is, if the difference
 *        HEARTBEAT LED ROUTINE             *   between the current time and last time you blinked the LED is greater than
 ********************************************   the 'blink' period (blink_interval)  */

 void heartbeat_LED()
 {     
   unsigned long currentMillis = millis();                // read the current time...
   if (currentMillis - previousMillis >= blink_interval)  // once 'blink_interval' time reached....
   {
     previousMillis = currentMillis;                      // ....save the last time you blinked the LED and then....
     if (ledState == LOW) 
     {
       ledState = HIGH;                                   // ....toggle state
       if (beep == true) beepState = true;
       else beepState = false;
     }
     else 
     {
       ledState = LOW;
       beepState = false;
     }
     digitalWrite(HEARTBEAT_LED, ledState);               // write the HEARTBEAT LED with 'ledState'
     digitalWrite(WINDLASS_LMP, ledState);
     digitalWrite(BUZZER, beepState);                     // write the BUZZER with 'beepState'
   }
 }

  
/********************************************   // synchronize with the rising edge of RC1 pulse, then measure pulse width 
 *   RC1 INPUT INTERUPT SERVICE ROUTINE     *   
 *******************************************/   

void timeRC1pulse()
{
  noInterrupts();
  if ((digitalRead (RC1_INPUT) == HIGH) && (startflag_RC1_Pulse == true))            // if interupt is RISING edge of new PWM pulse, record time of start of PWM pulse
  {
    startofPulse = micros();
    startflag_RC1_Pulse = false;
    interrupts();
    return;
  }
  else                                                                               // if interupt is FALLING edge of PWM pulse, validate and then measure width of PWM pulse
  {
    currentMicros = micros();
    RC1_Value = (currentMicros - startofPulse);
    if ((RC1_Value > max_pulse_length)||(RC1_Value < min_pulse_length))
    {
     bad_RC1_pulse = 1;                                                               // validate pulse width > min_pulse_length
     RC1_Value = 512;  
    }
    else
    {
      RC1_Value = constrain(RC1_Value, 1000, 2000);                                   // CONSTRAIN pulse width to VALID range (1000 - 2000 microsec)
      RC1_Value = map(RC1_Value, 1000, 2000, 0, 1024);                                // finally, SHIFT final RC value to VALID input range (comparable to joystick and sip and puff input range of 0 - 1024)
    }
    startflag_RC1_Pulse = true;                                                       // prepare to receive next PWM pulse
    interrupts();
  }
}

/********************************************   // synchronize with the rising edge of RC2 pulse, then measure pulse width 
 *   RC2 INPUT INTERUPT SERVICE ROUTINE     *   
 *******************************************/   

void timeRC2pulse()
{
  noInterrupts();
  if ((digitalRead (RC2_INPUT) == HIGH) && (startflag_RC2_Pulse == true))            // if interupt is RISING edge of new PWM pulse, recond time of start of PWM pulse
  {
    startofPulse = micros();
    startflag_RC2_Pulse = false;
    interrupts();
    return;
  }
  else                                                                               // if interupt is FALLING edge of PWM pulse, validate and then measure width of PWM pulse
  {
    currentMicros = micros();
    RC2_Value = (currentMicros - startofPulse);
    if ((RC2_Value > max_pulse_length)||(RC2_Value < min_pulse_length))
    {
     bad_RC2_pulse = 1;                                                               // validate pulse width > min_pulse_length
     RC2_Value = 512;  
    }
    else
    {
      RC2_Value = constrain(RC2_Value, 1000, 2000);                                   // CONSTRAIN pulse width to VALID range (1000 - 2000 microsec)
      RC2_Value = map(RC2_Value, 1000, 2000, 0, 1024);                                // finally, SHIFT final RC value to VALID input range (comparable to joystick and sip and puff input range of 0 - 1024) 
    }
    startflag_RC2_Pulse = true;                                                       // prepare to receive next PWM pulse
    interrupts();
  }
}






/********************************************      
 * 
 *         MAIN PROGRAM LOOP                *  MAIN program loop; executes continuously 
 *
 ******************************************/   
void loop()                                
{
  readbattery();                            // battery voltage is polled continuously.  When less than 12.2V, Battery Low LED is flashed.  When less than 11.5V, Battery Low LED is ON and buzzer is ON; HELM and WINCH are disabled.  
  checkInputs();                            // Input commands are polled continuously; for safety, RC commands are PRIORITY ONE and override conflicting joystick and sipp & puff commands for ONE second
  handleInputCommand();                     // when present, input commands are handled by actuating HELM or WINCH as required
  heartbeat_LED();                          // heartbeat LED is blinked at 1 Hz;  buzzer is activated when input command is present
}
