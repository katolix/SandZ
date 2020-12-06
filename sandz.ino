#include <AccelStepper.h>
#include <TM1637Display.h>

// User defined parameters
//===================================================================================================//
#define MOTOR_Z_STEP_PER_REV 400 // Number of motor steps per one rotation
#define SCREW_Z 100 // Number of microns per rev of the hand feed screw

//==============================================================================
// Assigned board pins to buttons

// Port D
//#define Seg7DispCLK 0 // 7-segment indicator CLK pin
//#define Seg7DispDIO 1 // 7-segment indicator DIO pin

#define EncPinA 2 // Encoder channel A interrupt pin
#define EncPinB 3 // Encoder channel B interrupt pin

#define buttonPinM 4 // Mul button pin
#define buttonPinR 5 // Run button pin
#define buttonPinJ 6 // Jog button pin
#define buttonPinS 7 // Set button pin

// Port B
int lightPinR = 8; // Run button pin ???
int lightPinJ = 9; // Jog button pin ???
int lightPinS = 10; // Set button pin ???


#define motorStepPin 11 // Stepper driver STEP (PUL) pin
#define motorDirPin 12 // Stepper driver DIR pin
#define motorEnaPin 13 // Stepper driver ENA pin

// Port C
#define binX1 A0
#define binX2 A1
#define binY1 A2
#define binY2 A3
#define Seg7DispCLK A5 // 7-segment indicator CLK pin
#define Seg7DispDIO A4 // 7-segment indicator DIO pin


//==============================================================================
#define Motor_Z_SetPulse()     PORTB &= ~(1<<3)     // Pin11 0 (PB3)
#define Motor_Z_RemovePulse()  PORTB |= (1<<3)      // Pin11 1
#define Motor_Z_InvertPulse()  PORTB ^= (1<<3)      // Pin11
#define Read_Z_State           (PINB & (1<<3))

#define Motor_Z_CW()           PORTB &= ~(1<<4)    // Pin12 0 (PB4)
#define Motor_Z_CCW()          PORTB |= (1<<4)     // Pin12 1

#define Motor_Z_Enable()   do {PORTB |= (1<<5); _delay_ms(120);} while(0)   // Pin13 1 (PB5)
#define Motor_Z_Disable()      PORTB &= ~(1<<5)                             // Pin13 0
#define Read_Z_Ena_State       (PINB & (1<<5))

#define CW               0
#define CCW              1
#define ON               1
#define OFF              0

typedef struct
{ 
   uint8_t bit0 : 1;
   uint8_t bit1 : 1;
   uint8_t bit2 : 1;
   uint8_t bit3 : 1;
   uint8_t bit4 : 1;
   uint8_t bit5 : 1;
   uint8_t bit6 : 1;
   uint8_t bit7 : 1;
}FLAG;

#define Motor_Z_Dir        ((volatile FLAG*)_SFR_MEM_ADDR(GPIOR0))->bit0    // CW-0, CCW-1


//==============================================================================
// State of system
#define ST_ONRUN 0 // Motor running in actions
#define ST_ONPAU 1 // Action pause
#define ST_ONJOG 2 // Motor jogging
#define ST_ONSET 3 // Setting distance
#define ST_ONPRG 4 // Changing parameters of run

// Types of modes

  enum run_mode
  {
  mod_RUN, // Run mode
  mod_JOG, // Jog mode
  mod_SET, // Seting distances
  mod_PRG, // Re-programming parameters
  MODES_NUM
  };
enum button
{
  bt_Non,
  bt_A,
  bt_B,
  bt_C,
};

// Global flags
byte _state = ST_ONSET; //After power-ON, we are in a Set mode
bool _update_display = true;


//==============================================================================
// Button reading, including debounce without delay function declarations
byte _buttonOldState_S = HIGH;  // assume switch open because of pull-up resistor
byte _buttonOldState_J = HIGH;  // assume switch open because of pull-up resistor
byte _buttonOldState_R = HIGH;  // assume switch open because of pull-up resistor
byte _buttonOldState_M = HIGH;  // assume switch open because of pull-up resistor


unsigned long _buttonPressTime_S;  // when the switch last changed state
unsigned long _buttonPressTime_J;  // when the switch last changed state
unsigned long _buttonPressTime_R;  // when the switch last changed state
unsigned long _buttonPressTime_M;  // when the switch last changed state

unsigned long _pauseBlinkTime = 0; 
#define BLINK_PERIOD 500 // ms

const unsigned long debounceTime = 10;  // milliseconds

//==============================================================================
// Axe and motor position
long zAxePos = 0;
long zMotorPos = 0;
long zNullPos = 0;
//==============================================================================
// Rotary encoder
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile long encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
long oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
long newEncPos = 0;
volatile byte readingEnc = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

volatile int16_t axeStepIdx = 0;
int16_t mulAxeSteps = 1;
byte currentAxeStepMul = 0;
const uint16_t MAX_AXE_STEPS = 9999; 

//==============================================================================
// Interupt pin definitiones for direct port access
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
#define PORT_ENC PIND
#define PORT_ENC_MSK 0xC
#define PIN23_MSK B00001100
#define PIN_2_MSK B00000100
#define PIN_3_MSK B00001000
#elif defined(ARDUINO_AVR_MEGA2560)
#define PORT_ENC PINE
#define PORT_ENC_MSK 0x30
#define PIN23_MSK B00110000
#define PIN_2_MSK B00010000
#define PIN_3_MSK B00100000
#elif defined(ARDUINO_SAM_DUE)
//Due specific code
#else
#error Unsupported hardware
#endif

/*
#define Hand_Ch_A             (PIND & (1<<2))
#define Hand_Ch_B             (PIND & (1<<3))
*/
/*
#define Ena_INT_Hcoder()      do { EIFR   = (1<<INTF0); EIMSK |=  (1<<INT0);} while(0)
#define Disa_INT_Hcoder()         EIMSK &= ~(1<<INT0)
*/

//==============================================================================
//Rotary encoder interrupt service routine for one encoder pin

void EncPinA_ISR() {
  cli(); //stop interrupts happening before we read pin values
  readingEnc = PORT_ENC & PORT_ENC_MSK; // read all eight pin values then strip away all but pinA and pinB's values
  if (readingEnc == PIN23_MSK && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingEnc == PIN_2_MSK) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void EncPinB_ISR() {
  cli(); //stop interrupts happening before we read pin values
  readingEnc = PORT_ENC & PORT_ENC_MSK; //read all eight pin values then strip away all but pinA and pinB's values
  if (readingEnc == PIN23_MSK && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingEnc == PIN_3_MSK) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
/*
ISR(INT0_vect)
{
   if (!Hand_Ch_A)
   {
      if (!Hand_Ch_B) {encoderPos --;}
   }
   
   else
   {
      if (!Hand_Ch_B) {encoderPos ++;}
   }

}
*/
/*
// INT0 = A channel
void activateInterrupt0 ()
  {
  EICRA &= ~(bit(ISC00) | bit (ISC01));  // clear existing flags
  EICRA |=  bit (ISC01);    // set wanted flags (falling level interrupt)
  EIFR   =  bit (INTF0);    // clear flag for interrupt 0
  EIMSK |=  bit (INT0);     // enable it
  }  // end of activateInterrupt0

void deactivateInterrupt0 ()
  {
  EIMSK &= ~bit (INT0);     // disable it
  }  // end of deactivateInterrupt0
*/

signed char UpdateAxePosFromEnc(){
    signed char encDir = 0;

///     deactivateInterrupt0 ();
     newEncPos = encoderPos;
///     activateInterrupt0 ();

    if (oldEncPos == newEncPos)
      return encDir; //Do nothing,do not update enc derection 

    if (newEncPos > oldEncPos) //incremental enc rotation
    {
      encDir = 1;
      }
    else if (newEncPos < oldEncPos) //decrementak enc rotation
    {
      encDir = -1;
      }
    oldEncPos = newEncPos;
    return encDir;
}


//AccelStepper stepper(AccelStepper::DRIVER, motorStepPin, motorDirPin);
TM1637Display display(Seg7DispCLK, Seg7DispDIO);


void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  
  digitalWrite(motorEnaPin, HIGH);// Debug restart system
  
  // Rotary encoder section of setup
/*
activateInterrupt0 ();
*/
  
  pinMode(EncPinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(EncPinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(EncPinA), EncPinA_ISR, RISING); // set an interrupt on EncPinA, looking for a rising edge signal and executing the "EncPinA_ISR" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(EncPinB), EncPinB_ISR, RISING); // set an interrupt on EncPinB, looking for a rising edge signal and executing the "EncPinB_ISR" Interrupt Service Routine (below)

  //=================================
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorStepPin, OUTPUT);
  pinMode(motorEnaPin, OUTPUT);
  //=================================
  pinMode(buttonPinR, INPUT_PULLUP);
  pinMode(buttonPinJ, INPUT_PULLUP);
  pinMode(buttonPinS, INPUT_PULLUP);
  pinMode(buttonPinM, INPUT_PULLUP);
  pinMode(lightPinR, OUTPUT);
  pinMode(lightPinJ, OUTPUT);
  pinMode(lightPinS, OUTPUT);

  pinMode(binX1, INPUT_PULLUP);
  pinMode(binX2, INPUT_PULLUP);
  pinMode(binY1, INPUT_PULLUP);
  pinMode(binY2, INPUT_PULLUP);
  pinMode(Seg7DispDIO, OUTPUT);
  pinMode(Seg7DispCLK, OUTPUT);

  display.setBrightness(1, true);
  display.showNumberDec(0, true);
  
  Motor_Z_Dir = CW;
  Motor_Z_RemovePulse();

//  stepper.setMaxSpeed(2000);
//  stepper.setAcceleration(1000);
  digitalWrite(motorDirPin, HIGH);
  digitalWrite(motorStepPin, HIGH);

  _state == ST_ONSET; // power-on mode is SET
  UpdateButtonLights();
  
}

void DoPause()
{
  if (_state != ST_ONPAU)
  return;
  if(millis() > _pauseBlinkTime + BLINK_PERIOD)
  {
    _pauseBlinkTime = millis();
    if(digitalRead(lightPinR) == HIGH)
      digitalWrite(lightPinR,LOW);
    else
      digitalWrite(lightPinR,HIGH);
  } 
}
void RunMotorByEnc()
{
  ///if (_state == ST_ONJOG){
  signed char encDir = UpdateAxePosFromEnc();
  if (encDir != 0){
    zAxePos = newEncPos * /* Scale */ MOTOR_Z_STEP_PER_REV / SCREW_Z;
    uint16_t stepsToGo = abs(zMotorPos - zAxePos);

      if (encDir > 0){ //incremental enc rotation
          Motor_Z_CW();
          //for (uint16_t i=0; i < stepsToGo; i++){
            zMotorPos++;
            Motor_Z_SetPulse();
            delayMicroseconds(2);
            Motor_Z_RemovePulse();
          //}
      }
      else if (encDir < 0){ //decrementak enc rotation
          Motor_Z_CCW();
          stepsToGo = abs(zMotorPos - zAxePos);
          //for (uint16_t i=0; i < stepsToGo; i++){
            zMotorPos++;
            Motor_Z_SetPulse();
            delayMicroseconds(2);
            Motor_Z_RemovePulse();
          //}
      }
     //Serial.println(newEncPos);
    }
  ///}// endif (_state == ST_ONJOG)
}

void UpdateButtonLights()
{
  if (_state == ST_ONRUN)
  {
    digitalWrite(lightPinR,HIGH);
    digitalWrite(lightPinJ,LOW);
    digitalWrite(lightPinS,LOW);}
  else if (_state == ST_ONPAU)
  {
    digitalWrite(lightPinR,HIGH);
    digitalWrite(lightPinJ,LOW);
    digitalWrite(lightPinS,LOW);}

  else if( _state == ST_ONJOG )
  {
    digitalWrite(lightPinR,LOW);
    digitalWrite(lightPinJ,HIGH);
    digitalWrite(lightPinS,LOW);}
  else if (_state == ST_ONSET)
  {
    digitalWrite(lightPinR,LOW);
    digitalWrite(lightPinJ, LOW);
    digitalWrite(lightPinS, HIGH);}
  else if( _state == ST_ONPRG )
  {
    digitalWrite(lightPinR,LOW);
    digitalWrite(lightPinJ,HIGH);
    digitalWrite(lightPinS,HIGH);}  
}

void ButtonR(bool buttonPressed) // Run button
{
  // In the RUN mode, pressing Run button initiates PAUSE mode
  if (_state == ST_ONRUN)
  {
     if (buttonPressed){
        Serial.println("Button R pressed in state ST_ONRUN");
        _state = ST_ONPAU;
     }
     else { // Button released. 
        Serial.println("Button R released in state ST_ONRUN");
     }
  }
  // In the PAUSE mode, pressing Run button initiates RUN mode
  else if( _state == ST_ONPAU )
  {
     if (buttonPressed){
        Serial.println("Button R pressed in state ST_ONPAU");
        _state = ST_ONRUN;
     }
     else { // Button released. 
        Serial.println("Button R released in state ST_ONPAU");
     }
  }
  // In the SET or JOG modes, pressing Run button initiates RUN mode
  else if ((_state == ST_ONSET) || (_state == ST_ONJOG))
  {
     if (buttonPressed){
        Serial.println("Button R pressed in state ST_ONSET");
        _state = ST_ONRUN;
     }
     else { // Button released. 
        Serial.println("Button R released in state ST_ONSET");
     }
  }
  else if( _state == ST_ONPRG )
  {
     if (buttonPressed)
        Serial.println("Button R pressed in state ST_ONPRG");
     else { // Button released. 
        Serial.println("Button R released in state ST_ONPRG");
     }
  }
  UpdateButtonLights();
  _update_display = true;

}

void ButtonJ(bool buttonPressed) // Jog button
{
  if ((_state == ST_ONSET) || (_state == ST_ONJOG)) {
    if (buttonPressed)
    {
      Serial.println("Button J pressed in state ST_ONSET or ST_ONJOG");
      _state = ST_ONJOG;
    }
    else // button released
    {
    }
  }
// No action in ST_ONRUN mode

   UpdateButtonLights();
   _update_display = true;
}

void ButtonS(bool buttonPressed) // Set button
{
  // In the SET, JOR or POUSE modes, Set button initiate Set mode
  // No action in ST_ONRUN mode
  if ((_state == ST_ONSET) || (_state == ST_ONJOG) || ( _state == ST_ONPAU )) {
    if (buttonPressed){
        _state = ST_ONSET;
    }
    else{ // button released
      axeStepIdx = 0;
    }
  }
  else if (_state == ST_ONPRG){
  }

  UpdateButtonLights();
   _update_display = true;
}
void ButtonM(bool buttonPressed) // Multiplier button
{

  if ((_state == ST_ONSET) || (_state == ST_ONJOG)) {
    if (buttonPressed)
    {
      Serial.println("Button M pressed in state ST_ONSET or ST_ONJOG");
      currentAxeStepMul++;
      if (currentAxeStepMul > 3) 
        currentAxeStepMul = 0;
      mulAxeSteps = 1;
      for (int i=0; i<currentAxeStepMul; i++){
      mulAxeSteps = 10*mulAxeSteps;}

      Serial.println(mulAxeSteps);
    }
    else // button released
    {
      digitalWrite(motorEnaPin, LOW);// Debug restart system, release the LED
      // Set a corresponding dot position in the 7-segmet display !!!
    }
  }

// No action in ST_ONRUN mode

   _update_display = true;
}



// Handle buttons Run/Jog/Set
void StartButtons()
{
  byte buttonState = digitalRead (buttonPinS);
  if (buttonState != _buttonOldState_S) {
    if (millis () - _buttonPressTime_S >= debounceTime) { // debounce
      _buttonPressTime_S = millis ();  // when we closed the switch
      _buttonOldState_S =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("button S closed"); // DEBUGGING: print that button has been closed
        ButtonS(true);
      }
      else {
        Serial.println ("button S opened"); // DEBUGGING: print that button has been opened
        ButtonS(false);
      }
    }  // end if debounce time up
  } // end of state change

  buttonState = digitalRead (buttonPinJ);
  if (buttonState != _buttonOldState_J) {
    if (millis () - _buttonPressTime_J >= debounceTime) { // debounce
      _buttonPressTime_J = millis ();  // when we closed the switch
      _buttonOldState_J =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("button J closed"); // DEBUGGING: print that button has been closed
        ButtonJ(true);
      }
      else {
        Serial.println ("button J opened"); // DEBUGGING: print that button has been opened
        ButtonJ(false);
      }
    }  // end if debounce time up
  } // end of state change

  buttonState = digitalRead (buttonPinR);
  if (buttonState != _buttonOldState_R) {
    if (millis () - _buttonPressTime_R >= debounceTime) { // debounce
      _buttonPressTime_R = millis ();  // when we closed the switch
      _buttonOldState_R =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("button R closed"); // DEBUGGING: print that button has been closed
        ButtonR(true);
      }
      else {
        Serial.println ("button R opened"); // DEBUGGING: print that button has been opened
        ButtonR(false);
      }
    }  // end if debounce time up
  } // end of state change

  buttonState = digitalRead (buttonPinM);
  if (buttonState != _buttonOldState_M) {
    if (millis () - _buttonPressTime_M >= debounceTime) { // debounce
      _buttonPressTime_M = millis ();  // when we closed the switch
      _buttonOldState_M =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("button M closed"); // DEBUGGING: print that button has been closed
        ButtonM(true);
      }
      else {
        Serial.println ("button M opened"); // DEBUGGING: print that button has been opened
        ButtonM(false);
      }
    }  // end if debounce time up
  } // end of state change

}


void doSteps(signed char encDir, uint16_t numSteps)
{
  if (encDir == 0) 
    return;
    if ( (encDir > 0) )//incremental enc rotation
    {
      digitalWrite(motorDirPin, HIGH);
      for(int i=0; i<numSteps; i++){
        digitalWrite(motorStepPin, LOW);
        delayMicroseconds(2);
        digitalWrite(motorStepPin, HIGH);}
    }
    else if ( (encDir < 0)  ) //decrementak enc rotation
    {
      digitalWrite(motorDirPin, LOW);
      for(int i=0; i<numSteps; i++){
        digitalWrite(motorStepPin, LOW);
        delayMicroseconds(2);
        digitalWrite(motorStepPin, HIGH);}
    }
}

void loop() {

  StartButtons();
  RunMotorByEnc();// Should be in Jog state
  DoPause(); //Should be in PAUSE state
}
