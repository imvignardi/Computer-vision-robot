/* CÓDIGO GENERADO A 07/09/2021
   Ignacio Martínez Vignardi
   Ingeniería Electrónica Industrial y Automática
   Controlador_Rob_v1_0_0
   
   Este código corresponde con el realizado para la fecha de entrega
   del TFG, Diseño y programación de un robot articulado filmográfico 
   con visión por computador  
*/

#include <math.h>
int *controlValues;

const int motor1 = 53;
const int motor2 = 51;
const int motor3 = 49;
const int motor4 = 47;
const int motor5 = 45;
const int motor6 = 43;

const int INITIAL_ANG[6] = {0, 90, 90, 90, 0, 0};
const int LENGTH_12A = 184;
const int LENGTH_12B = 60;
const long LENGTH_23 = 520; //To evade woverflow problems
const int LENGTH_34 = 524;
const int LENGTH_WR = 230;
const int MAX_FREQ = 6333;
const int MAX_SPEED[6] = {18, 18, 18, 18, 62, 62};
const int STEPS_PER_REV = 400;

//Reduction ratio given bye, pulses/rev of driver and reduction of gears
const float reductionRatio1 = 0.9/50;
const float reductionRatio2 = 0.9/50;
const float reductionRatio3 = 0.9/50;
const float reductionRatio4 = 0.9/48;
const float reductionRatio5 = 0.9/16;
const float reductionRatio6 = 0.9/16;

int movFreq = 0;
byte movDir = 0;
byte movType = 0;
bool m5moving = false;
bool m6moving = false;

float remainder[6] = {0,0,0,0,0,0};
volatile float j1a = 0;
volatile float j2a = 90;
volatile float j3a = 90;
volatile float j4a = 90;
volatile float j5a = 0;
volatile float j6a = 0;
float pitch = 90;

double xVal, yVal, zVal = 0;
double saved_array[100][6];
double position_check[6];
int myPositionInArray = 0;
int arrayReader = 0;

int x = LENGTH_12B + LENGTH_34;
int y = 0;
int z = LENGTH_12A + LENGTH_23 - LENGTH_WR;

int slowestMotor = 0;
int i = 0;
int calibrationCount = 0;
volatile int numberOfActiveMotors = 0;
volatile int numberOfFinishedMotors = 0;

const int directionPin[6] = {32, 30, 28, 26, 24, 22};
const int limitSwitch[6] = {7, 8, 9, 10, 11, 12};

//const int limitSwitch = 36; 

byte byte1, byte2, byte3, byte4;
bool done=false;
bool calibrated=false;
bool initialReach=false;
volatile bool firstPass = false;
bool finished = false;
bool set = true;
bool loopMode = false;
bool loopStarted = false;

volatile byte controlByte = 0xFF;
byte previousControlByte = 0x00;

//0x00 standby
//0x01 move J
//0x02 move linear
//
//0x0F calibrate 

volatile uint16_t inputFreq[6] = {0, 0, 0, 0, 0, 0};
volatile int32_t desiredPulses[6] = {0, 0, 0, 0, 0, 0};
volatile uint32_t actualPulses[6] = {0, 0, 0, 0, 0, 0};
volatile uint32_t totalPulses[6] = {0, 0, 0, 0, 0, 0};
uint8_t motorDirection[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(2, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  pinMode(motor5, OUTPUT);
  pinMode(motor6, OUTPUT);

  pinMode(directionPin[0], OUTPUT);
  pinMode(directionPin[1], OUTPUT);
  pinMode(directionPin[2], OUTPUT);
  pinMode(directionPin[3], OUTPUT);
  pinMode(directionPin[4], OUTPUT);
  pinMode(directionPin[5], OUTPUT);

  pinMode(limitSwitch[0], INPUT_PULLUP);
  pinMode(limitSwitch[1], INPUT_PULLUP);
  pinMode(limitSwitch[2], INPUT_PULLUP);
  pinMode(limitSwitch[3], INPUT_PULLUP);
  pinMode(limitSwitch[4], INPUT_PULLUP);
  pinMode(limitSwitch[5], INPUT_PULLUP);

  //ISR_Frequency = (16MHz/(2*prescaler*(cmp+1))) 
  //cmp = (16MHz/2*prescaler*ISR_Freq) - 1

  TCCR0A, TCCR1A, TCCR2A, TCCR3A, TCCR4A, TCCR5A = 0;
  TCCR0B, TCCR1B, TCCR2B, TCCR3B, TCCR4B, TCCR5B = 0;
  TCNT0, TCNT1, TCNT2, TCNT3, TCNT4, TCNT5 = 0;
  //Compare match (cmp)
  OCR0A, OCR1A, OCR2A, OCR3A, OCR4A, OCR5A = 1;
 
  //CTC Mode
  TCCR0A |= (1 << WGM01);
  TCCR1B |= (1 << WGM12);
  TCCR2A |= (1 << WGM21);
  TCCR3B |= (1 << WGM32);
  TCCR4B |= (1 << WGM42);
  TCCR5B |= (1 << WGM52);

  TIMSK0 &= ~(0 << OCIE0A);
  TIMSK1 &= ~(0 << OCIE1A);
  TIMSK2 &= ~(0 << OCIE2A);
  TIMSK3 &= ~(0 << OCIE3A);
  TIMSK4 &= ~(0 << OCIE4A);
  TIMSK5 &= ~(0 << OCIE5A);  
  
  Serial.begin(115200);
 
  while(!Serial) {
    ;
  }

  calibrationCount=500; //VALUES FOR TESTING REST OF THE CODE
  calibrated=true;
  controlByte=0x00;
}

void loop() {
  
  //15 -> 0x0F
  if(calibrated == false && Serial.available()==1) {
    controlByte=Serial.read();
//    digitalWrite(2, HIGH);
  }
  
  while(calibrated==false && controlByte==0x0F) {
    if(Serial.available()>=1) {
      byte emergencyByte=Serial.read();
      calibrated=false;
      calibrationCount=0;
      break;
    }   
    
    if(calibrationCount == 0) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      //Each pulse 1.8º wo\ reducer, 0.036º w\ reducer -> need 140º, total of 3888.89 pulses -> 3889
      desiredPulses[0]=1000000;
      digitalWrite(directionPin[0], HIGH);
      TCCR0A = 0;// set entire TCCR2A register to 0
      OCR0A = 20; //6kHz output for motor
      TCCR0A |= (1 << WGM01);
      TCCR0B |= (1 << CS01) | (1 << CS00);
      TIMSK0 = (1<<OCIE0A);
      calibrationCount++;
//      sei();
    }
        
    if(calibrationCount == 1 && digitalRead(limitSwitch[0])==LOW) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      //Serial.println("1 reached");
      desiredPulses[0]=15556; //return 140º 400 pulses do 360/50 degrees caused by the reducer
      actualPulses[0]=0;
      TIMSK0 = 0;
      TCCR0A, TCCR0B, TCNT0, OCR0A = 0;
      OCR0A = 20; //5.95kHz output for motor
      TCCR0A |= (1 << WGM01);
      digitalWrite(directionPin[0], LOW);
      TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
      controlValues = adjuster(3000*2, 8);   
      OCR0A = controlValues[0];
      TIMSK0 |= (1<<OCIE0A);
      calibrationCount++;
    }   

    if(calibrationCount == 2) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      //Each pulse 1.8º wo\ reducer, 0.036º w\ reducer -> need to move back 55º, total of 1527.78 pulses -> 1528
      desiredPulses[1]=1000000; 
      digitalWrite(directionPin[1], HIGH);
      TCCR1A=0;
      OCR1A = 20;
      TCCR1B |= (1 << WGM12);
      TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
      TIMSK1 |= (1<<OCIE1A);
      calibrationCount++;
    }   

    if(calibrationCount == 3 && digitalRead(limitSwitch[1])==LOW) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      desiredPulses[1]=6667; //return 60º
      actualPulses[1]=0;
      //Serial.println("2 reached");
      TIMSK1 = 0;
      TCCR1A, TCCR1B, TCNT1, OCR1A = 0;
      OCR1A = 20;
      TCCR1B |= (1 << WGM12);
      digitalWrite(directionPin[1], LOW);
      TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
      TIMSK1 |= (1<<OCIE1A);
      calibrationCount++;
    }   

    if(calibrationCount == 4) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      //Each pulse 1.8º wo\ reducer, 0.036º w\ reducer -> need 60º, total of 1666.67 pulses -> 1667 -> CHECK ANGLE
      desiredPulses[2]=1000000;
      digitalWrite(directionPin[2], HIGH);
      TCCR2A = 0;
      OCR2A = 20;
      TCCR2A |= (1 << WGM21);
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);  
      TIMSK2 |= (1<<OCIE2A);
      calibrationCount++;
    }  

    if(calibrationCount == 5 && digitalRead(limitSwitch[2])==LOW) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (50:1 reducer) m1 @ 450 rpm -> 9 rpm output
      desiredPulses[2]=3333; //Check angle, 30º
      actualPulses[2]=0;
      //Serial.println("3 reached");
      TIMSK2 = 0;
      TCCR2A, TCCR2B, TCNT2, OCR2A = 0;
      OCR2A = 20;
      TCCR2A |= (1 << WGM21);
      digitalWrite(directionPin[2], LOW);
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
      controlValues = adjuster(3000*2, 8);   
      TIMSK2 |= (1<<OCIE2A);
      calibrationCount++;
    }   

    if(calibrationCount == 6) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (16:1 reducer) m1 @ 450 rpm -> 28.125 rpm output
      //Each pulse 1.8º wo\ reducer, 0.0375 w\ reducer -> need 45º, total 4800 pulses
      desiredPulses[3]=1000000;
      digitalWrite(directionPin[3], HIGH);
      TCCR3A = 0;
      TCCR3B = 0;
      OCR3A = 20;
      TCCR3B |= (1 << WGM32);
      TCCR3B |= (0 << CS32) | (1 << CS31) | (1 << CS30);
      TIMSK3 |= (1<<OCIE3A);
      calibrationCount++;
    }  

    if(calibrationCount == 7 && digitalRead(limitSwitch[3])==LOW) {
      //Given 400 steps/rev and that at J4 (48:1 reducer)
      desiredPulses[3]=4800; //return 45º, in this case the reducer is 48:1
      actualPulses[3]=0;
      //Serial.println("4 reached");
      TIMSK3 = 0;
      TCCR3A, TCCR3B, TCNT3, OCR3A = 0;
      OCR3A = 20;
      TCCR3B |= (1 << WGM32);
      digitalWrite(directionPin[3], LOW);
      TCCR3B |= (0 << CS32) | (1 << CS31) | (1 << CS30);
      TIMSK3 |= (1<<OCIE3A);
      calibrationCount++;
    }
    
    if(calibrationCount == 8) {
      //Given 400 steps/rev (Have to change if different) and (16:1 reducer) m1 @ 450 rpm -> 28.125 rpm output
      //Each pulse 1.8º wo\ reducer, 0.1125 w\ reducer -> need 90º, 3200 pulses
      desiredPulses[4]=1000000;
      digitalWrite(directionPin[4], HIGH);
      TCCR4A = 0;
      TCCR4B = 0;
      OCR4A = 20;
      TCCR4B |= (1 << WGM42);
      TCCR4B |= (0 << CS42) | (1 << CS41) | (1 << CS40);
      TIMSK4 |= (1<<OCIE4A);
      calibrationCount++;
    }  

    if(calibrationCount == 9 && digitalRead(limitSwitch[4])==LOW) {
      desiredPulses[4]=3200; //return 90º, 16:1 reducer
      actualPulses[4]=0;
      //Serial.println("5 reached");
      TIMSK4 = 0;
      TCCR4A, TCCR4B, TCNT4, OCR4A = 0;
      OCR4A = 20;
      TCCR4B |= (1 << WGM42);
      digitalWrite(directionPin[4], LOW);
      TCCR4B |= (0 << CS42) | (1 << CS41) | (1 << CS40);
      TIMSK4 |= (1<<OCIE4A);
      calibrationCount++;
    }

    if(calibrationCount == 10) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (16:1 reducer) m1 @ 450 rpm -> 28.125 rpm output
      //Each pulse 1.8º wo\ reducer, 0.1125 w\ reducer -> need 45º, total of 400 pulses -> 400 -> CHECK ANGLE
      desiredPulses[5]=1000000;
      digitalWrite(directionPin[5], HIGH);
      TCCR5A, TCCR5B, TCNT5, OCR5A = 0;
      OCR5A = 20;
      TCCR5B |= (1 << WGM52);
      TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
      TIMSK5 |= (1<<OCIE5A);
      calibrationCount++;
    }  

    if(calibrationCount == 11 && digitalRead(limitSwitch[5])==LOW) {
      //Given 400 steps/rev (Have to change if different) and that at J0 (16:1 reducer) m1 @ 450 rpm -> 28.125 rpm output
      desiredPulses[5]=1067; //return 30º, 16:1 reducer
      actualPulses[5]=0;
      //Serial.println("6 reached");
      TIMSK5 = 0;
      TCCR5A, TCCR5B, TCNT5, OCR5A = 0;
      OCR5A = 20;
      TCCR5B |= (1 << WGM52);
      digitalWrite(directionPin[5], LOW);
      TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
      TIMSK5 |= (1<<OCIE5A);
      
      calibrationCount=100;      
    }
    
  }

  if(numberOfFinishedMotors==numberOfActiveMotors && numberOfActiveMotors > 0) {
    numberOfFinishedMotors=0;
    numberOfActiveMotors=0;
    
    TCCR1B |= (1 << WGM12);
    TCCR3B |= (1 << WGM32);
    TCCR4B |= (1 << WGM42);
    TCCR5B |= (1 << WGM52);
    
    finished = true;    
    if(controlByte==0x04) {
      Serial.write(1);
    }
  }

  if(Serial.available()>0 && calibrated==true) {
    //Save previous controlByte, used in one case scenario.
    //receive controlByte
    previousControlByte=controlByte;
    controlByte = Serial.read();
    //Serial.print("cB: ");
    //Serial.println(controlByte);
    firstPass=false;
  }
    
  switch(controlByte) {
    case 0x00:
    //Clear case
    //Disable and empty every timer
    //If we were moving the joint individually then the previous controlByte shall be 0x01 and 
    //we need to do a forward kinematics calculation.
    //firstPass is used to execute the code only once
      if(firstPass==false){
        //Serial.println("Clear cb=0x00");
        for(int i =0; i<=5; i++) {
          inputFreq[i]=0;
          desiredPulses[i]=0;
        }
        controlValues[0] = 0;
        controlValues[1] = 0;

        loopStarted=false;
        loopMode=false;
        
        TCCR0A, TCCR1A, TCCR2A, TCCR3A, TCCR4A, TCCR5A = 0x00;
        TCCR0B, TCCR1B, TCCR2B, TCCR3B, TCCR4B, TCCR5B = 0x00;
        
        TCNT0, TCNT1, TCNT2, TCNT3, TCNT4, TCNT5 = 0;
        OCR0A, OCR1A, OCR2A, OCR3A, OCR4A, OCR5A = 0;
  
        TCCR0A |= (1 << WGM01);
        TCCR1B |= (1 << WGM12);
        TCCR2A |= (1 << WGM21);
        TCCR3B |= (1 << WGM32);
        TCCR4B |= (1 << WGM42);
        TCCR5B |= (1 << WGM52);
        
        TIMSK0 = 0;
        TIMSK1 = 0;
        TIMSK2 = 0;
        TIMSK3 = 0;
        TIMSK4 = 0;
        TIMSK5 = 0;

        firstPass=true;
        finished = false;
        
        for(int i=0; i<=5; i++)
          totalPulses[i]+=actualPulses[i];

        if(previousControlByte==0x01) {
          fk_calc();
          previousControlByte=0x00;          
        }
        else if(previousControlByte==0x0F) {
          //Serial.println("Completed");
          Serial.write(0x01);
          previousControlByte=0x00;
        }
        controlByte=0xFF;
      }
      break;

    case 0x01:
    //This mode moves a single joint.
    //We need the speed of the motor and the direction
    //Only one joint must move
      if(firstPass==false) {
        //Serial.println("Joint mode");
        while(Serial.available()!=4) {
        //Wait till all bytes reach
        ;
        }    
        byte1 = Serial.read();
        byte2 = Serial.read();
        movFreq=0;
        movFreq = byteToInt16(byte1, byte2);
        //Serial.println(movFreq);
        movType = Serial.read();
        movDir = Serial.read();

        for(int i=0; i<=5; i++) {
          if((int)movType==i+1) {
            inputFreq[i] = movFreq;
            if(movDir==0x01)
              digitalWrite(directionPin[i], HIGH);
            else if(movDir==0xFF)
              digitalWrite(directionPin[i], LOW);
          }
          else {
            inputFreq[i] = 0;
            digitalWrite(directionPin[i],LOW);
          }
        }
        firstPass=true;
        set=false;
      }
      break;
        
    case 0x02:
    //Linear movement through x, y and z.
    //We need the max speed of the movement (movFreq), the coordinate to move (movType) and if we increment or decrement it (movDir).
    //The coordinates are represented by bytes:
    //0x01 for x, 0x02 for y and 0x03 for z.
    //We do inverse kinematics for moving linearly. More about it in the function obtainJointVariation
      if(firstPass == false) { 
        while(Serial.available()!=4) {
        //Wait till all bytes reach
        ;
        }    
        byte1 = Serial.read();
        byte2 = Serial.read();
        movFreq = byteToInt16(byte1, byte2);
        movType = Serial.read();
        movDir = Serial.read();
      }
      if(firstPass == false || finished == true) {
        numberOfActiveMotors=0;
        obtainJointVariation(movType, movFreq, movDir);
        firstPass = true;
        set=false;
      }
      break;

    case 0x03:
    //In this case we save the current position into an array.
    //We just need the speed that we want for reaching such position.
      if(firstPass == false) { 
        while(Serial.available()!=2) {
        //Wait till all bytes reach
        ;
        }    
        byte1 = Serial.read();
        byte2 = Serial.read();
        movFreq = byteToInt16(byte1, byte2);
        save_position();
      }
      firstPass = true;
      break;

    case 0x04: 
      //Begin movement case
      //FIRST MOVE TO STARTING POSITION
      //THEN PULSE TO CAMERA FOCUS
      //WAIT FOR USER CONFIRMATION
      //THEN LIGHT (or not) AND CAMERA START
      //AFTERWARDS START TRAJECTORY
      if(firstPass==false) {
        while(Serial.available()!=2) {
          //Wait till all bytes reach
          ;
          }  
        movType = Serial.read();
        byte looping = Serial.read();
        if(looping==0x01)
          loopMode=true;
        else
          loopMode=false;       
        
      }
      if(firstPass == false || finished == true) {
        firstPass = true;
        if(arrayReader==0 && saved_array[0][3]!=0 && saved_array[1][3]!=0) {
          Serial.write(0x01);
          obtainJointVariation(movType, saved_array[arrayReader][3], 0);
          arrayReader++;
        }
        else if((arrayReader==0 && saved_array[0][3]==0) || saved_array[1][3]==0) {
          if(saved_array[0][3]==0){
            Serial.write(0xE1); //No points in array
            controlByte=0x00;
          }
          else
            Serial.write(0xE2); //Not enough points in array
            controlByte=0x00;
          break; //break case
        }
        else if(arrayReader==1) {
          if(loopStarted = false) {
            until_focus_confirm();
            digitalWrite(12, HIGH); //START CAMERA
            delayMicroseconds(10000); //10ms DELAY, delayMicroseconds DOESNT USE TIMERS
            digitalWrite(12, LOW); //NOT NEEDED HIGH AFTER STARTED
          }
          obtainJointVariation(movType, saved_array[arrayReader][3], 0);
          arrayReader++;
        }
        else if(arrayReader>1 && arrayReader<(sizeof(saved_array)/sizeof(saved_array[0]))) {
          obtainJointVariation(movType, saved_array[arrayReader][3], 0);
          arrayReader++;
        }
        else if(arrayReader>=(sizeof(saved_array)/sizeof(saved_array[0]))) {
          if(loopMode=true) {
            loopStarted=true;
            arrayReader=0;
          }
          else
            controlByte=0x00;
        }
        finished=false;
      }
      break;

    case 0x05: 
      //Pause
      //Just masks and unmasks interrupts.
      //We need to mask and not disable interrupts so we can receive serial data.
      //The pause mode is self locking and wont execute any code (stuck in while clause) until the user inputs unpause
      while(Serial.available()!=1) {
      //Wait till all bytes reach
      ;
      }    
      byte pause = Serial.read(); 
      if(pause == 0x01){
        noInterrupts();
        TIMSK0=0;
        TIMSK1=0;
        TIMSK2=0;
        TIMSK3=0;
        TIMSK4=0;
        TIMSK5=0;
        interrupts();
      }
      else {
        noInterrupts();
        TIMSK0 |= (1<<OCIE0A);
        TIMSK1 |= (1<<OCIE1A);
        TIMSK2 |= (1<<OCIE2A);
        TIMSK3 |= (1<<OCIE3A);
        TIMSK4 |= (1<<OCIE4A);
        TIMSK5 |= (1<<OCIE5A);
        controlByte=0xFF; //So we dont lock the program again
        interrupts();
      }
      break;

    case 0x06: //STOP
      //Since we control at every moment the angle of the motors, we can retrieve x, y and z coordinates after stopped
      //by doing a simple forward kinematics calculation.
      //Afterwards the controlByte is set to 0 erasing every timer and reseting some variables
      fk_calc();
      controlByte=0x00;
      break;

    case 0x07:  //ERASE
      //Erases all positions saved in array
      for(int i=0; i<(sizeof(saved_array)/sizeof(saved_array[0])); i++) {
        for(int j=0; j<6; j++)
          saved_array[i][j] = 0;
      }
      controlByte=0x00;
      break;

   case 0x10: //FOLLOW OBJECT
      //IN PROGRESS
      while(Serial.available()!=2) {
        //Wait till all bytes reach
        ;
      } 
      int motor = 0;
      movType = Serial.read();
      movDir = Serial.read();
      if(movType = 0x05 && movDir!=0 && m5moving==false) {
        inputFreq[4] = 3000;
        inputFreq[5] = 0;
        m5moving=true;
      }
      else if (movType = 0x05 && movDir==0) {
        TCCR4B = 0;
        TCCR4A = 0;
        TCNT4 = 0;
        OCR4A = 0;
        TCCR4B |= (1<<WGM42);
        m5moving=false;
      }
      else if (movType = 0x06 && movDir!=0 && m6moving==false) {
        inputFreq[5] = 3000;
        inputFreq[4] = 0;
        m6moving=true;
      }
      else if (movType = 0x06 && movDir==0) {
        TCCR5B = 0;
        TCCR5A = 0;
        TCNT5 = 0;
        OCR5A = 0;
        TCCR5B |= (1<<WGM52);
        m6moving=false;
      }
      else if(movType=0x00) {
        m5moving=false;
        m6moving=false;
        controlByte = 0x00;      
      }
      if(movDir = 0x01)
        digitalWrite(directionPin[motor], HIGH);
      else if(movDir = 0xFF)
        digitalWrite(directionPin[motor], LOW);
      set=false;
      break;


    case 0xAA:
    //Special method to check if there are saved positions
    //Only accesible through realterm or other serial terminals
      if(firstPass == false) { 
        print_my_positions();
        firstPass=true;
      }
      set=false;
      break;
      
    default:
    //Default case
      break;
  }
  
  
  if(set==false) {
    //Execute this part of the code if the motor frequencies are not set.
    //Frequencies are set after executing 0x01, 0x02 and 0x04 cases. (Movement methods)
    //We call the adjuster function which returns the values for setting the timer; the count to OCRnA and the preescaler.
    //After the settings the timer is unmasked.
    //Everything has to be done without interrupts enabled, so the motors start at the same time. 
    noInterrupts();
    ////Serial.println("Enabling motors");
    if(inputFreq[0]>0) {   
      TCCR0A = 0;
      controlValues = adjuster(inputFreq[0], 8);   
      OCR0A = controlValues[0];
      TCCR0A |= (1 << WGM01);
//      //Serial.println(controlValues[0]);
//      //Serial.println(controlValues[1]);
      if(controlValues[1]==1) {
        TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
      }
      else if(controlValues[1]==8) {
        TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);
      }
      else if(controlValues[1]==64) {
        TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
      }
      else if(controlValues[1]==256) {
        TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00);
      }
      else if(controlValues[1]==1024) {
        TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
      }
      TIMSK0 |= (1 << OCIE0A);
    }
  
    if(inputFreq[1]>0) { 
      TCCR1A = 0;
      controlValues = adjuster(inputFreq[1], 16);
      OCR1A = controlValues[0];
      TCCR1B |= (1 << WGM12);
      if(controlValues[1]==1) {
        TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
      }
      else if(controlValues[1]==8) {
        TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
      }
      else if(controlValues[1]==64) {
        TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
      }
      else if(controlValues[1]==256) {
        TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
      }
      else if(controlValues[1]==1024) {
        TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
      } 
      TIMSK1 |= (1 << OCIE1A);
    }
    
    if(inputFreq[2]>0) { 
      TCCR2A = 0;
      controlValues = adjuster(inputFreq[2], 8);
      OCR2A = controlValues[0];
      TCCR2A |= (1 << WGM21);
      if(controlValues[1]==1) {
        TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);
      }
      else if(controlValues[1]==8) {
        TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20);
      }
      else if(controlValues[1]==64) {
        TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);
      }
      else if(controlValues[1]==256) {
        TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20);
      }
      else if(controlValues[1]==1024) {
        TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
      }
      TIMSK2 |= (1 << OCIE2A);
    }
        
    if(inputFreq[3]>0) { 
      TCCR3A = 0;
      controlValues = adjuster(inputFreq[3], 16);
      OCR3A = controlValues[0];
      TCCR3B |= (1 << WGM12);
      if(controlValues[1]==1) {
        TCCR3B |= (0 << CS32) | (0 << CS31) | (1 << CS30);
      }
      else if(controlValues[1]==8) {
        TCCR3B |= (0 << CS32) | (1 << CS31) | (0 << CS30);
      }
      else if(controlValues[1]==64) {
        TCCR3B |= (0 << CS32) | (1 << CS31) | (1 << CS30);
      }
      else if(controlValues[1]==256) {
        TCCR3B |= (1 << CS32) | (0 << CS31) | (0 << CS30);
      }
      else if(controlValues[1]==1024) {
        TCCR3B |= (1 << CS32) | (0 << CS31) | (1 << CS30);
      }
      TIMSK3 |= (1 << OCIE3A);
    }
        
    if(inputFreq[4]>0) {   
      TCCR4A = 0;
      controlValues = adjuster(inputFreq[4], 16); 
      OCR4A = controlValues[0];
      TCCR4B |= (1 << WGM12);
      if(controlValues[1]==1) {
        TCCR4B |= (0 << CS42) | (0 << CS41) | (1 << CS40);
      }
      else if(controlValues[1]==8) {
        TCCR4B |= (0 << CS42) | (1 << CS41) | (0 << CS40);
      }
      else if(controlValues[1]==64) {
        TCCR4B |= (0 << CS42) | (1 << CS41) | (1 << CS40);
      }
      else if(controlValues[1]==256) {
        TCCR4B |= (1 << CS42) | (0 << CS41) | (0 << CS40);
      }
      else if(controlValues[1]==1024) {
        TCCR4B |= (1 << CS42) | (0 << CS41) | (1 << CS40);
      }
      TIMSK4 |= (1 << OCIE4A);
    }
        
        
    if(inputFreq[5]>0) { 
      TCCR5A = 0;
      controlValues = adjuster(inputFreq[5], 16);
      OCR5A = controlValues[0];
      TCCR5B |= (1 << WGM12);
      if(controlValues[1]==1) {
        TCCR5B |= (0 << CS52) | (0 << CS51) | (1 << CS50);
      }
      else if(controlValues[1]==8) {
        TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50);
      }
      else if(controlValues[1]==64) {
        TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
      }
      else if(controlValues[1]==256) {
        TCCR5B |= (1 << CS52) | (0 << CS51) | (0 << CS50);
      }
      else if(controlValues[1]==1024) {
        TCCR5B |= (1 << CS52) | (0 << CS51) | (1 << CS50);
      }
      TIMSK5 |= (1 << OCIE5A);
    }
    set=true;
    interrupts();
  }
}


uint16_t byteToInt16(byte byte1, byte byte2) {
  //Conversion to int16 (frequency)
  return (uint16_t)byte2 << 8 | (uint16_t)byte1;
}

uint32_t byteToInt32(byte byte1, byte byte2, byte byte3, byte byte4) {
  //Conversion to int32 (desiredPulses), deprecated
  return (uint32_t)byte4 << 24 | (uint32_t)byte3 << 16 | (uint32_t)byte2 << 8 | (uint32_t)byte1;
}

void obtainJointVariation(byte linearType, int maxFreq, byte way) {
  //Inverse kinematics function.
  //The linearType (previously addressed as movType) adds the direction since a 1mm resolution is wanted. 
  //For example, if the robot is at x=500mm and the user wants to decrease it (way == movDir == -1), if we add one to the other the result is x=499mm.
  //This leaves the program with a 1mm resolution.
  //linearType==0x04 is for the case in which the user wants to start the robots motion after saving n number of positions.
  //In such case the x, y and z are saved in an array. Granting the capability to do a linear movement from the current position to the next one ssaved.
  double deltaJ[6] = {0, 0, 0, 0, 0, 0};
  int8_t variation = 0;
  variation = (int8_t)way;
  if(linearType == 1)
    x+=1;
  else if(linearType == 2)
    y+=1;
  else if(linearType == 3)
    z+=1;
  else if(linearType == 4) {
    x=saved_array[arrayReader][0];
    y=saved_array[arrayReader][1];
    z=saved_array[arrayReader][2];
    double aJ5 = saved_array[arrayReader][4];
    double aJ6 = saved_array[arrayReader][5];
  }
  else if(linearType == 10) {
    //For calculating if the movement is possible
    x = xVal;
    y = yVal;
    z = zVal;
  }

  //The inverse kinematic is done by trigonometry.
  //Check end of degree document for more information about the equations.
  double aJ1 = degrees(atan2(y, x)); //FIRST ANGLE!!
  double mod = sqrt(pow(x, 2) + pow(y, 2)); //Perspective change
  double l46x = cos(radians(pitch))*LENGTH_WR;
  double l24x = mod - l46x - LENGTH_12B;
  double l46y = sin(radians(pitch))*LENGTH_WR;
  double l24y = z - LENGTH_12A + l46y; //520
  double hip = sqrt(pow(l24y, 2) + pow(l24x, 2));//739
  double alpha = degrees(atan2(l24y, l24x));//44.73
  double beta = degrees(acos((pow(LENGTH_23, 2) - pow(LENGTH_34, 2) + pow(hip, 2))/(2*LENGTH_23*hip))); //Escalene cosine theorem45.16
  double aJ2 = alpha + beta; //ANGLE JOINT SHOULDER!!89.89
  double aJ3 = degrees(acos((pow(LENGTH_23, 2) + pow(LENGTH_34, 2) - pow(hip, 2))/(2*LENGTH_23*LENGTH_34)));//90.12
  double aJ4 = 360 - pitch - aJ2 - aJ3; //ANGLE JOINT WRIST CHECK!!89.99

//        aJ5 and aJ6 dont affect X, Y and Z as designed so
//        the movement of them is based purely on joint movement
  if(linearType!=10) {
    deltaJ[0] = j1a - aJ1;
    deltaJ[1] = j2a - aJ2;//0.11
    deltaJ[2] = j3a - aJ3;//-0.12
    deltaJ[3] = j4a - aJ4;//0.01
    if(controlByte==0x04) {
      deltaJ[4] = j3a - aJ3;//-0.12
      deltaJ[5] = j4a - aJ4;//0.01
    }
       
    if(controlByte==0x02) {
      //Serial.println("New angles");
      //Serial.println(aJ1);
      //Serial.println(aJ2);
      //Serial.println(aJ3);
      //Serial.println(aJ4);
      //Serial.println("Variation");
      //Serial.println(deltaJ[0]);
      //Serial.println(deltaJ[1]);
      //Serial.println(deltaJ[2]);
      //Serial.println(deltaJ[3]);
      j1a = aJ1;
      j2a = aJ2;
      j3a = aJ3;
      j4a = aJ4;
    }
    
    //Since microcontrollers and any digital electronic device works discretly, the accuracy of the motors movement has to be improved.
    //by saving some of the floating point data in a remainder. Afterwards this variable must be used to recover the lost step in the motor.
    //This is particularly important since the resolution is so high (1mm) that some motors need a movement of less than 1 pulse.
    //Serial.println("Desired Pulses");
    desiredPulses[0] = round((deltaJ[0]/reductionRatio1)*100000)/100000;
    remainder[0]= remainder[0]+(desiredPulses[0]-trunc(desiredPulses[0]));
    desiredPulses[1] = round((deltaJ[1]/reductionRatio2)*100000)/100000;
    remainder[1]=remainder[1]+(desiredPulses[1]-trunc(desiredPulses[1]));
    desiredPulses[2] = round((deltaJ[2]/reductionRatio3)*100000)/100000;
    remainder[2]=remainder[2]+(desiredPulses[2]-trunc(desiredPulses[2]));
    desiredPulses[3] = round((deltaJ[3]/reductionRatio4)*100000)/100000;
    remainder[3]=remainder[3]+(desiredPulses[3]-trunc(desiredPulses[3]));
    desiredPulses[4] = round((deltaJ[4]/reductionRatio5)*100000)/100000;
    remainder[4]=remainder[4]+(desiredPulses[4]-trunc(desiredPulses[4]));
    desiredPulses[5] = round((deltaJ[5]/reductionRatio6)*100000)/100000;
    remainder[5]=remainder[5]+(desiredPulses[5]-trunc(desiredPulses[5]));

    //Here the pulses are truncated and converted to positive.
    //Also, if the remainder is higher than 0.95 an extra pulse is added and the remainder is reset.
    
    for(int i = 0; i<=5; i++) { 
 
      if(desiredPulses[i]<0) {
          motorDirection[i] = -1;
          digitalWrite(directionPin[i], LOW);
      }
      else if(desiredPulses[i]>0) {
          digitalWrite(directionPin[i], HIGH);
          motorDirection[i] = 1;
      }
      desiredPulses[i]=trunc(abs(desiredPulses[i]));
                      
      if(abs(remainder[i])>0.95) {
          desiredPulses[i]=desiredPulses[i]+1;
          remainder[i] = 0;
      }
      //Serial.println(desiredPulses[i]);
    }
  
    float slow=0;
          
    for(int i = 0; i<=5; i++) {
        if(desiredPulses[i] > slow) {
            slow=desiredPulses[i];
            slowestMotor = i;
        }
    }
    //Serial.println("Max Frequency");
    //Serial.println(maxFreq);
    inputFreq[slowestMotor] = maxFreq;
    float time_to_complete = slow/inputFreq[slowestMotor]; 
    //Serial.println("Time to complete");
    //Serial.println(time_to_complete); 

    //Serial.println("Frequencies");
    for(int i = 0; i<=5; i++) {
      inputFreq[i] = (int)(desiredPulses[i]/time_to_complete); 
      if(inputFreq[i]>0) {      
        numberOfActiveMotors++;
      }
//      if(inputFreq[i]<5){
//        ////Serial.println(inputFreq[i]);
//      }
//      else {
//        //Serial.println(inputFreq[i]);
//      }
    }
    
    //Serial.print("Number of active motors: ");
    //Serial.println(numberOfActiveMotors);
  }
  else if(linearType==10) {
    position_check[0] = aJ1;
    position_check[1] = aJ2;
    position_check[2] = aJ3;
    position_check[4] = aJ4;
  }
 
}

void fk_calc(){
  double theta, alpha, d, a = 0;
  double TransMatrix[4][4] = {{cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)}, 
                             {sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)}, 
                             {0,sin(alpha),cos(alpha),d}, 
                             {0,0,0,1}}; 
                                                     
  double j1ap=(j1a*(PI/180));
  double j2ap=(j2a*(PI/180)-PI/2)+PI/2;
  double j3ap=(j3a*(PI/180)-PI/2)-PI/2;
  double j4ap=(j4a*(PI/180)-PI/2);
  double j5ap=(j5a*(PI/180));
  double T[5][4][4] ={{{cos(j1ap),-sin(j1ap)*cos(PI/2),sin(j1ap)*sin(PI/2),LENGTH_12B*cos(j1ap)}, {sin(j1ap),cos(j1ap)*cos(PI/2),-cos(j1ap)*sin(PI/2),LENGTH_12B*sin(j1ap)}, {0,sin(PI/2),cos(PI/2),LENGTH_12A}, {0,0,0,1}},
                     {{cos(j2ap),-sin(j2ap)*cos(0),sin(j2ap)*sin(0),LENGTH_23*cos(j2ap)}, {sin(j2ap),cos(j2ap)*cos(0),-cos(j2ap)*sin(0), LENGTH_23*sin(j2ap)}, {0,sin(0),cos(0),0}, {0,0,0,1}},
                     {{cos(j3ap),-sin(j3ap)*cos(0),sin(j3ap)*sin(0),LENGTH_34*cos(j3ap)}, {sin(j3ap),cos(j3ap)*cos(0),-cos(j3ap)*sin(0),LENGTH_34*sin(j3ap)}, {0,sin(0),cos(0),0}, {0,0,0,1}},
                     {{cos(j4ap),-sin(j4ap)*cos(-PI/2),sin(j4ap)*sin(-PI/2),0*cos(j4ap)}, {sin(j4ap),cos(j4ap)*cos(-PI/2),-cos(j4ap)*sin(-PI/2),0*sin(j4ap)}, {0,sin(-PI/2),cos(-PI/2),0}, {0,0,0,1}},
                     {{cos(j5ap),-sin(j5ap)*cos(PI/2),sin(j5ap)*sin(PI/2),0*cos(j5ap)}, {sin(j5ap),cos(j5ap)*cos(PI/2),-cos(j5ap)*sin(PI/2),0*sin(j5ap)}, {0,sin(PI/2),cos(PI/2),-LENGTH_WR}, {0,0,0,1}}};
  
  
  double T05[4][4] = {{0,0,0,0},{0,0,0,0}};
  double aux[4][4] = {{0,0,0,0},{0,0,0,0}};
  
  for(int pos=0; pos<=3; pos++) {
    for(int i=0; i<=3; i++) {
      for(int j=0; j<=3; j++) {
        for(int k=0; k<=3; k++) {
          if(pos==0) {
            T05[i][j]+=T[pos][i][k]*T[pos+1][k][j];
          }
          else if(pos>0) {
            aux[i][j]+=T05[i][k]*T[pos+1][k][j];
          }
        }
      }
    }

    //T05 reassignment
    if(pos>0){
      for(int i=0; i<=3; i++) {
        for(int j=0; j<=3; j++) {
           T05[i][j]=aux[i][j];
           aux[i][j]=0;
        }
      }
    }
  }
  //Serial.print("x: ");
  //Serial.println(T05[0][3]);
  x = T05[0][3];
  //Serial.print("y: ");
  //Serial.println(T05[1][3]);
  y = T05[1][3];
  //Serial.print("z: ");
  //Serial.println(T05[2][3]);
  z = T05[2][3];
}

void save_position() {
  fk_calc(); //COMMENT TO TEST
  saved_array[myPositionInArray][0] = x; //COMMENT THE BLOCK TO TEST
  saved_array[myPositionInArray][1] = y; //
  saved_array[myPositionInArray][2] = z; //
  saved_array[myPositionInArray][3] = movFreq; //
  saved_array[myPositionInArray][4] = j5a;
  saved_array[myPositionInArray][5] = j6a;

//  saved_array[0][0] = 447; //UNCOMMENT TO TEST VALUES
//  saved_array[0][1] = -375; //
//  saved_array[0][2] = 474; //
//  saved_array[0][3] = 633; //
//  saved_array[1][0] = -447; //UNCOMMENT TO TEST VALUES
//  saved_array[1][1] = 375; //
//  saved_array[1][2] = 474; //
//  saved_array[1][3] = 633; //
//  myPositionInArray = 1; //
//   int a,b,c=0;
//   a=447+447;
//   b=-375-375;
//   c=0; //bidimensional
  
  if(myPositionInArray==0) {
    myPositionInArray++; //Pos=0 is the initial position
    Serial.write(0x01);
  }
  else if(myPositionInArray>0) {
    xVal, yVal, zVal = 0;
    int num=0;
    do {   
      //Serial.println(num);
      if(num==0 && saved_array[myPositionInArray-1][0] == saved_array[myPositionInArray][0] && saved_array[myPositionInArray-1][1] == saved_array[myPositionInArray][1] && saved_array[myPositionInArray-1][2] == saved_array[myPositionInArray][2]) {
        Serial.write(0xE3); //Error, trying to save same point
        controlByte=0x00;
        break;
      }   
      num++;
      xVal = (saved_array[myPositionInArray-1][0]+num)/a;
      yVal = ((num/(saved_array[myPositionInArray][0]-saved_array[myPositionInArray-1][0]))*(saved_array[myPositionInArray][1]-saved_array[myPositionInArray-1][1]))+saved_array[myPositionInArray-1][1];
      zVal = ((num/(saved_array[myPositionInArray][0]-saved_array[myPositionInArray-1][0]))*(saved_array[myPositionInArray][2]-saved_array[myPositionInArray-1][2]))+saved_array[myPositionInArray-1][2];
      obtainJointVariation(10,0,0);
      
      //j5 and j6 dont affect the x, y and z value
      if(position_check[0]>140 || position_check[0]<140 || position_check[1]<35 || position_check[1]>140 || position_check[2]<40 || position_check[2]>180 || position_check[3]<45 || position_check[3]>125) {
        //Serial.println("Impossible trajectory");
        Serial.write(0xE4); //Error, cant realize movement
        controlByte=0x00;
        break;
      }  
      else if(xVal == saved_array[myPositionInArray][0]) {
        //End reached without problems
        Serial.write(0x01);   
        myPositionInArray++; 
      }
    }while(num<=saved_array[myPositionInArray][0]-saved_array[myPositionInArray-1][0]);
  }
}

void print_my_positions() {
  for(int i = 0; i<myPositionInArray; i++) {
    Serial.print("X value: ");
    Serial.println(saved_array[myPositionInArray][0]);
    Serial.print("Y value: ");
    Serial.println(saved_array[myPositionInArray][1]);
    Serial.print("Z value: ");
    Serial.println(saved_array[myPositionInArray][2]);
    Serial.print("Frequency value: ");
    Serial.println(saved_array[myPositionInArray][3]);
  }
  if(myPositionInArray==0) {
    Serial.println("0 positions");
  }  
}

void until_focus_confirm() {
  bool confirmed = false;
  byte check = 0;
  Serial.write(0x01);
  while(confirmed==false) {
    if(Serial.available()>0) {
      check = Serial.read();
    }
    if(check==0x01) { //START FOCUS/ REFOCUS
      digitalWrite(14, LOW);
      digitalWrite(14, HIGH);
    }
    else if(check==0x02) { //FOCUSED
      digitalWrite(14, LOW);
      confirmed=true;
    }
  }
}

int * adjuster(int x, int bits) {
  bool done = false;
  float cmp = 0.0;
  static int arr[2] = {0, 0};
  
  cmp = (16000000L/(2*1L*x))-1;
//  //Serial.print("prescaler 1: ");
//  Serial.println(cmp);
//  Serial.println(" ");
  if((cmp<256 && bits==8) || (cmp<65535 && bits==16)) {
    cmp = (int)cmp;
    arr[0] = cmp;
    arr[1] = 1;
  }
  else{
    cmp = (16000000L/(2*8L*x))-1L;
//    Serial.print("prescaler 8: ");
//    Serial.println(cmp);
//    Serial.println(" ");
    if((cmp<256 && bits==8) || (cmp<65535 && bits==16)) {
      cmp = (int)cmp;
      arr[0] = cmp;
      arr[1] = 8;
    }
    else {
      cmp = (16000000L/(2*64L*x))-1L;
//      Serial.print("prescaler 64: ");
//      Serial.println(cmp);
//      Serial.println(" ");
      if((cmp<256 && bits==8) || (cmp<65535 && bits==16)) {
        cmp = (int)cmp;
        arr[0] = cmp;
        arr[1] = 64;
      }
      else {
        cmp = (16000000L/(2*256L*x))-1L;
//        Serial.print("prescaler 256: ");
//        Serial.println(cmp);
//        Serial.println(" ");
        if((cmp<256 && bits==8) || (cmp<65535 && bits==16)) {
          cmp = (int)cmp;
          arr[0] = cmp;
          arr[1] = 256;
        }
        else {
          cmp = (16000000L/(2*1024L*x))-1L;
          if((cmp<256 && bits==8) || (cmp<65535 && bits==16)) {
            cmp = (int)cmp;
            arr[0] = cmp;
            arr[1] = 1024;
          }
          else {
            //Fail, do something
            ;
          }
        }
      }
    }
  }
  return arr;
}

ISR(TIMER0_COMPA_vect){
  if(actualPulses[0]<desiredPulses[0] || (controlByte==0x01 && j1a<=140 && j1a>=-140) || (controlByte==0x01 && ((j1a>=140 && movDir==0xFF) || (j1a<=-140 && movDir==0x01)))) {
    digitalWrite(motor1, !digitalRead(motor1));
    actualPulses[0]++;
    if((movDir==0x01 && actualPulses[0]%2!=0 && controlByte==0x01) || (motorDirection[0]==1 && actualPulses[0]%2!=0 && controlByte==0x04)) {
      j1a-=reductionRatio1;
      //Serial.println(j1a);
    }
    else if((movDir==0xFF && actualPulses[0]%2!=0 && controlByte==0x01) || (motorDirection[0]==-1 && actualPulses[0]%2!=0 && controlByte==0x04)) {
      j1a+=reductionRatio1;
      //Serial.println(j1a);
    }
  }  
  else if(controlByte==0x01 && ((j1a>=140 && movDir==0x01) || (j1a<=-140 && movDir==0xFF))){
    if(movDir==0x01)
      totalPulses[0] = totalPulses[0] + actualPulses[0];
    else if(movDir==0xFF)
      totalPulses[0] = totalPulses[0] - actualPulses[0];
    actualPulses[0]=0;
    TCCR0B=0;
  }
  else if(actualPulses[0]==desiredPulses[0] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[0] = totalPulses[0] + actualPulses[0];
    actualPulses[0]=0;
    numberOfFinishedMotors++;
    TCCR0B=0;
  }
  else if(actualPulses[0] == desiredPulses[0] && controlByte==0x0F) {
    TCCR0B, TCCR0A=0;
  }
  
}

ISR(TIMER1_COMPA_vect){
  if(actualPulses[1]<desiredPulses[1] || (controlByte==0x01 && j2a<=140 && j2a>=35) || (controlByte==0x01 && ((j2a>=140 && movDir==0xFF) || (j2a<=35 && movDir==0x01)))) {
    digitalWrite(motor2, !digitalRead(motor2));
    actualPulses[1]++;
    if((movDir==0x01 && actualPulses[1]%2!=0 && controlByte==0x01) || (motorDirection[1]==1 && actualPulses[1]%2!=0 && controlByte==0x04)) {
      j2a-=reductionRatio2;  
//      Serial.println(j2a);
    }
    else if((movDir==0xFF && actualPulses[1]%2!=0 && controlByte==0x01)|| (motorDirection[1]==-1 && actualPulses[1]%2!=0 && controlByte==0x04)) {
      j2a+=reductionRatio2; 
//      Serial.println(j2a);
    }
  }  
  else if(actualPulses[1]==desiredPulses[1] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[1] = totalPulses[1] + actualPulses[1];
    actualPulses[1]=0;
    numberOfFinishedMotors++;
    TCCR1B=0;
  }
  else if(controlByte==0x01 && ((j1a>=140 && movDir==0x01) || (j1a<=-140 && movDir==0xFF))){
    totalPulses[0] = totalPulses[0] + actualPulses[0];
    actualPulses[0]=0;
    TCCR1B=0;
  }
  else if(actualPulses[1] == desiredPulses[1] && controlByte==0x15) {
    TCCR1B, TCCR1A=0;
  }
}

ISR(TIMER2_COMPA_vect){
  if(actualPulses[2]<desiredPulses[2] || (controlByte==0x01 && j3a<=180 && j3a>=40) || (controlByte==0x01 && ((j3a>=180 && movDir==0xFF) || (j3a<=40 && movDir==0x01)))) { 
    //CHECK IN DESIGN THE LIMITS (180º TO 40º)
    digitalWrite(motor3, !digitalRead(motor3));
    actualPulses[2]++;
    if((movDir==0x01 && actualPulses[2]%2!=0 && controlByte==0x01) || (motorDirection[2]==1 && actualPulses[2]%2!=0 && controlByte==0x04)) {
      j3a-=reductionRatio3;  
//      Serial.println(j2a);
    }
    else if((movDir==0xFF && actualPulses[2]%2!=0 && controlByte==0x01)|| (motorDirection[2]==-1 && actualPulses[2]%2!=0 && controlByte==0x04)) {
      j3a+=reductionRatio3; 
//      Serial.println(j2a);
    }
  }  
  else if(actualPulses[2]==desiredPulses[2] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[2] = totalPulses[2] + actualPulses[2];
    actualPulses[2]=0;
    numberOfFinishedMotors++;
    TCCR2B=0;
  }
  else if(controlByte==0x01 && ((j3a>=180 && movDir==0x01) || (j3a<=40 && movDir==0xFF))){
    totalPulses[2] = totalPulses[2] + actualPulses[2];
    actualPulses[2]=0;
    TCCR2B=0;
  }
  else if(actualPulses[2] == desiredPulses[2] && controlByte==0x15) {
    TCCR2B, TCCR2A=0;
  }
}

ISR(TIMER3_COMPA_vect){
  if(actualPulses[3]<desiredPulses[3] || (controlByte==0x01 && j4a<=125 && j4a>=45) || (controlByte==0x01 && ((j4a>=125 && movDir==0xFF) || (j4a<=45 && movDir==0x01)))) {
    //CHECK IN DESIGN THE LIMITS (125º TO 45º)
    digitalWrite(motor4, !digitalRead(motor4));
    actualPulses[3]++;
    if((movDir==0x01 && actualPulses[3]%2!=0 && controlByte==0x01) || (motorDirection[3]==1 && actualPulses[3]%2!=0 && controlByte==0x04)) {
      j4a-=reductionRatio4;  
//      Serial.println(j2a);
    }
    else if((movDir==0xFF && actualPulses[3]%2!=0 && controlByte==0x01) || (motorDirection[3]==-1 && actualPulses[3]%2!=0 && controlByte==0x04)) {
      j4a+=reductionRatio4; 
//      Serial.println(j2a);
    }
  }  
  else if(actualPulses[3]==desiredPulses[3] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[3] = totalPulses[3] + actualPulses[3];
    actualPulses[3]=0;
    numberOfFinishedMotors++;
    TCCR3B=0;
  }
  else if(controlByte==0x01 && ((j4a>=180 && movDir==0x01) || (j4a<=40 && movDir==0xFF))){
    totalPulses[3] = totalPulses[3] + actualPulses[3];
    actualPulses[3]=0;
    TCCR3B=0;
  }
  else if(actualPulses[3] == desiredPulses[3] && controlByte==0x15) {
    TCCR3B, TCCR3A=0;
  }
}

ISR(TIMER4_COMPA_vect){
  if(actualPulses[4]<desiredPulses[4] || (controlByte==0x01 && j5a<=90 && j5a>=-90) || (controlByte==0x01 && ((j5a>=90 && movDir==0xFF) || (j5a<=-90 && movDir==0x01)))) {
    digitalWrite(motor5, !digitalRead(motor5));
    actualPulses[4]++;
    if((movDir==0x01 && actualPulses[4]%2!=0 && controlByte==0x01) || (motorDirection[4]==1 && actualPulses[4]%2!=0 && controlByte==0x04)) {
      j5a-=reductionRatio5;  
//      Serial.println(j2a);
    }
    else if((movDir==0xFF && actualPulses[4]%2!=0 && controlByte==0x01) || (motorDirection[4]==-1 && actualPulses[4]%2!=0 && controlByte==0x04)) {
      j5a+=reductionRatio5; 
//      Serial.println(j2a);
    }
  }  
  else if(actualPulses[4]==desiredPulses[4] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[4] = totalPulses[4] + actualPulses[4];
    actualPulses[4]=0;
    numberOfFinishedMotors++;
    TCCR4B=0;
  }
  else if(controlByte==0x01 && ((j5a>=90 && movDir==0x01) || (j5a<=-90 && movDir==0xFF))){
    totalPulses[4] = totalPulses[4] + actualPulses[4];
    actualPulses[4]=0;
    TCCR4B=0;
  }
  else if(actualPulses[4] == desiredPulses[4] && controlByte==0x15) {
    TCCR4B, TCCR4A=0;
  }
}

ISR(TIMER5_COMPA_vect){
  if(actualPulses[5]<desiredPulses[5] || (controlByte==0x01 && j6a<=30 && j6a>=-90) || (controlByte==0x01 && ((j6a>=30 && movDir==0xFF) || (j6a<=-90 && movDir==0x01)))) {
    digitalWrite(motor6, !digitalRead(motor6));
    actualPulses[5]++;
    if((movDir==0x01 && actualPulses[5]%2!=0 && controlByte==0x01) || (motorDirection[5]==1 && actualPulses[5]%2!=0 && controlByte==0x04)) {
      j6a+=reductionRatio6;  
//      Serial.println(j2a);
    }
    else if((movDir==0xFF && actualPulses[5]%2!=0 && controlByte==0x01) || (motorDirection[5]==-1 && actualPulses[5]%2!=0 && controlByte==0x04)) {
      j6a-=reductionRatio6; 
//      Serial.println(j2a);
    }
  }  
   else if(actualPulses[5]==desiredPulses[5] && (controlByte==0x02 || controlByte==0x04)){
    totalPulses[5] = totalPulses[5] + actualPulses[5];
    actualPulses[5]=0;
    numberOfFinishedMotors++;
    TCCR5B=0;
  }
  else if(controlByte==0x01 && ((j5a>=90 && movDir==0x01) || (j5a<=-90 && movDir==0xFF))){
    totalPulses[5] = totalPulses[5] + actualPulses[5];
    actualPulses[5]=0;
    TCCR5B=0;
  }
  else if(actualPulses[5] == desiredPulses[5] && controlByte==0x0F) {
    j1a = 0;
    j2a = 90;
    j3a = 90;
    j4a = 90;
    j5a = 0;
    j6a = 0;
    previousControlByte = controlByte;
    calibrated=true;
    controlByte=0x00; //The calibration has finished so we change the controlByte    
  }
}
