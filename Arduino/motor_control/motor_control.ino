/**
 * @author Kelly Lynch
 * @date 2018-06-28
 * @brief Motor Control for the DRC Competition
 *
 * @bugs None known
 */

/* -- Includes -- */
#include <Servo.h>
#include <EnableInterrupt.h>
 
/* -- Defined Constants -- */
#define MAX_LEN 20
#define WHEEL_RADIUS 0.060
#define HALF_BASE 0.120
#define INPUT_VOLTAGE 12.6
#define MAX_VOLTAGE 6
#define CONTROL_RATIO MAX_VOLTAGE/INPUT_VOLTAGE
#define MAX_PWM 1024*CONTROL_RATIO
#define MAX_RPM 295
#define MAX_RAD_PER_S 30.8923
#define MAX_SERVO 500*CONTROL_RATIO + 1000
#define MIN_SERVO 1000

 /* -- Serial input info -- */
uint8_t incomingByte = 0;
uint8_t buff[MAX_LEN];

/* -- Output Pins -- */
int frontRight = 8;
int frontLeft = 8;
int backRight = 8;
int backLeft = 8;
int dummyPin = 8;

/* Timout Variables */
const unsigned long TIMEOUT = 1000;
unsigned long timeElapsed = 0;
unsigned long previousTime;

/* Servos (Wheels) */
Servo FL;
Servo FR;
Servo BL;
Servo BR;

/* Wheel Velocity Pins */
float FLVel = 0;
float FRVel = 0;
float BLVel = 0;
float BRVel = 0;
unsigned long prevTime = 0;
unsigned long currTime = 0;
unsigned int frontLeftCountPrev = 0;
unsigned int frontRightCountPrev = 0;
unsigned int backLeftCountPrev = 0;
unsigned int backRightCountPrev = 0;
unsigned int frontLeftCountNow = 0;
unsigned int frontRightCountNow = 0;
unsigned int backLeftCountNow = 0;
unsigned int backRightCountNow = 0;

/**
* @brief Setup
* Sets up serial.
* 
* @return Does not return
*/
void setup() {
  Serial.begin(250000);
  Serial.setTimeout(5);

  previousTime = millis();

  /* Setup Servos (Wheels) */
  FL.attach(9);
  FR.attach(10);
  BL.attach(11);
  BR.attach(12);

  /* Interrupt functions */
  enableInterrupt(4, frontLeftChange,CHANGE);
  enableInterrupt(5, frontRightChange,CHANGE);
  enableInterrupt(6, backLeftChange,CHANGE);
  enableInterrupt(7, backRightChange,CHANGE);

  prevTime = millis();

}


/* -- Interrupt Functions -- */
volatile unsigned int frontLeftCount = 0;
volatile unsigned int frontRightCount = 0;
volatile unsigned int backLeftCount = 0;
volatile unsigned int backRightCount = 0;
/* Read rising edges */
void frontLeftChange(){
  if(bitRead(PORTD, 4) == 0){
      frontLeftCount++;
  }
}

void frontRightChange(){
  if(bitRead(PORTD, 5) == 0){
      frontRightCount++;
  }
}

void backLeftChange(){
  if(bitRead(PORTD, 6) == 0){
      backLeftCount++;
  }
}

void backRightChange(){
  if(bitRead(PORTD, 7) == 0){
      backRightCount++;
  }
}


/**
* @brief Main Loop
* Constantly checks for input strings and controls what to do.
* 
* @return Does not return
*/
void loop() {
//  Serial.print(frontLeftCount);
//  Serial.print(",");
//  Serial.print(frontRightCount);
//  Serial.print(",");
//  Serial.print(backLeftCount);
//  Serial.print(",");
//  Serial.print(backRightCount);
//  Serial.print("\n\r");
  /* Calculate Velocities of wheels */
  currTime = millis();
  frontLeftCountNow = frontLeftCount;
  frontRightCountNow = frontRightCount;
  backLeftCountNow = backLeftCount;
  backRightCountNow = backRightCount;

  float FLRad = 2*PI*(frontLeftCountNow-frontLeftCountPrev);
  float FRRad = 2*PI*(frontRightCountNow-frontRightCountPrev);
  float BLRad = 2*PI*(backLeftCountNow-backLeftCountPrev);
  float BRRad = 2*PI*(backRightCountNow-backRightCountPrev);
  
  FLVel = FLRad/(((float)(currTime-prevTime))/1000);
  FRVel = FRRad/(((float)(currTime-prevTime))/1000);
  BLVel = BLRad/(((float)(currTime-prevTime))/1000);
  BRVel = BRRad/(((float)(currTime-prevTime))/1000);
  
  frontLeftCountPrev = frontLeftCountNow;
  frontRightCountPrev = frontRightCountNow;
  backLeftCountPrev = backLeftCountNow;
  backRightCountPrev = backRightCountNow;
  prevTime = currTime;

  Serial.print(FLVel);
  Serial.print(",");
  Serial.print(FRVel);
  Serial.print(",");
  Serial.print(BLVel);
  Serial.print(",");
  Serial.print(BRVel);
  Serial.print("\n\r");
//  
  /* First check if there has been a timeout */
  if(millis()-previousTime > TIMEOUT){
    disable(); 
  }
  
  String inString;
  String instructionString = "<>";

  /* Read a string terminated by "\n\r" */
  if(Serial.available() > 0){
    inString = Serial.readString();
    previousTime = millis();
  }

  /* Manipulate string to determine action */
  if(inString.length() > 0){
    if(inString.startsWith("<") && inString.endsWith(">")){
      /* Prepare String for interpretation */
      inString.replace("<", "");
      inString.replace(">", "");

      /* Determine action by first letter in string */
      /* Enable Motors */
      if(inString.startsWith("E")){
        enable();
      /* Disable Motors*/
      }else if(inString.startsWith("D")){
        disable();
      /* Move Robot */
      }else if(inString.startsWith("M")){

        /* Extract arc radius and linear velocity */
        inString.replace("M", "");
        
        /* Convert from String Object to String */
        String arcString;
        String velString;
        char buf[MAX_LEN];
        inString.toCharArray(buf, MAX_LEN);
        char *p = buf;
        char *str;
        
        /* Split string by ',' */
        int cmdCount = 0;
        while ((str = strtok_r(p, ",", &p)) != NULL){ 
          if(cmdCount == 0){
            arcString = String(str);
            cmdCount++;
          }else{
            velString = String(str);
            cmdCount = 0;
          }
        }
        
        /* Convert strings to floats */
        float inputArcRadius = arcString.toFloat();
        float inputVelocity = velString.toFloat();

        /* Move the robot */
        moveRobot(inputArcRadius, inputVelocity);
      /* Calibrate the ESCs */
      }else if(inString.startsWith("C")){
        calibrate();
      }      
    }
  }
}


/**
* @brief Enable Motors
* Sets PWM pins to allow for instructions
* 
* @return Does not return
*/
void enable(){
  /* Set Servo pins */
//  frontLeft = 9;
//  frontRight = 10;
//  backLeft = 11;
//  backRight = 12;

  /* Attach */
//  FL.attach(frontLeft);
//  FR.attach(frontRight);
//  BL.attach(backLeft);
//  BR.attach(backRight);
  /* And set to 0 */
  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
}


/**
* @brief Disable Motors
* Sets PWM pins to disallow instructions
* 
* @return Does not return
*/
void disable(){
  /* Set to 0 */
  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  /* and Detach*/
//  FL.detach();
//  FR.detach();
//  BL.detach();
//  BR.detach();
}


/**
* @brief Move Robot
* Moves the robot based on the input arc radius and linear velocity
* 
* @return Does not return
*/
void moveRobot(float arcRadius, float velocity){
  /* Convert instantaneous velocity to angular velocity */
  float arcVelocity = velocity/arcRadius;
  
  /* Convert radius and velocity into wheel angular velocities */
  float angularSpeedLeft = (arcVelocity*(arcRadius+HALF_BASE))/(WHEEL_RADIUS);
  float angularSpeedRight = (arcVelocity*(arcRadius-HALF_BASE))/(WHEEL_RADIUS);

  /* If out of achievable range, scale values down */
  if(angularSpeedLeft > MAX_RAD_PER_S || angularSpeedRight > MAX_RAD_PER_S){
    float scaleRatio;
    if(angularSpeedLeft > angularSpeedRight){
      scaleRatio = MAX_RAD_PER_S/angularSpeedLeft;
    }else{
      scaleRatio = MAX_RAD_PER_S/angularSpeedRight;
    }

    angularSpeedLeft = angularSpeedLeft * scaleRatio;
    angularSpeedRight = angularSpeedRight * scaleRatio;
//    Serial.print("SCALE RATIO: ");
//    Serial.print(scaleRatio);
//    Serial.print("\n\r");
  }

  /* Convert angular velocities to PWM Signals */
  int leftServoGoal = map(angularSpeedLeft, 0, MAX_RAD_PER_S, MIN_SERVO, MAX_SERVO);
  int rightServoGoal = map(angularSpeedRight, 0, MAX_RAD_PER_S, MIN_SERVO, MAX_SERVO);

//  Serial.print("LEFT: ");
//  Serial.print(angularSpeedLeft);
//  Serial.print("\t");
//  Serial.print(leftServoGoal);
//  Serial.print("\n\r");
//  Serial.print("RIGHT: ");
//  Serial.print(angularSpeedRight);
//  Serial.print("\t");
//  Serial.print(rightServoGoal);
//  Serial.print("\n\r");
    FL.writeMicroseconds(leftServoGoal);
    FR.writeMicroseconds(rightServoGoal);
    BL.writeMicroseconds(leftServoGoal);
    BR.writeMicroseconds(rightServoGoal);

}


/**
* @brief Calibrate ESCs
* Sends instructions to calibrate the ESCs
* 
* @return Does not return
* 
* @todo everything
*/
void calibrate(){
  Serial.println("Calibrating...");
  FL.writeMicroseconds(1500);
  FR.writeMicroseconds(1500);
  BL.writeMicroseconds(1500);
  BR.writeMicroseconds(1500);
  delay(6000);
  FL.writeMicroseconds(800);
  FR.writeMicroseconds(800);
  BL.writeMicroseconds(800);
  BR.writeMicroseconds(800);
  Serial.println("Press Any Key to Continue When Beeping Stops");
  while(Serial.available() == 0){}
}
