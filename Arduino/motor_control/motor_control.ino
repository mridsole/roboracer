/**
 * @author Kelly Lynch
 * @date 2018-06-28
 * @brief Motor Control for the DRC Competition
 *
 * @bugs None known
 */

 
/* -- Defined Constants -- */
#define MAX_LEN 20
#define WHEEL_RADIUS 0.060
#define HALF_BASE 0.120
#define INPUT_VOLTAGE 12.6
#define MAX_VOLTAGE 6
#define CONTROL_RATIO MAX_VOLTAGE/INPUT_VOLTAGE
#define MAX_PWM 255*CONTROL_RATIO
#define MAX_RPM 295
#define MAX_RAD_PER_S 30.8923

 /* -- Serial input info -- */
uint8_t incomingByte = 0;
uint8_t buff[MAX_LEN];

/* -- Output Pins -- */
int frontRight = 3;
int frontLeft = 3;
int backRight = 3;
int backLeft = 3;
int dummyPin = 3;


/**
* @brief Setup
* Sets up serial.
* 
* @return Does not return
*/
void setup() {
  Serial.begin(250000);
  Serial.setTimeout(5);
}


/**
* @brief Main Loop
* Constantly checks for input strings and controls what to do.
* 
* @return Does not return
*/
void loop() {
  String inString;
  String instructionString = "<>";

  /* Read a string terminated by "\n\r" */
  if(Serial.available() > 0){
    inString = Serial.readString();
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
  /* Set PWM pins */
  frontRight = 10;
  frontLeft = 11;
  backRight = 5;
  backLeft = 6;
  
  /* Set all PWM Pins to 0% */
  analogWrite(frontRight, 0);
  analogWrite(backRight, 0);
  analogWrite(frontLeft, 0);
  analogWrite(backLeft, 0);
}


/**
* @brief Disable Motors
* Sets PWM pins to disallow instructions
* 
* @return Does not return
*/
void disable(){
  /* Set all PWM Pins to 0% */
  analogWrite(frontRight, 0);
  analogWrite(backRight, 0);
  analogWrite(frontLeft, 0);
  analogWrite(backLeft, 0);
  
  /* Redirect all PWM outputs to pin D3 */
  frontRight = dummyPin;
  frontLeft = dummyPin;
  backRight = dummyPin;
  backLeft = dummyPin;
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
    Serial.print("SCALE RATIO: ");
    Serial.print(scaleRatio);
    Serial.print("\n\r");
  }

  /* Convert angular velocities to PWM Signals */
  int leftPWM = map(angularSpeedLeft, 0, MAX_RAD_PER_S, 0, MAX_PWM);
  int rightPWM = map(angularSpeedRight, 0, MAX_RAD_PER_S, 0, MAX_PWM);

  /* Write to PWM Pins */
  analogWrite(frontRight, rightPWM);
  analogWrite(backRight, rightPWM);
  analogWrite(frontLeft, leftPWM);
  analogWrite(backLeft, leftPWM);
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
  Serial.println("CALIBRATING");
}
