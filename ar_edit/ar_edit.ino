#include <SharpIR.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>
#include <stdio.h>
#include <math.h>

// Legacy include
/*
  #include <EnableInterrupt.h>
  #include <RunningMedian.h>
*/

// Definitions
// encoderpin1a
#define outputA 11 // 3
// encoderpin1b
#define outputB 13 // 5
// encoderpin2a
#define inputA 3 // 11
// encoderpin2b
#define inputB 5 // 13

#define irFR A0
#define irFM A1
#define irFL A2
#define irLB A3
#define irLM A4
#define irLong A5

#define modelS 1080
#define modelR 20150

/*
  #define PWM_MIN (60)
  #define PWM_MAX (100)
  #define turnErrorRate 1.11
  #define distanceErrorRate 1.03\
*/

// Specifications

DualVNH5019MotorShield* md;
SharpIR sFR(irFR, modelS); // instantiation of sensors //DO NOT CHANGE
SharpIR sFM(irFM, modelS);
SharpIR sFL(irFL, modelS);
SharpIR sLM(irLM, modelS);
SharpIR sLB(irLB, modelS);
SharpIR sLong(irLong, modelR);

/*
  SharpIR sr1 =  SharpIR(irFR, modelS); //instantiation of sensors //DO NOT CHANGE
  SharpIR sr2 =  SharpIR(irFM, modelS);
  SharpIR sr3 =  SharpIR(irFL, modelS);
  SharpIR sr4 =  SharpIR(irRM, modelS);
  SharpIR sr5 =  SharpIR(irLM, modelS);
  SharpIR sr6 = SharpIR(irRLong, modelR);
*/

// Fields
int numOfGrid;
int degreeOfTurn;
int forwardCount = 0;
volatile int counterA = 0;
volatile int counterB = 0;
double prevTick = counterA;
volatile double counterC = 0;
double difference;
double Setpoint, Input, Output;
int calibrateMother = 0;

String data;
char instruction;
int charposition = 5;

/*
  long volatile ticksmoved1 = 0;
  long volatile ticksmoved2 = 0;

  unsigned long volatile currentpulsetime1 = 0.0;
  unsigned long volatile currentpulsetime2 = 0.0;

  unsigned long previoustime1 = 0;
  unsigned long previoustime2 = 0;
*/

/*
  /* -------------- PID --------------------
  // Right wheel PID
  double rkp = 0.4033; //0.5633 and 0.4033
  double rki = 0.8219;
  double rkd = 0;

  // Left Wheel PID
  double lkp = 0.8184; //8124 and 8184 //8504
  double lki = 0.8820;
  double lkd = 0;

  double volatile Rpwm;
  double volatile Lpwm;
  double volatile cur_Rrpm;
  double volatile cur_Lrpm;
  double volatile Rrpm = 80;//setpoint
  double volatile Lrpm = 80;//setpoint
  double volatile out_Rrpm;
  double volatile out_Lrpm;
  double volatile Lspeed;
  double volatile Rspeed;

  PID rightPID(&cur_Rrpm, &out_Rrpm, &Rrpm, rkp, rki, rkd, DIRECT);
  PID leftPID(&cur_Lrpm, &out_Lrpm, &Lrpm, lkp, lki, lkd, DIRECT);
  /* ---- END OF RPM ----

  /* -------------- Radius --------------------
  #define wheelRadius 5.75 // cm
  #define distanceBtwWheel 19.0 // cm
  #define PI 3.1415926535897932384626433832795
  /* -------------- END OF Radius --------------------
*/

double computePID() {
  // Serial.println(String(counterA) + ", " + String(counterB) + ", " + String(counterA - counterB));
  double kp, ki, kd, p, i, d, error, pid, integral;

  kp = 23; // trial and error 24 keagan
  ki = 0;
  kd = 0;

  error = counterA - counterB;
  integral += error;

  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - counterA);
  pid = p + i + d;

  prevTick = counterA;

  return pid;
}

void setup() {
  Serial.begin(115200);

  /* ---- motor encoder settings ---- */
  md = new DualVNH5019MotorShield();
  md->init();

  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  // pinMode(encoderpin2a, INPUT);
  // pinMode(encoderpin2b, INPUT);
  attachPCINT(digitalPinToPCINT(outputA), outputAInc, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB), outputBInc, CHANGE);
  // enableInterrupt(encoderpin1a, encoder1change, RISING);
  // enableInterrupt(encoderpin2a, encoder2change, RISING);

  /* ---- PID settings ----
    rightPID.SetMode(AUTOMATIC);
    rightPID.SetOutputLimits(PWM_MIN, PWM_MAX);
    leftPID.SetMode(AUTOMATIC);
    leftPID.SetOutputLimits(PWM_MIN, PWM_MAX);
  */

}

void outputAInc(void) {
  counterA++;
  counterC++;
}

void outputBInc(void) {
  counterB++;
}

void moveForward(int numOfGrid) {
  int forwardClimb = 300;
  int testClimb = 150;
  int initialSpeed = 0;
  int pid;
  counterA = 0;
  counterB = 0;
  counterC = 0;
  int target_tick = 560; //Trial and Error

  if (numOfGrid > 1) {
    target_tick  = 592;
  }

  while (counterC < target_tick * numOfGrid) {
    pid = computePID();

    if (counterC < 200) {
      md->setSpeeds(counterC + 150 - pid, counterC + 150 + pid); //270
    } else if (counterC > target_tick - 200) {
      md->setSpeeds(counterC - 150 - pid, counterC - 150 + pid); //270
    } else {
      md->setSpeeds(forwardClimb - pid, forwardClimb + pid); //270
    }
  }
  counterC = 0;
  md->setBrakes(400, 400);

  /* while(initialSpeed < forwardClimb) {
          while (counterC < target_tick * numOfGrid) {
           initialSpeed = initialSpeed + 50;
             if((counterB%30) == 0){
               counterB++;
             }
             pid = computePID();
             md->setSpeeds(initialSpeed-pid,initialSpeed+pid);

         }
         break;
    }

    counterC = 0;
    md->setBrakes(350,370);
  */
}

// Legacy Forward
/*
  void forward(int distance) {
  resetEncoder();
  double wheelcircumference, distance_traveled2, distance_traveled1, distanceT;
  wheelcircumference = 2.0 * PI * (wheelRadius / 2);
  do
  {
    rightPID.Compute();
    leftPID.Compute();
    Rspeed = ((2.769 * out_Rrpm) + 5.064);
    Lspeed = ((2.821 * out_Lrpm) + 12.684);
    md.setSpeeds(Rspeed, Lspeed);
    cur_Rrpm = getRPM1();
    cur_Lrpm = getRPM2();
    distance_traveled1 = ( abs(ticksmoved1 / 526.25) * wheelcircumference); //distance travel by Right wheel
    distance_traveled2 = ( abs(ticksmoved2 / 526.25) * wheelcircumference); //distance travel by Left wheel
    distanceT = (distance_traveled1 + distance_traveled2) / 2 * distanceErrorRate; // Total distance
  }  while (distanceT < (distance));
  md.setBrakes(400, 400);
  }
*/

/*
  String data;
  char instruction;
  int charposition = 5;
*/

// Legacy Fastest Path
/*
  void fastestPath() {
  // send to android feedback -- done
  while (1) {
    if (Serial.available() > 0) {
      data = Serial.readString();
      do {
        instruction = data.charAt(charposition);
        Serial.println(instruction);
        if (instruction == 'F')
        {
          forward(10);
          Serial.println("Back");
          charposition++;
        }
        else if (instruction == 'L') {
          delay(100);
          turnLeft(90);
          Serial.println("Back");
          charposition++;
          delay(100);
        }
        else if (instruction == 'R') {
          delay(100);
          turnRight(90);
          Serial.println("Back");
          charposition++;
          delay(100);
        }
      } while (instruction != '\'');
      break;
    }
  }
  }
*/

// Legacy Backward
/*
  void backward(int distance) {
  resetEncoder();
  double wheelcircumference, distance_traveled2, distance_traveled1, distanceT;
  wheelcircumference = 2.0 * PI * (wheelRadius / 2);
  do
  {
    rightPID.Compute();
    leftPID.Compute();
    Rspeed = ((2.769 * out_Rrpm) + 5.064);
    Lspeed = ((2.821 * out_Lrpm) + 12.684);
    md.setSpeeds(-Rspeed, -Lspeed);
    cur_Rrpm = getRPM1();
    cur_Lrpm = getRPM2();
    distance_traveled1 = ( abs(ticksmoved1 / 526.25) * wheelcircumference);
    distance_traveled2 = ( abs(ticksmoved2 / 526.25) * wheelcircumference);
    distanceT = (distance_traveled1 + distance_traveled2) / 2 * distanceErrorRate;
  }  while (distanceT < (distance));
  md.setBrakes(400, 400);
  }
*/

void turnLeft(void) {
  int forwardClimb = 300; //270 keagan
  counterA = 0;
  counterB = 0;
  counterC = 0;
  int target_tick = 757;

  while (counterC < target_tick) {
    int pid = computePID();

    if (counterC > target_tick - 200) {
      md->setSpeeds(-counterC - 200 + pid, counterC - 200 + pid); //270
    } else {
      md->setSpeeds(-forwardClimb + pid, forwardClimb + pid );
    }
  }

  md->setBrakes(400 , 400);
  counterC = 0;
}

// Legacy Left
/*
  void turnLeft(double degree) {
  resetEncoder();
  float circumference, distanceToTurn, wheelC, cmPerTick, tickToTurn , newtick;
  // https://www.robotshop.com/community/forum/t/trouble-getting-redbot-to-turn-using-encoders/12411/2);
  double divider = 360.00 / degree;
  circumference = 2.0 * PI * (distanceBtwWheel / 2);
  distanceToTurn = circumference / divider;
  cmPerTick = (2.0 * PI * (wheelRadius / 2)) / 562.25;

  tickToTurn = (distanceToTurn / cmPerTick);
  newtick = tickToTurn / turnErrorRate;
  do
  {
    rightPID.Compute();
    leftPID.Compute();
    Rspeed = ((2.769 * out_Rrpm) + 5.064);
    Lspeed = ((2.821 * out_Lrpm) + 12.684);
    md.setSpeeds(Rspeed, -Lspeed);
    cur_Rrpm = getRPM1();
    cur_Lrpm = getRPM2();

  }
  while (abs(ticksmoved2) < newtick || abs(ticksmoved1) < newtick);
  md.setBrakes(400, 400);
  }
*/

void turnRight(void) {
  int forwardClimb = 300; //250 keagan
  counterA = 0;
  counterB = 0;
  counterC = 0;
  int target_tick = 763; //800 keagan

  while (counterC < target_tick) {
    int pid = computePID();

    if (counterC > target_tick - 200) {
      md->setSpeeds(counterC - 200 - pid, -counterC - 200 - pid); //270
    } else {
      md->setSpeeds(forwardClimb - pid, -forwardClimb - pid);
    }
  }

  md->setBrakes(400 , 400);
  counterC = 0;
}

// Legacy Right
/*
  void turnRight(double degree) {
   resetEncoder();
   float circumference, distanceToTurn, wheelC, cmPerTick, tickToTurn , newtick;
   // https://www.robotshop.com/community/forum/t/trouble-getting-redbot-to-turn-using-encoders/12411/2);
   double divider = 360.00 / degree;
   circumference = 2.0 * PI * (distanceBtwWheel / 2);
   distanceToTurn = circumference / divider;
   cmPerTick = (2.0 * PI * (wheelRadius / 2)) / 562.25;

   tickToTurn = (distanceToTurn / cmPerTick); // basically how many ticks to rotate to perfom rotation
   newtick = tickToTurn / turnErrorRate;
   do
   {
     rightPID.Compute();
     leftPID.Compute();
     Rspeed = ((2.769 * out_Rrpm) + 5.064);
     Lspeed = ((2.821 * out_Lrpm) + 12.684);
     md.setSpeeds(-Rspeed, Lspeed);
     cur_Rrpm = getRPM1();
     cur_Lrpm = getRPM2();

   }
   while (abs(ticksmoved2) < newtick || abs(ticksmoved1) < newtick);
   md.setBrakes(400, 400);
  }
*/

float getReading(int IRpin) {
  float valueArray[31];
  int i = 0;

  while (i < 31) {
    valueArray[i] = analogRead(IRpin);
    i++;
  }

  qsort(valueArray, 31, sizeof(valueArray[0]), sort_desc);
  return valueArray[31 / 2];
}

static int sort_desc(const void *cmp1, const void *cmp2) {
  float a = *((float *)cmp1);
  float b = *((float *)cmp2);

  return a > b ? -1 : (a < b ? 1 : 0);
}

// Legacy Sensor Sampling
/*
  float getMedianDistance(int IRpin, int model) {
  RunningMedian samples = RunningMedian(30);             //take 20 samples of sensor reading
  for (int i = 0; i < 30; i ++)
    samples.add(readSensor(IRpin, model));      //samples call readSensor() to read in sensor value
  float median = samples.getMedian();
  return median;
  }
*/

float processSense(int IRpin, int model) {
  float tmp = getReading(IRpin);
  float value, adjValue;

  // Front Right
  if (IRpin == 14) {
    value = (5666.9 / tmp) - 4.4813;
    return value;
  }

  // Front Centre
  if (IRpin == 15) {
    value = (5630.4 / tmp) - 3.9969;
    return value;
  }

  // Front Left
  if (IRpin == 16) {
    value = (5284.7 / tmp) - 4.095;
    return value;
  }

  // Left Back
  if (IRpin == 17) {
    value = (5139.1 / tmp) - 2.1157;
    adjValue = (0.9449 * value) + 0.4124;
    return adjValue;
  }

  // Left front
  if (IRpin == 18) {
    value = (5074.8 / tmp) - 1.96989;
    adjValue = (0.8925 * value) + 0.9597;
    return adjValue;
  }

  if (model == modelR) {
    value = (12101 / tmp) - 2.8711;
    return value;
  }

}

// Legacy readSensor
/*
  float readSensor(void) {
   float distance, correctdistance;
   float sensorValue = analogRead(IRpin);
   if (model == 1080 ) {
     if (IRpin == 14) { //a0 14 or front right
       distance = 5666.9 / sensorValue - 4.4813; // confirmed values
       //correctdistance = 0.9536 * distance + 0.4124;
       return distance;
     }
     if (IRpin == 15) { //a1 15 or front middle
       distance = 5630.4 / sensorValue - 3.9969; // confirmed values
       //correctdistance = 0.9536 * distance + 0.4124;
       return distance;
     }
     if (IRpin == 16) { //a2 16 or front left
       distance = 5248.7 / sensorValue - 4.095; //confirmed values
       correctdistance = 0.9536 * distance + 0.4124;
       return correctdistance;
     }
     if (IRpin == 17) { //a3 17 or RIGHT
       distance = 5139.1 / sensorValue - 2.1157; //confirmed values
       correctdistance = 0.9449 * distance + 1.1547;
       return correctdistance;
     }
     if (IRpin == 18) { //a4 18 or Left
       distance = 5074.8 / sensorValue - 1.96989; //confirmed values
       correctdistance = 0.8925 * distance + 0.9597;
       return correctdistance;
     }
   }
   if (model == 20150) {//for 20-150cm sensor
     distance =  12101 / sensorValue - 2.8711;
     //correctdistance = 0.9697 * distance + 1.0893;
     return distance;
   }
  }
*/

void getSensorData(void) {
  float frontR = processSense(irFR, modelS);
  float frontM = processSense(irFM, modelS);
  float frontL = processSense(irFL, modelS);
  float leftBack = processSense(irLB, modelS);
  float left = processSense(irLM, modelS);
  float longrange = processSense(irLong, modelR);

  // format: frontL_frontM_frontR_left_LeftBack_longrange;
  Serial.print("P;SDATA;");
  Serial.print(frontL);
  Serial.print("_");
  Serial.print(frontM);
  Serial.print("_");
  Serial.print(frontR);
  Serial.print("_");
  Serial.print(left);
  Serial.print("_");
  Serial.print(leftBack);
  Serial.print("_");
  Serial.println(longrange);
}

// Legacy Sensor Data
/*
  String getSensorData() {
  String frontR = String(getMedianDistance(irFR, 1080));
  String frontM = String(getMedianDistance(irFM, 1080));
  String frontL = String(getMedianDistance(irFL, 1080));
  String right = String(getMedianDistance(irRM, 1080));
  String left = String(getMedianDistance(irLM, 1080));
  String longrange = String(getMedianDistance(irRLong, 20150));

  //return frontL+"_"+frontM+"_"+frontR+"_"+left+"_"+righ+"_"+longrange;
  return "P;SDATA;" + frontL + "_" + frontM + "_" + frontR + "_" + left + "_" + right + "_" + longrange;
  }
*/

// TODO: implement calibrate
void loop() {
  // readAndRun();
  testRun();
  delay(120);
  exit(0);
}

void readAndRun() {
  boolean sendData = true;
  boolean endPhase = false;
  while (1) {
    if (Serial.available() > 0) {
      data = Serial.readString();
      instruction = data.charAt(7);

      switch (instruction) {
        case 'F':
          // move forward
          moveForward(1);
          delay(120);
          break;
        case 'B':
          // move backwards
          turnLeft();
          delay(120);
          turnLeft();
          delay(120);
          moveForward(1);
          delay(120);
          break;
        case 'L':
          // turn left
          turnLeft();
          delay(120);
          break;
        case 'R':
          // turn right
          turnRight();
          delay(120);
          break;
        case 'S':
          // calibrate
          calibrate(true);
          calibrateMother = 0;
          delay(120);
          break;
        case 'E':
          // end step-by-step exec
          endPhase = true;
          break;
        default:
          sendData = false;
          Serial.println("Error");
      }

      if (sendData) {
        getSensorData();
      }

      if (endPhase) {
        break;
      }
    }
  }
  delay(10);
  // fastestPath();
}

void testRun() {
   boolean sendData = true;
  boolean endPhase = false;
  while (1) {
  data = Serial.readString();
      instruction = data.charAt(7);

      switch (instruction) {
        case 'F':
          // move forward
          moveForward(1);
          delay(120);
          break;
        case 'B':
          // move backwards
          turnLeft();
          delay(120);
          turnLeft();
          delay(120);
          moveForward(1);
          delay(120);
          break;
        case 'L':
          // turn left
          turnLeft();
          delay(120);
          break;
        case 'R':
          // turn right
          turnRight();
          delay(120);
          break;
        case 'S':
          // calibrate
          calibrate(true);
          calibrateMother = 0;
          delay(120);
          break;
        case 'E':
          // end step-by-step exec
          endPhase = true;
          break;
        default:
          sendData = false;
          Serial.println("Error");
      }
       
      getSensorData();

      if (endPhase) {
        break;
      }
    }
  delay(120);
}

// Legacy Exec
/*
  void readAndRun() {
  while (1) {
    if (Serial.available() > 0) {
      data = Serial.readString();
      instruction = data.charAt(7);
      if (instruction == 'F')
      {
        forward(10);
        Serial.println("P;Done");
      }
      else if (instruction == 'B') {
        backward(10);
        Serial.println("P;Done");
      }
      else if (instruction == 'L') {
        turnLeft(90);
        Serial.println("P;Done");
      }
      else if (instruction == 'R') {
        turnRight(90);
        Serial.println("P;Done");
      }
      else if (instruction == 'C') {
        Serial.println(getSensorData());
      }

      //RETURN RECIEPT WITH EXTRA 'E', PADDING FOR ALGO
      //NEW STUFF HERE
      else if (instruction == 'S') {
        alignFront();
        Serial.println("P;Done");
      }
      //NEW STUFF HERE

      else if (instruction == 'E') {
        break;
      }
    }
  }
  delay(10);
  fastestPath();
  }
*/

// Calibration Functions
void tiltLeft(void) {
  // Serial.print("left tilt");
  float tiltLeftAngle = 1;
  counterA = 0;
  counterB = 0;
  counterC = 0;
  int forwardClimb = 200; //100 keagan
  int target_tick = tiltLeftAngle * 4.5; //4.5 keagan

  while (counterC < target_tick) {
    int pid = computePID();
    md->setSpeeds(-forwardClimb + pid, forwardClimb + pid);
  }

  md->setBrakes(400, 400);
  counterC = 0;
}

void tiltRight(void) {
  // Serial.print("right tilt");
  float tiltRightAngle = 1;
  counterA = 0;
  counterB = 0;
  counterC = 0;
  int forwardClimb = 200; //100 keagan
  int target_tick = tiltRightAngle * 4.5; //keagan

  while (counterC < target_tick) {
    int pid = computePID();
    md->setSpeeds(forwardClimb - pid, -forwardClimb - pid);
  }

  md->setBrakes(400, 400);
  counterC = 0;
}

void calibrate(boolean pos) {
  int calibrationCounter = 0;
  int calibrate1, calibrate3;
  double offset = 0.7;

  while (true) {
    calibrate1 = sLB.distance(); //getSensorReading1();
    calibrate3 = sLM.distance(); //getSensorReading3();
    // Serial.println(calibrate1);
    // Serial.println(calibrate3);
    // return;
    if (abs((calibrate3 - offset) - calibrate1) <= 0.4) { // keagan
      // Serial.println("End Condition Met");
      if (pos) {
        adjustPosition();
      }
      delay(50);
      break;
    }
    // if(calibrateMother >= 30){//keagan
    // break;
    // }
    // if(calibrationCounter >= 10){ //keagan
    // delay(500);
    // adjustPosition();
    // break;
    // }
    if (calibrate3 < (calibrate1 + offset)) {
      // Serial.println("Tilting right");
      tiltRight();
      delay(50);
      calibrateMother++; //keagan
      calibrationCounter++;
    } else if ((calibrate1 + offset) < calibrate3) {
      // Serial.println("Tilting left");
      tiltLeft();
      delay(50);
      calibrateMother++; //keagan
      calibrationCounter++;
    }
  }
}

void adjustPosition(void) {
  counterA = 0;
  counterB = 0;
  counterC = 0;
  double averageReading, offset;
  int adjustCounter = 0;
  int tempAdjust1;
  int tempAdjust2;
  int forwardClimb = 150; //100 keagan

  while (true) {
    tempAdjust1 = sLM.distance();
    // Serial.println(tempAdjust3);
    // return;
    if (abs(tempAdjust1 - 10)  <= 0 ) {
      md->setBrakes(400, 400);
      break;
    }
    // if (calibrateMother > 40 ) {
    // md->setBrakes(400,400);//keagan
    // break;
    // }
    if (tempAdjust1 < 10 ) {
      int target_tick = 10; //Trail and Error 20 keagan

      while (counterC < target_tick) {
        int pid = computePID();
        md->setSpeeds((-forwardClimb) + pid, (-forwardClimb) - pid); //270
      }

      md->setBrakes(400, 400);
      counterC = 0;
      // calibrateMother++;//keagan
      adjustCounter++;
      calibrate(false); //keagan
    } else if (tempAdjust1 > 10) {
      int target_tick = 10; //Trail and Error 20 keagan

      while (counterC < target_tick) {
        int pid = computePID();
        md->setSpeeds(forwardClimb - pid, forwardClimb + pid); //270
      }

      md->setBrakes(400, 400);
      counterC = 0;
      // calibrateMother++; //keagan
      adjustCounter++;
    }
  }
}

// Legacy Calibration
/*
  void CornerCalibrate() {
  alignFront();
  delay(100);
  turnLeft(90);
  delay(100);
  alignFront();
  delay(100);
  turnRight(90);
  delay(100);
  alignFront();
  delay(100);
  turnRight(90);
  delay(100);
  }

  void RightCalibrate() {
  delay(50);
  turnRight(90);
  delay(50);
  alignFront();
  delay(50);
  turnLeft(90);
  delay(50);

  }

  void LeftCalibrate() {
  delay(50);
  turnLeft(90);
  delay(50);
  alignFront();
  delay(50);
  turnRight(90);
  delay(50);

  }

  void alignFront() {
  int mdspeed = 80;
  int sensorError;
  int sensorErrorAllowance = 0;
  resetEncoder();



  while (sensorError = (int)getMedianDistance(irFL, 1080) - (int)getMedianDistance(irFR, 1080) > sensorErrorAllowance)
  {
    // tilted left , right wheel backwards , left wheel front --- turn right
    md.setSpeeds(-mdspeed, mdspeed);

  }

  md.setBrakes(200, 200);
  delay(30);
  resetEncoder();

  while ( sensorError = (int)getMedianDistance(irFL, 1080) - (int)getMedianDistance(irFR, 1080 ) < sensorErrorAllowance ) { //robot tilted right, turn left until acceptable error angle

    //tilted right ,left wheel backwards , right wheel front --- turn left
    md.setSpeeds(mdspeed, -mdspeed);
  }

  md.setBrakes(400, 400);
  delay(30);
  resetEncoder();

  while (getMedianDistance(irFM, 1080) > 6.9)
  {
    forward(2);
  }

  md.setBrakes(400, 400);
  delay(30);
  resetEncoder();

  //    while (getMedianDistance(irFM, 1080) >= 6.95 && getMedianDistance(irFM, 1080) < 12)
  //    {
  //      backward(2);
  //    }
  //    md.setBrakes(400, 400);

  }
*/

// Legacy Encoder
/*
  void encoder1change() {
  if (digitalRead(encoderpin1b) == LOW)
  {
    ticksmoved1++;
  }
  else
  {
    ticksmoved1--;
  }
  currentpulsetime1 = micros() - previoustime1;
  previoustime1 = micros();
  }

  void encoder2change() {
  if (digitalRead(encoderpin2b) == LOW)
  {
    ticksmoved2++;
  }
  else
  {
    ticksmoved2--;
  }
  currentpulsetime2 = micros() - previoustime2;
  previoustime2 = micros();
  }

  double volatile getRPM1() {
  if (currentpulsetime1 == 0)
  {
    return 0;
  }
  return 60000 / (((currentpulsetime1) / 1000.0) * 562.25);
  }

  double volatile getRPM2() {
  if (currentpulsetime2 == 0)
  {
    return 0;
  }
  return 60000 / (((currentpulsetime2) / 1000.0) * 562.25);
  }

  void resetEncoder() {
  ticksmoved1 = 0;
  ticksmoved2 = 0;
  }
*/
