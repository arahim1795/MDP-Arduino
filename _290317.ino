/*
#include "DualVNH5019MotorShield.h"
#include "PinChangeInterrupt.h"
#include "PID_v1.h"
#include "stdio.h"
#include "math.h"
#include "SharpIR.h"


//mapping digital pins on Arduino //DO NOT CHANGE UNLESS NECESSARY
#define outputA 3 //encoder output for left motor M1
#define inputA 9 // left motor speed input
#define outputB 5 //encoder output for right motor M2
#define inputB 10 //right motor speed input

#define s1 A1 //mapping analog pins on arduino //DO NOT CHANGE UNLESS NECESSARY
#define s2 A2
#define s3 A3
#define s4 A4
#define s5 A5

#define model 1080

SharpIR sr1 =  SharpIR(s1, model); //instantiation of sensors //DO NOT CHANGE
SharpIR sr2 =  SharpIR(s2, model);
SharpIR sr3 =  SharpIR(s3, model);
SharpIR sr4 =  SharpIR(s4, model);
SharpIR sr5 =  SharpIR(s5, model);
*/

const int s1r[] = {306, 203, 148}; //
const int s2r[] = {387, 228, 160}; //
const int s3r[] = {320, 214, 170}; //
const int s4r[] = {300, 180, 135};
const int s5r[] = {148, 91, 70}; //276

int numOfGrid;
int degreeOfTurn;
int forwardCount = 0;
volatile int counterA = 0;
volatile int counterB = 0;
double prevTick = counterA;
volatile double counterC = 0;
double difference;
double Setpoint, Input, Output;
//double Kp=0.5, Ki=0, Kd=0;
String commandReceived;
int calibrateMother = 0;

//PID myPID(&counterA, &Output, &counterB, Kp, Ki, Kd, DIRECT);
DualVNH5019MotorShield* md;

double computePID() {
//  Serial.println(String(counterA) + ", " + String(counterB) + ", " + String(counterA - counterB));
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
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.read();
  Serial.setTimeout(20);
  md = new DualVNH5019MotorShield();
  md->init();
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);
  attachPCINT(digitalPinToPCINT(outputA), outputAInc, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB), outputBInc, CHANGE);
  //myPID.SetOutputLimits(-50,50);
  //myPID.SetMode(AUTOMATIC);
  //turnRight();
  //delay(500);
  //turnRight();

}

void outputAInc(void){
  counterA++; //counterA = ((counterA % 33) == 0) ? counterA+2 : counterA+1;
  counterC++;
}

void outputBInc(void){
  //counterB = ((counterB % 20) == 0) ? counterB+2 : counterB+1;
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
   if(numOfGrid > 1)
      target_tick  = 592;
   while (counterC < target_tick * numOfGrid){
    pid = computePID();
    if(counterC<200)
      md->setSpeeds(counterC+150-pid,counterC+150+pid);//270
    else if(counterC >target_tick -200)
      md->setSpeeds(counterC-150-pid,counterC-150+pid);//270
    else
      md->setSpeeds(forwardClimb-pid,forwardClimb+pid);//270

  }
  counterC = 0;
  md->setBrakes(400,400);


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
  md->setBrakes(350,370); */

}

void turnLeft(void){
    int forwardClimb = 300; //270 keagan
    counterA = 0;
    counterB = 0;
    counterC = 0;
    int target_tick = 757;
    while(counterC < target_tick){
       int pid = computePID();
      if(counterC >target_tick -200)
      md->setSpeeds(-counterC-200+pid,counterC-200+pid);//270
      else
      md->setSpeeds(-forwardClimb+pid,forwardClimb+pid );
    }
    md-> setBrakes(400 ,400);
    counterC = 0;


}

void turnRight(void){
    int forwardClimb = 300;//250 keagan
    counterA = 0;
    counterB = 0;
    counterC = 0;
    int target_tick = 763;//800 keagan
    while(counterC < target_tick){
       int pid = computePID();
       if(counterC >target_tick -200)
       md->setSpeeds(counterC-200-pid,-counterC-200-pid);//270
       else
       md->setSpeeds(forwardClimb-pid,-forwardClimb-pid);
    }
    md-> setBrakes(400 ,400);
    counterC = 0;


 }

int getSensorReading1(void){
  int sensorValueArray[17];
  int i = 0;
  while (i<17)
  {
//    Serial.println(analogRead(s1));
  sensorValueArray[i]= analogRead(s1);
  i++;
  }
  qsort(sensorValueArray,17,sizeof(int),intCompareFn);
//  Serial.println(sensorValueArray[17/2]);
  return sensorValueArray[17/2];

}

int getSensorReading2(void){
  int sensorValueArray[17];
  int i = 0;
  while (i<17)
  {
  sensorValueArray[i]= analogRead(s2);
  i++;
  }
  qsort(sensorValueArray,17,sizeof(int),intCompareFn);
//  Serial.println(sensorValueArray[17/2]);

  return sensorValueArray[17/2];

}

int getSensorReading3(void){
  int sensorValueArray[17];
  int i = 0;
  while (i<17)
  {
  sensorValueArray[i]= analogRead(s3);
  i++;
  }
  qsort(sensorValueArray,17,sizeof(int),intCompareFn);
//  Serial.println(sensorValueArray[17/2]);
  return sensorValueArray[17/2];

}

int getSensorReading4(void){
  int sensorValueArray[17];
  int i = 0;
  while (i<17)
  {
  sensorValueArray[i]= analogRead(s4);
  i++;
  }
  qsort(sensorValueArray,17,sizeof(int),intCompareFn);
  return sensorValueArray[17/2];

}

int getSensorReading5(void){
  int sensorValueArray[17];
  int i = 0;
  while (i<17)
  {
  sensorValueArray[i]= analogRead(s5);
  i++;
  }
  qsort(sensorValueArray,17,sizeof(int),intCompareFn);
//    Serial.println(sensorValueArray[17/2]);

  return sensorValueArray[17/2];

}

static int intCompareFn( void * arg1, void * arg2) {
  int * a = (int *)arg1;
  int * b = (int *)arg2;

  if (*a < *b)
    return -1;

  if (*a > *b)
    return 1;

  return 0;
}

void tiltLeft(void){
//    Serial.print("left tilt");
    float tiltLeftAngle = 1;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    int forwardClimb = 200;//100 keagan
    int target_tick = tiltLeftAngle * 4.5; //4.5 keagan
    while(counterC < target_tick){
      int pid = computePID();
      md->setSpeeds(-forwardClimb+pid ,forwardClimb+pid);
    }
    md-> setBrakes(400,400);
    counterC = 0;
}

void tiltRight(void){
//    Serial.print("right tilt");
    float tiltRightAngle = 1;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    int forwardClimb = 200;//100 keagan
    int target_tick = tiltRightAngle * 4.5; //keagan
    while(counterC < target_tick){
      int pid = computePID();
      md->setSpeeds(forwardClimb-pid ,-forwardClimb-pid);
    }
    md-> setBrakes(400,400);
    counterC = 0;
}

void calibrate(boolean pos){
  int calibrationCounter = 0;
  int calibrate1, calibrate3;
  double offset = 0.7;
      while (true){
        calibrate1 = sr1.distance();//getSensorReading1();
        calibrate3 = sr3.distance();//getSensorReading3();
//        Serial.println(calibrate1);
//        Serial.println(calibrate3);
//        return;
        if ( abs((calibrate3  - offset) - calibrate1) <=0.4){ // keagan
//          Serial.println("End Condition Met");
              if(pos == true){
                  adjustPosition();
              }
              delay(50);
              break;
            }
//          if(calibrateMother >= 30){//keagan
//            break;
//          }
//         if(calibrationCounter >= 10){ //keagan
//                //delay(500);
//                adjustPosition();
//                break;
//              }
        if(calibrate3 < calibrate1 + offset){
//          Serial.println("Tilting right");
          tiltRight();
          delay(50);
          calibrateMother++; //keagan
          calibrationCounter++;
        }
        else if (calibrate1 +offset < calibrate3) {
//          Serial.println("Tilting left");
          tiltLeft();
          delay(50);
          calibrateMother++; //keagan
          calibrationCounter++;
         }

        }

  }

int sensor1(void){
  int temp1 = getSensorReading1();
//  Serial.println(temp1);
  int grid;
  if(temp1>s1r[0]){
    grid = 0;
    return grid;
  }
  else if(temp1>s1r[1]) {
    grid = 1;
    return grid;
  }
  else if(temp1>s1r[2]) {
    grid = 2;
    return grid;
  }
  else
    return 20;
}

int sensor2(void){
  int temp2 = getSensorReading2();
//  Serial.println(temp2);
  int grid;
  if(temp2>s2r[0]){
    grid = 0;
    return grid;
  }
  else if(temp2>s2r[1]) {
    grid = 1;
    return grid;
  }
  else if(temp2>s2r[2]) {
    grid = 2;
    return grid;
  }
  else
    return 20;

};

int sensor3(void){
  int temp3 = getSensorReading3();
//  Serial.println(temp3);
  int grid;
  if(temp3>s3r[0]){
    grid = 0;
    return grid;
  }
  else if(temp3>s3r[1]) {
    grid = 1;
    return grid;
  }
  else if(temp3>s3r[2]) {
    grid = 2;
    return grid;
  }
  else
    return 20;

}

int sensor4(void){
 int temp4 = getSensorReading4();
//  Serial.println(temp4);
  int grid;
  if(temp4>s4r[0]){
    grid = 0;
    return grid;
  }
  else if(temp4>s4r[1]) {
    grid = 1;
    return grid;
  }
  else if(temp4>s4r[2]) {
    grid = 2;
    return grid;
  }
  else
    return 20;
}

int sensor5(void){
 int temp5 = getSensorReading5();
// Serial.println(temp5);
  int grid;
  if(temp5>s5r[0]){
    grid = 0;
    return grid;
  }
  else if(temp5>s5r[1]) {
    grid = 1;
    return grid;
  }
  else if(temp5>s5r[2]) {
    grid = 2;
    return grid;
  }
  else
    return 20;
}

void getSensorDistance(void){
  int sensorDis1 = sensor1();
  int sensorDis2 = sensor2();
  int sensorDis3 = sensor3();
  int sensorDis4 = sensor4();
  int sensorDis5 = sensor5();

  Serial.print(sensorDis4);
  Serial.print(";");
  Serial.print(sensorDis1);
  Serial.print(";");
  Serial.print(sensorDis2);
  Serial.print(";");
  Serial.print(sensorDis3);
  Serial.print(";");
  Serial.print(sensorDis5);
  Serial.println(";");
}

void adjustPosition(void){
  counterA = 0;
  counterB = 0;
  counterC = 0;
  double averageReading, offset;
  int adjustCounter = 0;
  int tempAdjust1;
  int tempAdjust2;
  int forwardClimb = 150; //100 keagan

  while(true){
      tempAdjust1 = sr3.distance();
      //Serial.println(tempAdjust3);
      //return;
      if (abs(tempAdjust1 - 10)  <= 0 ) {
          md->setBrakes(400,400);
          break;
        }
//      if (calibrateMother > 40 ) {
//          md->setBrakes(400,400);//keagan
//          break;
//        }
      if (tempAdjust1 < 10 ) {
          int target_tick = 10; //Trail and Error 20 keagan
          while (counterC < target_tick){
            int pid = computePID();
            md->setSpeeds((-forwardClimb)+pid,(-forwardClimb)-pid);//270
          }
          md->setBrakes(400,400);
          counterC = 0;
//          calibrateMother++;//keagan
          adjustCounter++;
          calibrate(false); //keagan
          }
       else if (tempAdjust1 > 10) {
         int target_tick = 10; //Trail and Error 20 keagan
          while (counterC < target_tick){
            int pid = computePID();
            md->setSpeeds(forwardClimb-pid,forwardClimb+pid);//270
          }
          md->setBrakes(400,400);
          counterC = 0;
//          calibrateMother++; //keagan
          adjustCounter++;
          }
        }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
        commandReceived = Serial.readString();
       delay(120);
        int lengthOfString = commandReceived.length();
        char charArray[lengthOfString];
        bool flag = false;

        commandReceived.toCharArray(charArray, lengthOfString+1);

        for(int i = 0; i < lengthOfString; i+=2){
                flag = true;
                switch(charArray[i]){
                            case 'f':
                              {

                                if(charArray[i+1] == ';'){
                                    moveForward(1);
//                                    Serial.println("moveForward 1");
                                }else{
                                  String num = "";
                                  while(charArray[i+1] !=';'){
                                    num = num + charArray[i+1];
                                    i++;
                                  }

//                                    Serial.println("moveForward "+num.toInt());
                                   moveForward(num.toInt());
                               }
                               delay(120);
                               continue;
                              }

                            case 'l':
                              {
                                  turnLeft();
                                delay(120);
                                  continue;
                                }
                             case 'o':
                              {
                                  turnRight();
                                delay(120);
                                  continue;
                                }
                              case 'm':
                              {
                                  //getSensorDistance();
                                  delay(120);
                                  continue;
                                 }
                              case 'c':
                              {
                                 calibrate(true);
                                delay(120);
                                calibrateMother =0; //keagan
                                 continue;
                                }

                  }
          }
          if (flag == true){
              getSensorDistance(); //sending sensor data to algo once string of commands finish executing
            }

  }


}
