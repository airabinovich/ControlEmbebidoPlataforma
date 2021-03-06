// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <PID_v1.h>
#include <Servo.h>
#include <math.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#define LED_PIN 13 // (Arduino is 13)
#define SAMPLE_TIME 50
#define MAX_YPR_OUTPUT 180
#define MIN_YPR_OUTPUT -180
#define N_SERVOS 6
#define DATA_RATE_DIV 5
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



//Define Variables we'll be connecting to
double yawSP, yawIn, yawOut,pitchSP,pitchIn,pitchOut,rollSP,rollIn,rollOut;

//Specify the links and initial tuning parameters
double yawKp=0.1, yawKi=0, yawKd=0;
double pitchKp=0.1, pitchKi=0, pitchKd=0;
double rollKp=0.1, rollKi=0, rollKd=0;
PID yawPID(&yawIn, &yawOut, &yawSP, yawKp, yawKi, yawKd, DIRECT);
PID pitchPID(&pitchIn, &pitchOut, &pitchSP, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollIn, &rollOut, &rollSP, rollKp, rollKi, rollKd, DIRECT);

float yawActual,pitchActual,rollActual, yawActRad, pitchActRad, rollActRad, yawCalibracion,yawMedido;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
boolean mpuCalibrated=false;
float yawAnterior=0;
enum quadrant{FIRST,SECOND};
struct platformServos{
  Servo servo;
  quadrant quad;
};

//Servo servos[6];
platformServos servos [6];

int pidOn=0;
String message;
char messageData[12][10];  
int loopCounter= 0;
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
const float MMTOCMCONV = 1;
float A1x=8.66/MMTOCMCONV     ,   A1y=68.21/MMTOCMCONV   ,
      A2x=45.16/MMTOCMCONV    ,   A2y=5/MMTOCMCONV       ,
      A3x=230.5/MMTOCMCONV    ,   A3y=5/MMTOCMCONV       ,
      A4x=267/MMTOCMCONV      ,   A4y=68.21/MMTOCMCONV   ,
      A5x=174.33/MMTOCMCONV   ,   A5y=233.71/MMTOCMCONV  ,
      A6x=101.33/MMTOCMCONV   ,   A6y=233.71/MMTOCMCONV  ;
      
float B1x=-81.5/MMTOCMCONV    ,   B1y=-38.4/MMTOCMCONV   ,
      B2x=-74/MMTOCMCONV      ,   B2y=-51.3/MMTOCMCONV   ,
      B3x=74/MMTOCMCONV       ,   B3y=-51.3/MMTOCMCONV   ,
      B4x=81.5/MMTOCMCONV     ,   B4y=-38.4/MMTOCMCONV   ,
      B5x=7.5/MMTOCMCONV      ,   B5y=89.8/MMTOCMCONV    ,
      B6x=-7.5/MMTOCMCONV     ,   B6y=89.8/MMTOCMCONV    ;
     
//float B=22 / MMTOCMCONV; //longitud corta de las patas
//float C=234 / MMTOCMCONV; // longitud larga
float B=54 / MMTOCMCONV; //longitud corta de las patas
float C=246 / MMTOCMCONV;//long larga

float x=101 / MMTOCMCONV,
      y=113 / MMTOCMCONV,
      z=230 / MMTOCMCONV;
      
float terminos[N_SERVOS][3];
float terminoA,terminoB,terminoC,terminoD,terminoE,terminoF;

float motorAng[N_SERVOS];

int minAngPrimerCuadrante=10;
int maxAngPrimerCuadrante=90;
int minAngSegundoCuadrante=90;
int maxAngSegundoCuadrante=170;
int contFIR;
float yawFIR,pitchFIR,rollFIR;
boolean debugFlag=false;
void setup() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
        
    // initialize device
    Serial.println(F("/Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("/Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
   // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setRate(12); // frecuencia 40Hz en teoría, 20Hz en la practica (no sabemos porque es la mitad de frecuencia todavia)
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
    //    Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //    Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("/DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        // configure LED for output
        pinMode(LED_PIN, OUTPUT);
        Serial.println(F("/Enabling interrupt detection (Arduino external interrupt 0)..."));
        //noInterrupts(5);
    }    
        attachInterrupt(5, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();    

        pidSetupConfigurations();
        servosSetupConfigurations();

}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if(Serial.available()){ 
             receiveData();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("/FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // get Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            yawMedido=ypr[0] * 180/M_PI;
            yawIn=yawMedido-yawCalibracion;
            pitchIn=-ypr[1] * 180/M_PI;
            rollIn=ypr[2] * 180/M_PI;
           if(debugFlag || (!mpuCalibrated && (yawMedido-yawAnterior)==0)){
             mpuCalibrated=true;
             debugFlag=false;
             yawCalibracion=yawMedido;
             yawIn=yawMedido-yawCalibracion;
             yawActual=yawIn;
             rollActual=rollIn;
             pitchActual=pitchIn;
                for(int j=0;j<6;j++){
                  pinMode(j+2, OUTPUT); 
                  servos[j].servo.attach(j+2);
                }
                yprToServoAng(yawSP,pitchSP,rollSP,motorAng);                    
                writeToServos();
            Serial.println("/Calibrado");
           }else if(mpuCalibrated){
                   if(pidOn==1){
                        yawPID.Compute();
                        pitchPID.Compute();
                        rollPID.Compute();
                        //yawActual+=yawOut;
                        pitchActual+=pitchOut;
                        rollActual+=rollOut;
                        yprToServoAng(-yawActual,-pitchActual,rollActual,motorAng);                    
                        writeToServos();
                   }else{
                      yawActual=yawIn;
                      rollActual=rollIn;
                      pitchActual=pitchIn;
                      yprToServoAng(-yawSP,-pitchSP,rollSP,motorAng);  
                      writeToServos();
                   }
                   if(loopCounter%DATA_RATE_DIV==0){
                      sendData();
                   }
                   loopCounter++;
           }else{
             Serial.print("/Calibrando: ");
             Serial.println(yawMedido);
              yawAnterior=yawMedido;
           }

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void pidSetupConfigurations(){
        yawPID.SetMode(AUTOMATIC);
        pitchPID.SetMode(AUTOMATIC);
        rollPID.SetMode(AUTOMATIC);
        yawPID.SetSampleTime(SAMPLE_TIME);
        pitchPID.SetSampleTime(SAMPLE_TIME);
        rollPID.SetSampleTime(SAMPLE_TIME);
        yawPID.SetOutputLimits(MIN_YPR_OUTPUT, MAX_YPR_OUTPUT);
        pitchPID.SetOutputLimits(MIN_YPR_OUTPUT, MAX_YPR_OUTPUT);
        rollPID.SetOutputLimits(MIN_YPR_OUTPUT, MAX_YPR_OUTPUT); 
        yawSP=0;
        pitchSP=0;
        rollSP=0;
}

void servosSetupConfigurations(){
   servos[0].quad=SECOND;  
   servos[1].quad=FIRST;  
   servos[2].quad=SECOND;  
   servos[3].quad=FIRST;  
   servos[4].quad=SECOND;  
   servos[5].quad=SECOND;  
}


void yprToServoAng(float yaw, float pitch, float roll,float *motorAngles){
                float alpha= yaw*(PI/180),
                      beta= pitch*(PI/180),
                      gamma= roll*(PI/180);    
                terminoA= cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma);
                terminoB= cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
                terminoC= sin(alpha)*cos(beta);
                terminoD= cos(alpha)*cos(beta);
                terminoE= sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
                terminoF= sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma);
                terminos[0][0]= (B2x * terminoA + B2y * terminoB + x - A2x);
                terminos[0][1]= (B2x * terminoC + B2y * terminoD + y - A2y);
                terminos[0][2]= (B2x * terminoE + B2y * terminoF + z);
                terminos[1][0]= (B3x * terminoA + B3y * terminoB + x - A3x);
                terminos[1][1]= (B3x * terminoC + B3y * terminoD + y - A3y);
                terminos[1][2]= (B3x * terminoE + B3y * terminoF + z);
                terminos[2][0]= (B4x * terminoA + B4y * terminoB + x - A4x);
                terminos[2][1]= (B4x * terminoC + B4y * terminoD + y - A4y);
                terminos[2][2]= (B4x * terminoE + B4y * terminoF + z);
                terminos[3][0]= (B5x * terminoA + B5y * terminoB + x - A5x);
                terminos[3][1]= (B5x * terminoC + B5y * terminoD + y - A5y);
                terminos[3][2]= (B5x * terminoE + B5y * terminoF + z);
                terminos[4][0]= (B6x * terminoA + B6y * terminoB + x - A6x);
                terminos[4][1]= (B6x * terminoC + B6y * terminoD + y - A6y);
                terminos[4][2]= (B6x * terminoE + B6y * terminoF + z);
                terminos[5][0]= (B1x * terminoA + B1y * terminoB + x - A1x);
                terminos[5][1]= (B1x * terminoC + B1y * terminoD + y - A1y);
                terminos[5][2]= (B1x * terminoE + B1y * terminoF + z);
                
                float feetLengths[N_SERVOS];             
                for (int k=0;k<N_SERVOS;k++){
                  feetLengths[N_SERVOS-k-1] = sqrt ( pow(terminos[k][0],2) + pow(terminos[k][1],2) + pow(terminos[k][2],2) );           
                }
                
                for (int k=0;k<N_SERVOS;k++){
                  motorAngles[k] = 90- (acos((B*B + feetLengths[k]*feetLengths[k] - C*C) / (2*B*feetLengths[k])))* (180/PI);
                  if(isnan(motorAngles[k])){motorAngles[k]=90;}
                  if(servos[k].quad == SECOND){
                    motorAngles[k]=180-motorAngles[k];

                  }
                }
}

void  writeToServos(){
  for (int k=0;k<N_SERVOS;k++){                 
    //Serial.print("m");Serial.print(k+1);Serial.print(":\t");Serial.println(motorAng[k]);
     if(servos[k].quad == SECOND && (motorAng[k]<=minAngSegundoCuadrante)){motorAng[k]=minAngSegundoCuadrante;}
     else if(servos[k].quad == SECOND && (motorAng[k]>=maxAngSegundoCuadrante)){motorAng[k]=maxAngSegundoCuadrante;}
     else if(servos[k].quad == FIRST && (motorAng[k]<=minAngPrimerCuadrante)){motorAng[k]=minAngPrimerCuadrante;}
     else if(servos[k].quad == FIRST && (motorAng[k]>=maxAngPrimerCuadrante)){motorAng[k]=maxAngPrimerCuadrante;}
           servos[k].servo.write(motorAng[k]);
   }
}

void sendData(){  
    dtostrf(x,1,2,messageData[0]);
    dtostrf(y,1,2,messageData[1]);
    dtostrf(z,1,2,messageData[2]);
    dtostrf(yawIn,1,2,messageData[3]);
    dtostrf(pitchIn,1,2,messageData[4]);
    dtostrf(rollIn,1,2,messageData[5]);
  //  dtostrf(motorAng[0],1,2,messageData[6]);
  //  dtostrf(motorAng[1],1,2,messageData[7]);
  //  dtostrf(motorAng[2],1,2,messageData[8]);
                      dtostrf(yawOut,1,2,messageData[6]);
                      dtostrf(pitchOut,1,2,messageData[7]);
                      dtostrf(rollOut,1,2,messageData[8]);
    dtostrf(motorAng[3],1,2,messageData[9]);
    dtostrf(motorAng[4],1,2,messageData[10]);
    dtostrf(motorAng[5],1,2,messageData[11]);
    message="(";
    for (int i=0;i<12;i++){
      if(i==11){
         message=message+ (String) messageData[i]+")";
      }else{
         message=message+ (String) messageData[i]+",";
      }
    }
    Serial.println(message);
}

void receiveData(){
   if(Serial.find("(")){
                x=Serial.parseFloat();
                y=Serial.parseFloat();
                z=Serial.parseFloat();
                yawSP=Serial.parseFloat();
                pitchSP=Serial.parseFloat();
                rollSP=Serial.parseFloat();
                yawKp=Serial.parseFloat();
                yawKi=Serial.parseFloat();
                yawKd=Serial.parseFloat();
                pitchKp=Serial.parseFloat();
                pitchKi=Serial.parseFloat();
                pitchKd=Serial.parseFloat();
                rollKp=Serial.parseFloat();
                rollKi=Serial.parseFloat();
                rollKd=Serial.parseFloat();
                pidOn=Serial.parseInt();
                yawPID.SetTunings(yawKp, yawKi, yawKd);
                pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
                rollPID.SetTunings(rollKp, rollKi, rollKd);
                yawOut=pitchOut=rollOut=0;
                yawActual=yawIn;
                rollActual=rollIn;
                pitchActual=pitchIn;
                //yprToServoAng(yawSP,pitchSP,rollSP,motorAng);                    
                //writeToServos();
          }
}
