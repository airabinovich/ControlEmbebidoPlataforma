#include <PID_v1.h>
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
#include <Servo.h>
#include "stewartmath.h"
#include "servoStewart.h"
#define LED_PIN 13 
#define SAMPLE_TIME 50
#define MAX_YPR_OUTPUT 180
#define MIN_YPR_OUTPUT -180
#define NUM_OF_DATA_TO_SEND 12
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
double yawKp=0.05, yawKi=0, yawKd=0;
double pitchKp=0.05, pitchKi=0, pitchKd=0;
double rollKp=0.05, rollKi=0, rollKd=0;
PID yawPID(&yawIn, &yawOut, &yawSP, yawKp, yawKi, yawKd, DIRECT);
PID pitchPID(&pitchIn, &pitchOut, &pitchSP, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollIn, &rollOut, &rollSP, rollKp, rollKi, rollKd, DIRECT);

float yawActual,pitchActual,rollActual, yawActRad, pitchActRad, rollActRad, yawCalibracion,yawMedido;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
boolean mpuCalibrated=false;
float yawAnterior=0;


servoStewart Servos;

int pidOn=0;
String message;
char messageData[12][10];  
int loopCounter= 0;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
float x=101,
      y=101,
      z=230;
      
float motorAng[N_SERVOS];
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
        Servos.servosSetupConfigurations();

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
             Servos.attachServos(2);
             xyz_yprToServoAng(x,y,z,yawSP,pitchSP,rollSP,motorAng);                    
             Servos.writeToServos(motorAng);
             Serial.println("/Calibrado");
           }else if(mpuCalibrated){
                   if(pidOn==1){
                        yawPID.Compute();
                        pitchPID.Compute();
                        rollPID.Compute();
                        //yawActual+=yawOut;
                        pitchActual+=pitchOut;
                        rollActual+=rollOut;
                        xyz_yprToServoAng(x,y,z,yawActual,pitchActual,rollActual,motorAng);                    
                        Servos.writeToServos(motorAng);
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
    for (int i=0;i<NUM_OF_DATA_TO_SEND;i++){
      if(i==NUM_OF_DATA_TO_SEND-1){
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
                //yprToServoAng(yawSP,pitchSP,rollSP,motorAng);                    
                //writeToServos();
          }
}
