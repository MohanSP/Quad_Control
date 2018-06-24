#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2 
#define LED_PIN 13 
bool blinkState = false;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
float ypri[3];          // [yaw, pitch, roll]   initial yaw/pitch/roll container

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
 {
  mpuInterrupt = true;
 }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
 {
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
 
  Serial.begin(115200);
  while (!Serial);

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // empty buffer
  while (Serial.available() && Serial.read()); 

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
   {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
   } 
   
  else
   {
    Serial.print(F("DMP Initialization failed"));
   }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
 }

float t,t1;
//Initialing PID Gains
    float kp[3]={0.7,0.7,1};
    float ki[3]={0,0,0};
    float kd[3]={0.1,0.1,0.12};
    float u_roll,u_pitch,u_yaw;
    float ud_roll,ud_pitch,ud_yaw;
    float ui_roll,ui_pitch,ui_yaw;   
    float e_roll,e_pitch,e_yaw; 
    float ui_roll_prev,ui_pitch_prev,ui_yaw_prev;   
    float e_roll_prev,e_pitch_prev,e_yaw_prev;     
    int m1,m2,m3,m4;
//Variables used in PID controller
    float roll_meas=0,roll_req=0,offset_r=0;
    float pitch_meas=0,pitch_req=0,offset_p=0;
    float yaw_meas=0,yaw_req=0,offset_y=0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
    
void loop()
 { 
  t=millis();
  
  while(1)
   {
    while(millis()-t<15000)
     {
      IMU();
     }  
  
    while(millis()-t>15000 && millis()-t<15100)
     {
      IMU();

      //Converting radians to degrees
      ypri[0]=ypr[0] * 180/M_PI;
      ypri[1]=ypr[1] * 180/M_PI;
      ypri[2]=ypr[2] * 180/M_PI;             
     }
  
    while(millis()-t>15200)
     {
      IMU();  
      ypr[0]=ypr[0] * 180/M_PI;
      ypr[1]=ypr[1] * 180/M_PI;
      ypr[2]=ypr[2] * 180/M_PI;

      //Simple check to make sure the system does not go uncontrollable
      if((abs(ypri[1]-ypr[1])<30) && (abs(ypri[2]-ypr[2])<30))
       {
        PID();
        delay(3);
       }
    
      else
       {
   
       }
     }
   }
 }

// ================================================================
// ===                      GETTING IMU DATA                    ===
// ================================================================

void IMU()
 {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize)
   {

   }
   
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
   {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    delay(7);
   }
   
  else if (mpuIntStatus & 0x02)
   {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
   }
 }

// ================================================================
// ===                         PID CONNTROL                     ===
// ================================================================

void PID()
 {
  //Get errors
  e_roll=(ypri[2]-ypr[2]);
  e_pitch=(ypri[1]-ypr[1]);
  e_yaw=(ypri[0]-ypr[0]);
    
  t1=(millis()-t)/1000;

  // Calculating I and D correction terms
  ui_roll = ui_roll_prev + (ki[0] * t1*e_roll);
  ud_roll = kd[0] * (e_roll - e_roll_prev)/t1;
  ui_pitch = ui_pitch_prev + (ki[1] * (t1/10)*e_pitch);  
  ud_pitch = kd[1] * (e_pitch - e_pitch_prev)/t1;
  ui_yaw = ui_yaw_prev + (ki[2] * (t1/10)*e_yaw); 
  ud_yaw = kd[2] * (e_yaw - e_yaw_prev)/t1;
 
  //Retaining previous values for Integrator  
  e_roll_prev = e_roll;
  ui_roll_prev = ui_roll;
  e_pitch_prev = e_pitch;
  ui_pitch_prev = ui_pitch;
  e_yaw_prev = e_yaw;
  ui_yaw_prev = ui_yaw;

  //Evaluating PID correction term
  u_roll = ((kp[0] * e_roll) + ui_roll + ud_roll);
  u_pitch = ((kp[1] * e_pitch) + ui_pitch + ud_pitch);
  u_yaw = ((kp[2] * e_yaw) + ui_yaw + ud_yaw);

  //Control mixing for 'X' configuration
  m1=150+u_roll+u_pitch-u_yaw;
  m2=150+u_roll-u_pitch+u_yaw;
  m3=150-u_roll-u_pitch-u_yaw;
  m4=150-u_roll+u_pitch+u_yaw;

  //m1
  analogWrite(6,m1);
  //m2
  analogWrite(5,m2);
  //m3
  analogWrite(11,m3);
  //m4
  analogWrite(10,m4); 
 }
