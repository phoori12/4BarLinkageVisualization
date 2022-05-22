#include <Arduino.h>


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN_MPU1 15  
#define INTERRUPT_PIN_MPU2 14
#define LED_PIN 2 // 

bool blinkState = false;

//////////////////////////// MPU_1 control/status vars ///////////////////////////////////////////
bool dmpReady_MPU1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_MPU1;   // holds actual interrupt status byte from MPU
uint8_t devStatus_MPU1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_MPU1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_MPU1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_MPU1[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_MPU1;           // [w, x, y, z]         quaternion container
VectorInt16 aa_MPU1;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_MPU1;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_MPU1;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_MPU1;    // [x, y, z]            gravity vector
float euler_MPU1[3];         // [psi, theta, phi]    Euler angle container
float ypr_MPU1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_USED_MPU1[3];      // [yaw, pitch, roll]   yaw/pitch/roll container for serial comms
float acc_USED_MPU1[3];      // [x, y, z]            world-frame calculated acceleration for serial comms

volatile bool mpuInterrupt_MPU1 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_MPU1() {
    mpuInterrupt_MPU1 = true;
}

//////////////////////////// MPU_2 control/status vars ///////////////////////////////////////////

bool dmpReady_MPU2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_MPU2;   // holds actual interrupt status byte from MPU
uint8_t devStatus_MPU2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_MPU2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_MPU2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_MPU2[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q_MPU2;           // [w, x, y, z]         quaternion container
VectorInt16 aa_MPU2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_MPU2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_MPU2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_MPU2;    // [x, y, z]            gravity vector
float euler_MPU2[3];         // [psi, theta, phi]    Euler angle container
float ypr_MPU2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_USED_MPU2[3];      // [yaw, pitch, roll]   yaw/pitch/roll container for serial comms
float acc_USED_MPU2[3];      // [x, y, z]            world-frame calculated acceleration for serial comms

volatile bool mpuInterrupt_MPU2 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_MPU2() {
    mpuInterrupt_MPU2 = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

int c = 0;

union packed_float {
  float i;
  byte b[4];
} ax1,ay1,az1,gy1,gp1,gr1,ax2,ay2,az2,gy2,gp2,gr2;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void sendCmd(float ypr1[], float acc1[], float ypr2[], float acc2[]);

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); 

    delay(500);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    mpu2.initialize();
    pinMode(INTERRUPT_PIN_MPU1, INPUT);
    pinMode(INTERRUPT_PIN_MPU2, INPUT);
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050_1 connection successful") : F("MPU6050_1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050_2 connection successful") : F("MPU6050_2 connection failed"));
    delay(500);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_MPU1 = mpu1.dmpInitialize();
    devStatus_MPU2 = mpu2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(220);
    mpu1.setYGyroOffset(76);
    mpu1.setZGyroOffset(-85);
    mpu1.setZAccelOffset(1788); // 1688 factory default for my test chip

    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788); // 1688 factory default for my test chip

    ///////////////////////////////////////////// Check for MPU_1 /////////////////////////////////////////////
    // make sure it worked (returns 0 if so)
    if (devStatus_MPU1 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu1.CalibrateAccel(6);
        mpu1.CalibrateGyro(6);
        mpu1.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP MPU_1..."));
        mpu1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_MPU1));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU1), dmpDataReady_MPU1, RISING);
        mpuIntStatus_MPU1 = mpu1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_MPU1 = true;

        // get expected DMP packet size for later comparison
        packetSize_MPU1 = mpu1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP MPU_1 Initialization failed (code "));
        Serial.print(devStatus_MPU1);
        Serial.println(F(")"));
    }
    ///////////////////////////////////////////// Check for MPU_2 /////////////////////////////////////////////
    if (devStatus_MPU2 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu2.CalibrateAccel(6);
        mpu2.CalibrateGyro(6);
        mpu2.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP MPU_2..."));
        mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_MPU2));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU2), dmpDataReady_MPU2, RISING);
        mpuIntStatus_MPU2 = mpu2.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_MPU2 = true;

        // get expected DMP packet size for later comparison
        packetSize_MPU2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP MPU_2 Initialization failed (code "));
        Serial.print(devStatus_MPU2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  // MPU_1 Fetch //
  if (!dmpReady_MPU1) return;

  if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer_MPU1)) {
    mpu1.dmpGetQuaternion(&q_MPU1, fifoBuffer_MPU1);
    mpu1.dmpGetAccel(&aa_MPU1, fifoBuffer_MPU1);
    mpu1.dmpGetGravity(&gravity_MPU1, &q_MPU1);
    mpu1.dmpGetYawPitchRoll(ypr_MPU1, &q_MPU1, &gravity_MPU1);
    ypr_USED_MPU1[0] = ypr_MPU1[0] * 180/M_PI;
    ypr_USED_MPU1[1] = ypr_MPU1[1] * 180/M_PI;
    ypr_USED_MPU1[2] = ypr_MPU1[2] * 180/M_PI;
//     Serial.print("ypr1\t");
//     Serial.print(ypr_USED_MPU1[0]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU1[1]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU1[2]);
//     Serial.print("\t");
    mpu1.dmpGetLinearAccel(&aaReal_MPU1, &aa_MPU1, &gravity_MPU1);
    mpu1.dmpGetLinearAccelInWorld(&aaWorld_MPU1, &aaReal_MPU1, &q_MPU1);
    acc_USED_MPU1[0] = ((float)aaWorld_MPU1.x) / 16384.0;
    acc_USED_MPU1[1] = ((float)aaWorld_MPU1.y) / 16384.0;
    acc_USED_MPU1[2] = ((float)aaWorld_MPU1.z) / 16384.0;
//     Serial.print("aworld1\t");
//     Serial.print(acc_USED_MPU1[0]);
//     Serial.print("\t");
//     Serial.print(acc_USED_MPU1[1]);
//     Serial.print("\t");
//     Serial.print(acc_USED_MPU1[2]);
  }
  // MPU_2 Fetch //
  if (!dmpReady_MPU2) return;

  if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer_MPU2)) {
    mpu2.dmpGetQuaternion(&q_MPU2, fifoBuffer_MPU2);
    mpu2.dmpGetAccel(&aa_MPU2, fifoBuffer_MPU2);
    mpu2.dmpGetGravity(&gravity_MPU2, &q_MPU2);
    mpu2.dmpGetYawPitchRoll(ypr_MPU2, &q_MPU2, &gravity_MPU2);
    ypr_USED_MPU2[0] = ypr_MPU2[0] * 180/M_PI;
    ypr_USED_MPU2[1] = ypr_MPU2[1] * 180/M_PI;
    ypr_USED_MPU2[2] = ypr_MPU2[2] * 180/M_PI;
//     Serial.print("\t|\typr2\t");
//     Serial.print(ypr_USED_MPU2[0]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU2[1]);
//     Serial.print("\t");
//     Serial.print(ypr_USED_MPU2[2]);
//     Serial.print("\t");
    
    mpu2.dmpGetLinearAccel(&aaReal_MPU2, &aa_MPU2, &gravity_MPU2);
    mpu2.dmpGetLinearAccelInWorld(&aaWorld_MPU2, &aaReal_MPU2, &q_MPU2);
    acc_USED_MPU2[0] = ((float)aaWorld_MPU2.x) / 16384.0;
    acc_USED_MPU2[1] = ((float)aaWorld_MPU2.y) / 16384.0;
    acc_USED_MPU2[2] = ((float)aaWorld_MPU2.z) / 16384.0;   
//     Serial.print("aworld2\t");
//     Serial.print(acc_USED_MPU2[0]);
//     Serial.print("\t");
//     Serial.print(acc_USED_MPU2[1]);
//     Serial.print("\t");
//     Serial.println(acc_USED_MPU2[2]);
//    sendCmd(accX, accY, accZ, ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
  }

  sendCmd(ypr_USED_MPU1, acc_USED_MPU1, ypr_USED_MPU2, acc_USED_MPU2);
  delay(10);
  
}


void sendCmd(float ypr1[], float acc1[], float ypr2[], float acc2[]) {
  ax1.i = acc1[0];
  ay1.i = acc1[1];
  az1.i = acc1[2];
  gy1.i = ypr1[0];
  gp1.i = ypr1[1];
  gr1.i = ypr1[2];
  ax2.i = acc2[0];
  ay2.i = acc2[1];
  az2.i = acc2[2];
  gy2.i = ypr2[0];
  gp2.i = ypr2[1];
  gr2.i = ypr2[2];

  const char buf[52] = {'#', 's', ax1.b[3], ax1.b[2], ax1.b[1], ax1.b[0],
                                  ay1.b[3], ay1.b[2], ay1.b[1], ay1.b[0],
                                  az1.b[3], az1.b[2], az1.b[1], az1.b[0],
                                  gy1.b[3], gy1.b[2], gy1.b[1], gy1.b[0],
                                  gp1.b[3], gp1.b[2], gp1.b[1], gp1.b[0],
                                  gr1.b[3], gr1.b[2], gr1.b[1], gr1.b[0],
                                  ax2.b[3], ax2.b[2], ax2.b[1], ax2.b[0],
                                  ay2.b[3], ay2.b[2], ay2.b[1], ay2.b[0],
                                  az2.b[3], az2.b[2], az2.b[1], az2.b[0],
                                  gy2.b[3], gy2.b[2], gy2.b[1], gy2.b[0],
                                  gp2.b[3], gp2.b[2], gp2.b[1], gp2.b[0],
                                  gr2.b[3], gr2.b[2], gr2.b[1], gr2.b[0],
                        '\r', '\n'
                       };

 for (uint8_t i = 0; i < 52; i++) {
   // Serial.write(cmd[i]);
    Serial.write(buf[i]);
  }                
}

//long time_acc;
//long prev_time_acc = 0;
//float calculateVelocity(float acc, float prev_v) {
//  // sampling freq. 10ms
//  long time_acc = millis();
//  float dv = acc * (time_acc - prev_time_acc);
//  float v = prev_v + dv;
//  prev_time_acc = time_acc;
//  return v;
//} calculate in high level
