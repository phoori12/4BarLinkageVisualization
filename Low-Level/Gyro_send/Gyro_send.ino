#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 15  
#define LED_PIN 2 // 

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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int c = 0;

union packed_float {
  float i;
  byte b[4];
} ax1,ay1,az1,gx1,gy1,gz1;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


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
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(500);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[2] * 180/M_PI);
//    Serial.print("\t");
    
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    float accX = ((float)aaWorld.x) / 16384.0;
    float accY = ((float)aaWorld.y) / 16384.0;
    float accZ = ((float)aaWorld.z) / 16384.0;
    
//    Serial.print("aworld\t");
//    Serial.print(accX);
//    Serial.print("\t");
//    Serial.print(accY);
//    Serial.print("\t");
//    Serial.println(accZ);
    sendCmd(accX, accY, accZ, ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
//    sendCmd(1, 2, 3, 1, 5, 6);
    
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(10);
  }
  

  

 
}


void sendCmd(float accx1, float accy2,float accz3,float gyx1,float gyy1,float gyz1) {
  ax1.i = accx1;
  ay1.i = accy2;
  az1.i = accz3;
  gx1.i = gyx1;
  gy1.i = gyy1;
  gz1.i = gyz1;

  const char buf[52] = {'#', 's', ax1.b[3], ax1.b[2], ax1.b[1], ax1.b[0],
                                  ay1.b[3], ay1.b[2], ay1.b[1], ay1.b[0],
                                  az1.b[3], az1.b[2], az1.b[1], az1.b[0],
                                  gx1.b[3], gx1.b[2], gx1.b[1], gx1.b[0],
                                  gy1.b[3], gy1.b[2], gy1.b[1], gy1.b[0],
                                  gz1.b[3], gz1.b[2], gz1.b[1], gz1.b[0],
                                  ax1.b[3], ax1.b[2], ax1.b[1], ax1.b[0],
                                  ay1.b[3], ay1.b[2], ay1.b[1], ay1.b[0],
                                  az1.b[3], az1.b[2], az1.b[1], az1.b[0],
                                  gx1.b[3], gx1.b[2], gx1.b[1], gx1.b[0],
                                  gy1.b[3], gy1.b[2], gy1.b[1], gy1.b[0],
                                  gz1.b[3], gz1.b[2], gz1.b[1], gz1.b[0],
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
 
