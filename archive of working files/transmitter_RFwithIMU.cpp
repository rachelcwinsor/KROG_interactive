/* NANOv3 (RF24 Transmitter)
   Rachel Winsor in AIRLab under Prof Andrea Bonarini
   Final Master Project

   Credit to "Getting Started with nRF24L01+ radios"
   by J. Coliz <maniacbug@ymail.com>
  nRF24L01 Pin-out
  GND   (1) => GND
  Vcc   (2) => 3.3V - Power and GND connected to 10uF + 1uF in series
  CE    (3) => 9
  CSN   (4) => 10
  SCK   (5) => 13
  MOSI  (6) => 11
  MISO  (7) => 12
  IRQ   (8) => --

  MPU6050 Pin-out
  Vcc   (1) => 3.3V
  GND   (2) => GND
  SCL   (3) => A5
  SCA   (4) => A4
  XDA   (5) => --
  XCL   (6) => --
  ADD   (7) => --
  INT   (8) => D2

*/
#include "I2Cdev.h"
#include "RF24.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

const int pinCE = 9; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 10; //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
RF24 radio (pinCE, pinCSN);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

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
VectorInt16 gyro;
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

struct package {
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

typedef struct package IMU_package;
IMU_package data;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#define LOOP_INTERVAL 20                   // Loop interval (IN MILLIS).
static unsigned long loop_timer = 0;       // Control the loop time of the tower RED/GREEN LED.

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(38400); // initialize serial communication
  delay(1000);
  radio.begin();
  radio.setRetries(15, 15);

  //set as Transmitter
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);

//FOLLOWING LINES REMOVED TO MAKE RF WORK WITH IMU
  //  radio.setPALevel(RF24_PA_MAX);
  //  radio.setDataRate( RF24_250KBPS );
  //  delay(1000);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  //delay(5000);

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
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
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

  radio.startListening();
}

void loop()
{
  radio.stopListening();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt) {// && fifoCount < packetSize) {
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
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    //Accelerometer data
    data.aaWorld.x = aaWorld.x;
    data.aaWorld.y = aaWorld.y;
    data.aaWorld.z = aaWorld.z;

    //Gyroscope data
    data.gyro.x = gyro.x;
    data.gyro.y = gyro.y;
    data.gyro.z = gyro.z;;

    //Quaternion data
    data.q.w = q.w;
    data.q.x = q.x;
    data.q.y = q.y;
    data.q.z = q.z;

    bool result = radio.write(&data, sizeof(data));
    Serial.println(result);
#endif
  }
}
