/* MEGA (RF24 Receiver & Audio-Input Receiver)
   Rachel Winsor in AIRLab under Prof Andrea Bonarini
   Final Master Project


   Credit to "Getting Started with nRF24L01+ radios"
   by J. Coliz <maniacbug@ymail.com>
  nRF24L01 Pin-out
  GND   (1) => GND
  Vcc   (2) => 3.3V - Power and GND connected to 10uF + 1uF in series
  CE    (3) => 40
  CSN   (4) => 53
  SCK   (5) => 52
  MOSI  (6) => 51
  MISO  (7) => 50
  IRQ   (8) => --

  MAX9814 Pin-out
  GND   (1) => GND
  Vdd   (2) => 3.3V
  Gain  (3) => Vdd
  Out   (4) => A0
  AR    (5) => --

*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define VOLT_ARRAY_SIZE 20
#define GYRO_ARRAY_SIZE 5

// Hardware configuration
// Set up nRF24L01 radio on SPI bus plus pins 40 & 53
RF24 radio (40, 53);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

long testTime = 250;           // milliseconds of on-time

void startRadio();
void readRadio();
void listenMusic();

//MPU6050
typedef struct package {
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
} IMU_package;

int gIndex = 0;
VectorInt16 gyroArray[GYRO_ARRAY_SIZE];
VectorInt16 prev_gyroArray[GYRO_ARRAY_SIZE];

//MAX9814
const int sampleWindow = 500; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
unsigned long previousTime = 0;
int vIndex = 0;
double voltArray[VOLT_ARRAY_SIZE];
double prev_voltArray[VOLT_ARRAY_SIZE];

void setup(void)
{
  Serial.begin(38400);
  printf_begin();
  startRadio();

  // Dump the configuration of the RF unit for debugging
  radio.printDetails();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - previousTime >= testTime)
  {
    if (radio.available())
      readRadio();
    //Serial.println("radio");
    else
      listenMusic();
    //Serial.println("audio");
    previousTime = currentTime;
  }
}

void startRadio() {
  // Setup and configure rf radio
  radio.begin();
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);
  // Open pipes for writing and reading
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  // Start listening
  radio.startListening();
}

void readRadio() {
  // Dump the payloads until we've gotten everything
  IMU_package dataIn;
  bool done = false;
  while (!done)
  {
    // Fetch the payload, and see if this was the last one.
    done = radio.read( &dataIn, sizeof(dataIn) );

    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(20);
  }
  if (gIndex > (GYRO_ARRAY_SIZE - 1) )
  {
    for (int i = 0; i < GYRO_ARRAY_SIZE; ++i) {
      prev_gyroArray[i] = gyroArray[i];
    }
    gIndex = 0;
  }
  gyroArray[gIndex] = dataIn.gyro;
  ++gIndex;
}

void listenMusic() {
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  // collect data for 500 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(0);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  if (vIndex > (VOLT_ARRAY_SIZE - 1) )
  {
    for (int i = 0; i < VOLT_ARRAY_SIZE; ++i) {
      prev_voltArray[i] = voltArray[i];
    }
    vIndex = 0;
  }
  voltArray[vIndex] = (peakToPeak * 5.0) / 1024;  // convert to volts
  ++vIndex;
}
