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
  Gain  (3) =>
  Out   (4) =>
  AR    (5) =>

*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

struct package {
  Quaternion q;
  VectorInt16 aaWorld;
  VectorInt16 gyro;
};

typedef struct package IMU_package;
IMU_package data;

// Hardware configuration
// Set up nRF24L01 radio on SPI bus plus pins 40 & 53
RF24 radio (40, 53);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup(void)
{
  Serial.begin(38400);
  printf_begin();

  // Setup and configure rf radio
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  // Open pipes for writing and reading
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);

  // Start listening
  radio.startListening();

  // Dump the configuration of the RF unit for debugging
  radio.printDetails();
}

void loop() {
  if (radio.available())
  {
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

    Serial.print("gyro\t");
    Serial.print(dataIn.gyro.x);
    Serial.print("\t");
    Serial.print(dataIn.gyro.y);
    Serial.print("\t");
    Serial.println(dataIn.gyro.z);

    // Now, resume listening so we catch the next packets.
    //radio.startListening();
  }
}
