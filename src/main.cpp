/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */

#include <Arduino.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include "SparkFunBME280.h"
#include "Wire.h"
#include <stdint.h>

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const int role_pin = 7;
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;
// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
// The role of the current running sketch
role_e role;
uint8_t counter;

//Global sensor object
BME280 mySensor;

void setup(void)
{

  // set up the role pin
  pinMode(role_pin, INPUT);
  digitalWrite(role_pin,HIGH);
  delay(20); // Just to get a solid reading on the role pin

  // read the address pin, establish our role
  if ( ! digitalRead(role_pin) )
    role = role_ping_out;
  else
    role = role_pong_back;

  Serial.begin(57600);
  printf_begin();
  printf("\n\rRF24/examples/pingpair/\n\r");
  printf("ROLE: %s\n\r",role_friendly_name[role]);

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  if ( role == role_ping_out )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }

  radio.startListening();
  radio.printDetails();

  //***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x77;

	//For SPI enable the following and dissable the I2C section
	//mySensor.settings.commInterface = SPI_MODE;
	//mySensor.settings.chipSelectPin = 10;
	//***Operation settings*****************************//
	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 0;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;

  //Calling .begin() causes the settings to be loaded
  Serial.print("Starting BME280... result of .begin(): 0x");
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	Serial.println(mySensor.begin(), HEX);
}

void loop(void)
{

  if (role == role_ping_out){

    //Each loop, take a reading.
  	//Start with temperature, as that data is needed for accurate compensation.
  	//Reading the temperature updates the compensators of the other functions
  	//in the background.

    float temperature = mySensor.readTempC();
    float pressure = mySensor.readFloatPressure();
    float humidity = mySensor.readFloatHumidity();
    Serial.print("Temperature: ");
  	Serial.print(temperature, 2);
  	Serial.println(" degrees C");
  	Serial.print("Pressure: ");
  	Serial.print(pressure, 2);
  	Serial.println(" Pa");
    /*
  	Serial.print("Altitude: ");
  	Serial.print(mySensor.readFloatAltitudeMeters(), 2);
  	Serial.println("m");
    */
  	Serial.print("%RH: ");
  	Serial.print(humidity, 2);
  	Serial.println(" %");

    radio.stopListening();

    char packet[32];
    packet[0] = 1; // id
    packet[1] = counter; // packet counter
    packet[2] = 0; // packet type
    packet[3] = 0; // reserved

    float *packetF = (float*) &packet;
    packetF[1] = temperature;
    packetF[2] = pressure;
    packetF[3] = humidity;

    printf("counter=%d, ", counter);
    bool ok = radio.write( &packet, 32 );
    counter++;
    if (!ok)
      printf("write failed, ");

    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200 )
        timeout = true;

    // Describe the results
    if ( timeout ){
      printf("response timed out");
    }else{

      // Spew it
      printf("response: ");

      // Dump the payloads until we've gotten everything
      bool done = false;
      char data[32];
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        //done = radio.read( &got_time, sizeof(unsigned long) );
        done = radio.read( &data, 32 );
        uint8_t size = radio.getPayloadSize();
        printf("size=%d, ", size);

        // Spew it
        //printf("Got payload %lu...",got_time);
        data[31]=0;
        printf("data[0]=%d", data[0]);
        //printf("data[1:end]=%s", &data[1]);
      }
    }
    Serial.println("\n=========================================");
    delay(2000);
  }


  if ( role == role_pong_back )
  {
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      char data[32];
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        //done = radio.read( &got_time, sizeof(unsigned long) );
        done = radio.read( &data, 32 );
        uint8_t size = radio.getPayloadSize();
        printf("size=%d\n", size);

        // Spew it
        //printf("Got payload %lu...",got_time);
        data[32]=0;
        printf("data: %s\n", &data);

        // Delay just a little bit to let the other unit
        // make the transition to receiver
        delay(20);

      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &data, 32 );
      //printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
  }
}

/*
if (role == role_ping_out)
{
  // First, stop listening so we can talk.
  radio.stopListening();

  // Take the time, and send it.  This will block until complete
  unsigned long time = millis();
  printf("Now sending %lu...",time);
  bool ok = radio.write( &time, sizeof(unsigned long) );

  if (ok)
    printf("ok...");
  else
    printf("failed.\n\r");

  // Now, continue listening
  radio.startListening();

  // Wait here until we get a response, or timeout (250ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 200 )
      timeout = true;

  // Describe the results
  if ( timeout )
  {
    printf("Failed, response timed out.\n\r");
  }
  else
  {
    // Grab the response, compare, and send to debugging spew
    unsigned long got_time;
    radio.read( &got_time, sizeof(unsigned long) );

    // Spew it
    printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
  }

  delay(2000);
}
*/
