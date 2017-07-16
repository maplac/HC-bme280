

#include <Arduino.h>

#include <Ports.h>

#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

#include <SparkFunBME280.h>
#include <stdint.h>

#define I2C_ADDRESS 0x3C
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
SSD1306AsciiWire oled;

int analogPin = 0;
int buttonPin = 2;
int oledNotEnabledPin = 10;
int ledPin = 9;

/*
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18(&oneWire);
*/

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 (CE, CS)
RF24 radio(15, 16);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

uint8_t counter;

//Global sensor object
BME280 mySensor;

volatile bool isTimeout = false;
volatile bool isButton = false;
volatile bool isButtonUp = false;
volatile bool isButtonDown = false;
ISR(WDT_vect) {
  Sleepy::watchdogEvent();
  isTimeout = true;
}
// ISR(PCINT2_vect) {
//   isButton = true;
// }
void buttonInt(void){
   isButton = true;
}

void setup(void){

  pinMode(buttonPin,INPUT);
  /*
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT22);
  MCUCR = (1<<ISC01) | (1<<ISC01);
  bitSet(PCMSK2, buttonPin); //handler for interrupt
  bitSet(PCICR, PCIE2); //handler for interrupt
  */
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInt, CHANGE );

  pinMode(oledNotEnabledPin, OUTPUT);
  digitalWrite(oledNotEnabledPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  delay(20); // Just to get a solid reading on the role pin


  Serial.begin(57600);

  printf_begin();

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


    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

    //radio.openWritingPipe(pipes[1]);
  // radio.openReadingPipe(1,pipes[0]);


  radio.startListening();
  radio.printDetails();

  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
  //mySensor.settings.commInterface = I2C_MODE;
  //mySensor.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  mySensor.settings.commInterface = SPI_MODE;
  mySensor.settings.chipSelectPin = 17;
  //***Operation settings*****************************//
  //renMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  mySensor.settings.runMode = 1; //Normal mode

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


  Wire.begin();
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.set2X();
  oled.clear();


  //analogReference(INTERNAL);

  //DS18.begin();
}

void loop(void){

    isTimeout = false;
    isButton = false;

    // Sleepy::powerDown();
    Sleepy::loseSomeTime(5000);
    digitalWrite(ledPin, HIGH);


    int val = analogRead(analogPin);
    //float voltage = val * 5 / 1024.0;//0.0537;//0.0118;
    float voltage = (val+21) * 5.0 * 10.8 / 1024;
    float voltage2 = (val+21) * 5.0 / 1024;
    float ds18Temp = 123.456;
    /*
    Serial.print(" Requesting temperatures...");
    DS18.requestTemperatures(); // Send the command to get temperature readings
    Serial.println("DONE");
    Serial.print("Temperature is: ");
    ds18Temp = DS18.getTempCByIndex(0);
    Serial.println(ds18Temp);//Serial.print("\n");
    */

    uint8_t dataToWrite = (mySensor.settings.tempOverSample << 0x5) & 0xE0;
  	//Next, pressure oversampling
  	dataToWrite |= (mySensor.settings.pressOverSample << 0x02) & 0x1C;
  	//Last, set mode
  	dataToWrite |= (mySensor.settings.runMode) & 0x03;
    mySensor.writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
    delay(10); // 1.25+2.3+2.3+0.575+2.3+0.575 = 9.3
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


    oled.home();
    //display.setTextSize(1);
    //display.print("T[");display.print((char)247);display.println("C],RH[%],P[Pa]");
    oled.print(temperature);oled.print(" ");oled.print((char)247);oled.println("C");
    oled.print(humidity);oled.println(" %");
    oled.print(round(pressure));oled.println(" Pa");
    // oled.print(ds18Temp);oled.print(" ");oled.print((char)247);oled.println("C");
    oled.print(counter);oled.print(" ");
    if (isTimeout){
      oled.print("T");
    }else{
      oled.print("t");
    }

    if (isButton){
      oled.print("B");
    }else{
      oled.print("b");
    }
    oled.print(" ");
    if (digitalRead(buttonPin)){
      oled.print("U");
    }else{
      oled.print("D");
    }

    oled.print("    ");


    //oled.print(voltage);oled.print(" ");
    //oled.print(voltage2);

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

    digitalWrite(ledPin, LOW);
    Serial.println("\n=========================================");
    delay(20);
    // delay(1000);

}
