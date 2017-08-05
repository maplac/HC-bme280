

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>
#include "printf.h"

#include <Ports.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SparkFunBME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#define DEBUG_ENABLED

#define I2C_ADDRESS 0x3C

SSD1306AsciiWire oled;
BME280 bme280Sensor;
RF24 radio(15, 16); // Set up nRF24L01 radio on SPI bus (CE, CS)

int pinAnalog = 0;
int pinButton1 = 2;
int pinButton2 = 3;
int pinOledNotEnabled = 10;
int pinLed = 9;

uint8_t counterPackets = 0;
float temperature = 0;
float humidity = 0;
float pressure = 0;
float voltage = 0;
bool isFirstRun = true;
uint8_t buttonLastState = 3;

uint32_t counterSendAttempts = 0;
uint32_t counterSendFailed = 0;

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
volatile bool isTimeout = false;
volatile bool isButton = false;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
  isTimeout = true;
}

void buttonInt(void){
  isButton = true;
}

void setup(void){

  pinMode(pinButton1, INPUT);
  pinMode(pinButton2, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinButton1), buttonInt, CHANGE );
  attachInterrupt(digitalPinToInterrupt(pinButton2), buttonInt, CHANGE );

  pinMode(pinOledNotEnabled, OUTPUT);
  digitalWrite(pinOledNotEnabled, LOW);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  Serial.begin(57600);
  printf_begin();

  radio.begin();
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  // optionally, reduce the payload size.  seems to improve reliability
  //radio.setPayloadSize(8);
  // Open 'our' pipe for writing
  radio.openWritingPipe(pipes[0]);
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)
  radio.openReadingPipe(1,pipes[1]);
  //radio.openWritingPipe(pipes[1]);
  //radio.openReadingPipe(1,pipes[0]);
  // radio.startListening();
  radio.stopListening();

  #ifdef DEBUG_ENABLED
    radio.printDetails();
  #endif

  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
  //mySensor.settings.commInterface = I2C_MODE;
  //mySensor.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  bme280Sensor.settings.commInterface = SPI_MODE;
  bme280Sensor.settings.chipSelectPin = 17;
  //***Operation settings*****************************//
  //renMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  bme280Sensor.settings.runMode = 1; //Normal mode

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  bme280Sensor.settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  bme280Sensor.settings.filter = 0;

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme280Sensor.settings.tempOverSample = 1;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme280Sensor.settings.pressOverSample = 1;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme280Sensor.settings.humidOverSample = 1;

  //Calling .begin() causes the settings to be loaded
  //Serial.print("Starting BME280... result of .begin(): 0x");
  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  uint8_t res = bme280Sensor.begin();
  #ifdef DEBUG_ENABLED
    Serial.println(res, HEX);
  #endif


  Wire.begin();

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.set2X();
  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  // delay(100);
  // digitalWrite(pinOledNotEnabled, HIGH);

  analogReference(DEFAULT);//EXTERNAL
}

void loop(void){

    delay(20);
    if(isFirstRun){
      isFirstRun = false;
      isTimeout = true;
    }else{
      // Sleepy::powerDown();
      if (!isButton && !isTimeout){
        Sleepy::loseSomeTime(3000);
      }
    }

    // button was pressed or released
    if(isButton){
      isButton = false;
      Serial.println("Button interrupt");
//*
      // debounce timeout
      delay(10);
      uint8_t buttonState = (digitalRead(pinButton1) << 1) | digitalRead(pinButton2);

      // if a button actualy changed its state
      if (buttonLastState != 0 && buttonState != 0){
        switch(buttonLastState){
          case 1:{
            switch(buttonState){
              case 1:break;
              case 2:break;
              case 3:{
                // Wire.end();
                // digitalWrite(pinLed, LOW);
                oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
                // digitalWrite(pinOledNotEnabled, HIGH);
                // digitalWrite(18, LOW);
                // digitalWrite(19, LOW);
                // pinMode(18, INPUT);
                // pinMode(19, INPUT);
                // Wire.end();
                break;
              }
            }
            break;
          }
          case 2:{
            switch(buttonState){
              case 1:break;
              case 2:break;
              case 3:{
                // Wire.end();
                // digitalWrite(pinLed, LOW);
                oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
                // digitalWrite(pinOledNotEnabled, HIGH);
                // digitalWrite(18, LOW);
                // digitalWrite(19, LOW);
                // pinMode(18, INPUT);
                // pinMode(19, INPUT);

                break;
              }
            }
            break;
          }
          case 3:{
            switch(buttonState){
              case 1:{
                // digitalWrite(pinLed, HIGH);
                // pinMode(18, OUTPUT);
                // pinMode(18, OUTPUT);
                // digitalWrite(18, HIGH);
                // digitalWrite(19, HIGH);
                // digitalWrite(pinOledNotEnabled, LOW);
                // delay(50);
                // Wire.begin();

                // SSD1306AsciiWire oled;
                // oled.begin(&Adafruit128x64, I2C_ADDRESS);
                oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
                oled.clear();
                oled.home();
                oled.print(temperature);oled.print(" ");oled.print((char)247);oled.println("C");
                oled.print(humidity);oled.println(" %");
                oled.print(round(pressure));oled.println(" Pa");
                oled.print(voltage);oled.println(" V");
                delay(500);
                break;
              }
              case 2:{
                // digitalWrite(pinLed, HIGH);
                // pinMode(18, OUTPUT);
                // pinMode(18, OUTPUT);
                // digitalWrite(18, HIGH);
                // digitalWrite(19, HIGH);
                //
                // digitalWrite(pinOledNotEnabled, LOW);
                //
                // delay(50);
                // Wire.begin();
                // SSD1306AsciiWire oled;
                oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
                oled.clear();
                oled.home();
                oled.println(counterSendAttempts);
                oled.println(counterSendFailed);
                delay(500);
                break;
              }
              case 3:break;
            }
            break;
          }
        }
      }
      buttonLastState = buttonState;
//*/
    }


    if(isTimeout){
      isTimeout = false;
      // Serial.println("Timeout interrupt");
      digitalWrite(pinLed, HIGH);

      int val = analogRead(pinAnalog);
      voltage = val * 3.3 / 1024.0;

      uint8_t dataToWrite = (bme280Sensor.settings.tempOverSample << 0x5) & 0xE0;
      //Next, pressure oversampling
      dataToWrite |= (bme280Sensor.settings.pressOverSample << 0x02) & 0x1C;
      //Last, set mode
      dataToWrite |= (bme280Sensor.settings.runMode) & 0x03;
      bme280Sensor.writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
      delay(10); // 1.25+2.3+2.3+0.575+2.3+0.575 = 9.3
      temperature = bme280Sensor.readTempC();
      pressure = bme280Sensor.readFloatPressure();
      humidity = bme280Sensor.readFloatHumidity();

      #ifdef DEBUG_ENABLED
        Serial.print("Temperature: ");
        Serial.print(temperature, 2);
        Serial.println(" degrees C");
        Serial.print("Pressure: ");
        Serial.print(pressure, 2);
        Serial.println(" Pa");
        Serial.print("%RH: ");
        Serial.print(humidity, 2);
        Serial.println(" %");
      #endif

      radio.stopListening();

      char packet[32];
      packet[0] = 1; // id
      packet[1] = counterPackets; // packet counter
      packet[2] = 0; // packet type
      packet[3] = 0; // reserved

      float *packetF = (float*) &packet;
      packetF[1] = temperature;
      packetF[2] = pressure;
      packetF[3] = humidity;

      printf("counter=%d, ", counterPackets);
      bool ok = radio.write( &packet, 32 );
      counterPackets++;
      counterSendAttempts++;
      if (!ok){
        printf("write failed, ");
        counterSendFailed++;
      }
      radio.startListening();

      // Wait here until we get a response, or timeout (250ms)
      unsigned long started_waiting_at = millis();
      bool timeout = false;
      while ( ! radio.available() && ! timeout ){
        if (millis() - started_waiting_at > 200 ){
          timeout = true;
        }
      }
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

      digitalWrite(pinLed, LOW);
      Serial.println("\n=========================================");

    }

}

/*
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18(&oneWire);
*/
//DS18.begin();
/*
Serial.print(" Requesting temperatures...");
DS18.requestTemperatures(); // Send the command to get temperature readings
Serial.println("DONE");
Serial.print("Temperature is: ");
ds18Temp = DS18.getTempCByIndex(0);
Serial.println(ds18Temp);//Serial.print("\n");
*/
// ISR(PCINT2_vect) {
//   isButton = true;
// }

/*
PCICR |= (1<<PCIE2);
PCMSK2 |= (1<<PCINT22);
MCUCR = (1<<ISC01) | (1<<ISC01);
bitSet(PCMSK2, buttonPin); //handler for interrupt
bitSet(PCICR, PCIE2); //handler for interrupt
*/

/*
    oled.home();
    //display.setTextSize(1);
    //display.print("T[");display.print((char)247);display.println("C],RH[%],P[Pa]");
    oled.print(temperature);oled.print(" ");oled.print((char)247);oled.println("C");
    oled.print(humidity);oled.println(" %");
    oled.print(round(pressure));oled.println(" Pa");
    //oled.print(voltage);oled.print("   ");

    // oled.print(counter);oled.print(" ");
    if (isTimeout){
      oled.print("T");
    }else{
      oled.print(" ");
    }
    oled.print(" ");
    oled.print(buttonState);
    oled.print("    ");
*/
