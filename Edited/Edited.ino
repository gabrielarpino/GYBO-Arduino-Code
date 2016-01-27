///////////////Bluetooth//////////////////////////////////////////

// This version uses call-backs on the event and RX so there's no data handling in the main loop!

#include <SPI.h>
#include <math.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);


//This function is called whenever select ACI events happen

////////////////// Temperature //////////////////////////////////

#include <Wire.h>

int tmp102Address = 0x48;


////////////////// Moisture sensor /////////////////////////////////

  //Example code for the moisture sensor
  // Connect the sensor to the A0(Analog 0) pin on the Arduino board
   
  // the sensor value description
  // 0  ~300     dry soil
  // 300~700     humid soil
  // 700~950     in water


/////////////////////////// light sensor ///////////////////////////////////

int LDR_Pin = A1;


//////////////////////////Humidity Sensor///////////////////////////////

int HIH4030_Pin = A2; //analog pin 0

///////////////////////// Force Sensor/////////////////////////////////

//From the article: http://bildr.org/2012/11/force-sensitive-resistor-arduino

int FSR_Pin = A3; //analog pin 0

/////////////////////// Motor ////////////////////////////////////////
// motor A connected between A01 and A02, motor B connected between B01 and B02

int STBY = 6; //standby

//Motor A
int PWMA = 0; //Speed control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

//Motor B
int PWMB = 3; //Speed Control
int BIN1 = 5;//Direction
int BIN2 = 4;//Direction


//*************************FUNCTIONS**************************//

//-----------------------Temperature------------------------------
float getTemperature(){
    Wire.requestFrom(tmp102Address,2);

    byte MSB = Wire.read();
    byte LSB = Wire.read();

    int TemperatureSum = ((MSB << 8) | LSB) >> 4;

    float celsius = TemperatureSum*0.0625;
    return celsius;
}

//-------------- Humidity -----------------------------------------

float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(HIH4030_Pin);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}

//-------------- Motor - Functions --------------------------
void stop(){
  //enable standby
  digitalWrite(STBY, LOW);
}

void move(int motor, int speed, int direction){
  //Move specific motor at speed and direction
  //Motor: 0 for B. 1 for A
  //Speed: 0 is off, 255 is full speed
  //Direction:0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;

  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

//-------------------------------Bluetooth---------------------------

void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      break;
    default:
      break;
  }
}

/*!
    This function is called whenever data arrives on the RX channel
*/
void rxCallback(uint8_t *buffer, uint8_t len)
{
  int gotTrigger = 0;
  int isEmpty;
  float radius = 55;
  float cup_height = 90;
  float water_height = 60; // Make tape after
  float theta;
  double initial_weight = 700;
  double weight_ratio;
  double delay_time;
  double rotation_speed = 5*2*(3.141592653589793238462643383279502)/3200;

  int FSRReading = analogRead(FSR_Pin);
  int final_force = 300;
  
  //Serial.print(F("Received "));
  //Serial.print(len);
  //Serial.print(F(" bytes: "));

  // Watering Algorithm

  if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 't' && buffer[3] == 'e' && buffer[4] == 'r'){
    // all units in mm

    int FSRReading = analogRead(FSR_Pin);
    if ((FSRReading - initial_weight) < 10 | (initial_weight - FSRReading) < 10){
      isEmpty = 0;
    }

    if (isEmpty != 1) {

      if (FSRReading < final_force){

        uart.print("Water is at a low level!");

        move(2,10,1);
        move(1,10,2);

        delay_time = (3.145926 - theta)/rotation_speed;
        delay(delay_time);

        stop();

        uart.print("Plant watered!");

        uart.print("Cup is empty!");

        isEmpty = 1;
        
      }

      if (FSRReading > final_force) {

  
      // Calculate initial angle change from equation

      theta = atan( radius / (cup_height - water_height));

      float theta_to_fall = theta + 0.3;

      delay_time = theta_to_fall/rotation_speed;

        move(2,50,1);
        move(1,50,2);

       delay(delay_time);

       stop();

     // Measure weight

       FSRReading = analogRead(FSR_Pin); 

       weight_ratio = initial_weight/FSRReading;

       water_height = weight_ratio*water_height; 

       uart.print("Plant Watered! ");

      }
  }

  if (buffer[0] == 'x'){
  stop();}
  
  if (buffer[0] == 's' && buffer[1] == 't' && buffer[2] == 'a' && buffer[3] == 't' && buffer[4] == 'u' && buffer[5] == 's'){
    gotTrigger = 1;
  }
  
  //for(int i=0; i<len; i++)
  //{
  // Serial.print((char)buffer[i]); 
  //}
  //Serial.print(F(" ["));

  //for(int i=0; i<len; i++)
  //{
   // Serial.print(" 0x"); Serial.print((char)buffer[i], HEX); 
  //}
  //Serial.println(F(" ]"));

  /* Echo the same data back! */
  //uart.write(buffer, len);

//---------------------------INIT---------------------------------------
 /* int soiltype;
  int plant;
  int humid;
  int light;
  int temp;
  int suit = 0;

  if (buffer[0] == 'r' && buffer[1] == 'e' && buffer[2] == 's' && buffer[3] == 'e' && buffer[4] == 't'){
    plant = 0;
  }

  if (plant == 0){
    soiltype = 0;
    humid = 0;
    light = 0;
    temp = 0;
  }
  
  if (buffer[0] == 's' && buffer[1] == 't' && buffer[2] == 'a' && buffer[3] == 'r' && buffer[4] == 't'){
    plant = 1;
    
    uart.print("Soil: dry or humid?");
    if (buffer[0] == 'd' && buffer[1] == 'r' && buffer[2] == 'y'){
      soiltype = 1;

      uart.print("Humid: high, low, med?");
      if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
        humid = 1;

        uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
     
          
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light = 2;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }

        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }
      
      if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          humid = 2;

          uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light = 2;
          
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
          
        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }
          
      if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
        humid = 3;
        
        uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light = 2;
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        
        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }
          
    }
    if (buffer[0] == 'h' && buffer[1] == 'u' && buffer[2] == 'm' && buffer[3] == 'i' && buffer[4] == 'd'){
      soiltype = 2;

     uart.print("Humid: high, low, med?");
      if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
        humid = 1;

        uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
     
          
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light = 2;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }

        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }
      
      if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          humid = 2;

          uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light = 2;
          
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
          
        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }
          
      if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
        humid = 3;
        
        uart.print("Light: high, med, low?");
        if (buffer[0] == 'h' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h'){
          light = 1;
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        
        if (buffer[0] == 'm' && buffer[1] == 'e' && buffer[2] == 'd'){
          light =2;
          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
        
        if (buffer[0] == 'l' && buffer[1] == 'o' && buffer[2] == 'w'){
          light = 3;

          uart.print("Temp: freeze, cool, room, warm, hot?");
          if (buffer[0] == 'f' && buffer[1] == 'r' && buffer[2] == 'e' && buffer[3] == 'e' && buffer[4] == 'z' && buffer[5] == 'e'){
            temp = 1;
          }
          if (buffer[0] == 'c' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'l'){
            temp = 2;
          }
          if (buffer[0] == 'r' && buffer[1] == 'o' && buffer[2] == 'o' && buffer[3] == 'm'){
            temp = 3;
          }
          if (buffer[0] == 'w' && buffer[1] == 'a' && buffer[2] == 'r' && buffer[3] == 'm'){
            temp = 4;
          }
          if (buffer[0] == 'h' && buffer[1] == 'o' && buffer[2] == 't'){
            temp = 5;
          }
        }
      }

      
    }

    
    
  }

*/


  if (gotTrigger == 1)
  {


    /////////////////////// Moisture Sensor /////////////////////////////////////////

    Serial.begin(57600);

    //Serial.print("Moisture Sensor Value:");
    int moist = analogRead(A0);
    Serial.begin(9600);
    
    if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is dry");
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil has overflowed with water");
    }
    if (moist > 300 && moist < 700){
      uart.print("The soil is humid");
    }
    delay(5000);
    //Serial.println("We got our TRIGGER!");
    //uart.print("TRIGGER");


    ////////////////////// Temperature /////////////////////////

    float celsius = getTemperature();
    uart.print("Celsius:  ");
    uart.print(celsius);
    delay(5000);

    //////////////////////// Light sensor //////////////////////////

    int LDRReading = analogRead(LDR_Pin);
    if (LDRReading < 150 | LDRReading == 0 | (LDRReading == 150)){
      uart.print("It is bright");
    }
    if ((LDRReading > 150 && LDRReading < 512) | LDRReading == 512){
      uart.print("It is well lit");
    }
    if ((LDRReading > 512) && (LDRReading < 800)){
      uart.print("It is dark");
    }
    if ((LDRReading == 800) && (LDRReading > 800)){
      uart.print("It is very dark");
    }
    delay(5000);

    //////////////////////// Humidity ////////////////////////

    //To properly caculate relative humidity, we need the temperature.
    float temperature = celsius; //replace with a thermometer reading if you have it
    float relativeHumidity  = getHumidity(temperature);

    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels uncomfortably dry. Relative Humidity: ");
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels uncomfortably wet. Relative Humidity: ");
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It feels comfortable. Relative Humidity: ");
    }
    uart.print(relativeHumidity);
    
    delay(5000); //just here to slow it down so you can read it   

    ///////////////////// Force Sensor ///////////////////////

    int FSRReading = analogRead(FSR_Pin); 

    uart.print("Force: ");
    uart.print(FSRReading);
    delay(250); //just here to slow down the output for easier reading
}
//////////////////////////// Individual Stats ///////////////////////////////

////////////////////////// Moisture //////////////////////////////////////
  
  if (buffer[0] == 'm' && buffer[1] == 'o' && buffer[2] == 'i' && buffer[3] == 's' && buffer[4] == 't' && buffer[5] == 'u' && buffer[6] == 'r' && buffer[7] == 'e'){
    Serial.begin(57600);
    //Serial.print("Moisture Sensor Value:");
    int moist = analogRead(A0);
    Serial.begin(9600);

    //if (soiltype == 0){
    
    if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is dry");
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil has overflowed with water");
    }
    if (moist > 300 && moist < 700){
      uart.print("The soil is humid");
    }
    //}
/*
    if (soiltype == 1){
      if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is perfect");
      suit += 1;
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil is too damp");
    }
    if (moist > 300 && moist < 700){
      uart.print("Your plant is in danger from water!");
      suit -= 1;
    }
    }

    if (soiltype == 2){
      if (moist < 300 | moist == 300 | moist == 0){
      uart.print("The soil is too dry");
    }
    if (moist > 700 | moist == 700){
      uart.print("The soil is perfect");
      suit += 1;
    }
    if (moist > 300 && moist < 700){
      uart.print("Your plant is in danger from water!");
    }
    }
    
*/
    }
////////////////////////////// Temp ///////////////////////////////////
  if (buffer[0] == 't' && buffer[1] == 'e' && buffer[2] == 'm' && buffer[3] == 'p' && buffer[4] == 'e' && buffer[5] == 'r' && buffer[6] == 'a' && buffer[7] == 't' && buffer[8] == 'u' && buffer[9] == 'r' && buffer[10] == 'e'){
    float celsius = getTemperature();

  //  if (temp == 0){
    uart.print("Celsius:  ");
    uart.print(celsius);
    }
/*
    if (temp == 1){
      if (celsius < 0 | celsius == 0){
        uart.print("Temperature is perfect");
        suit += 1;
      }
      if ((celsius > 0 && celsius < 15) | celsius == 15){
        uart.print("Temperature is a little too warm");
      }
      if ((celsius > 15 && celsius < 25) | celsius == 25){
        uart.print("Temperature is too warm");
        suit -= 1;
      }
      if ((celsius > 25 && celsius < 32) | celsius == 32){
        uart.print("Temperature is too hot!");
        suit -= 1;
      }
      if (celsius > 32){
        uart.print("Your plant will die from heat!");
        suit -= 1;
      }
    }

    if (temp == 2){
      if (celsius < 0 | celsius == 0){
        uart.print("Temperature is too cold");
      }
      if ((celsius > 0 && celsius < 15) | celsius == 15){
        uart.print("Temperature is perfect");
        suit += 1;
      }
      if ((celsius > 15 && celsius < 25) | celsius == 25){
        uart.print("Temperature is too warm");
      }
      if ((celsius > 25 && celsius < 32) | celsius == 32){
        uart.print("Temperature is too hot!");
        suit -= 1;
      }
      if (celsius > 32){
        uart.print("Your plant will die from heat!");
        suit -= 1;
      }
    }

     if (temp == 3){
      if (celsius < 0 | celsius == 0){
        uart.print("Temperature is too cold");
        suit -= 1;
      }
      if ((celsius > 0 && celsius < 15) | celsius == 15){
        uart.print("Temperature is a little too cold");
      }
      if ((celsius > 15 && celsius < 25) | celsius == 25){
        uart.print("Temperature is perfect");
        suit += 1;
      }
      if ((celsius > 25 && celsius < 32) | celsius == 32){
        uart.print("Temperature is too warm!");
      }
      if (celsius > 32){
        uart.print("Temperature is too hot!");
        suit -= 1;
      }
    }

     if (temp == 4){
      if (celsius < 0 | celsius == 0){
        uart.print("Your plant will die from the cold!");
        suit -= 1;
      }
      if ((celsius > 0 && celsius < 15) | celsius == 15){
        uart.print("Temperature is too cold");
        suit -= 1;
      }
      if ((celsius > 15 && celsius < 25) | celsius == 25){
        uart.print("Temperature is a little too coo");
      }
      if ((celsius > 25 && celsius < 32) | celsius == 32){
        uart.print("Temperature is perfect!");
        suit += 1;
      }
      if (celsius > 32){
        uart.print("Temperature is too warm!");
      }
    }

    if (temp == 5){
      if (celsius < 0 | celsius == 0){
        uart.print("Your plant will die from the cold!");
        suit -= 1;
      }
      if ((celsius > 0 && celsius < 15) | celsius == 15){
        uart.print("Temperature is far too cold");
        suit -= 1;
      }
      if ((celsius > 15 && celsius < 25) | celsius == 25){
        uart.print("Temperature is too cool");
        suit -= 1;
      }
      if ((celsius > 25 && celsius < 32) | celsius == 32){
        uart.print("Temperature is a little too cool");
      }
      if (celsius > 32){
        uart.print("Temperature is perfect!");
        suit += 1;
      }
    }
    */
 // }
////////////////////////// Humid //////////////////////////////////////
  if (buffer[0] == 'h' && buffer[1] == 'u' && buffer[2] == 'm' && buffer[3] == 'i' && buffer[4] == 'd' && buffer[5] == 'i' && buffer[6] == 't' && buffer[7] == 'y'){
    float celsius = getTemperature();
    float temperature = celsius; //replace with a thermometer reading if you have it
    float relativeHumidity  = getHumidity(temperature);

   // if (humid == 0){
    
    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels uncomfortably dry. Relative Humidity: ");
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels uncomfortably wet. Relative Humidity: ");
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It feels comfortable. Relative Humidity: ");
    }
    uart.print(relativeHumidity);
  }
/*
  if (humid == 3){
    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("Perfect! It's dry");
      suit += 1;
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels uncomfortably wet");
      suit -= 1;
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It feels too wet");
    }
  }

  if (humid == 2){
    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels too dry");
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("It feels too wet");
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("Perfect!");
      suit += 1;
    }
  }

  if (humid == 1){
    if (relativeHumidity < 25 | relativeHumidity == 0 | (relativeHumidity == 25)){
      uart.print("It feels uncomfortably dry");
      suit -= 1;
    }
    if (relativeHumidity > 60 | relativeHumidity == 60){
      uart.print("Perfect");
      suit += 1;
    }
    if (relativeHumidity < 60 && relativeHumidity > 25){
      uart.print("It is too dry");
    }
  }

  }
*/
///////////////////////// Light ////////////////////////////////////////
  if (buffer[0] == 'l' && buffer[1] == 'i' && buffer[2] == 'g' && buffer[3] == 'h' && buffer[4] == 't'){
    int LDRReading = analogRead(LDR_Pin);


   // if (light == 0){
    if (LDRReading < 150 | LDRReading == 0 | (LDRReading == 150)){
      uart.print("It is bright");
    }
    if ((LDRReading > 150 && LDRReading < 512) | LDRReading == 512){
      uart.print("It is well lit");
    }
    if ((LDRReading > 512) && (LDRReading < 800)){
      uart.print("It is dark");
    }
    if ((LDRReading == 800) && (LDRReading > 800)){
      uart.print("It is very dark");
    }
   }
/*
  if (light == 1){
    if (LDRReading < 450 | LDRReading == 0 | (LDRReading == 450)){
      uart.print("It is bright. Perfect!");
      suit += 1;
    }
    if ((LDRReading > 450 && LDRReading < 650) | LDRReading == 650){
      uart.print("Too dark");
    }
    if (LDRReading > 650){
      uart.print("Your plant isn't getting enough light!");
      suit -= 1;
    }
  }

  if (light == 2){
    if (LDRReading < 450 | LDRReading == 0 | (LDRReading == 450)){
      uart.print("Too bright!");
    }
    if ((LDRReading > 450 && LDRReading < 650) | LDRReading == 650){
      uart.print("Perfect");
      suit += 1;
    }
    if (LDRReading > 650){
      uart.print("Too dark");
    }
  }

  if (light == 1){
    if (LDRReading < 450 | LDRReading == 0 | (LDRReading == 450)){
      uart.print("Your plant is getting too much light!");
      suit -= 1;
    }
    if ((LDRReading > 450 && LDRReading < 650) | LDRReading == 650){
      uart.print("Too bright");
    }
    if (LDRReading > 650){
      uart.print("Dark and perfect!");
      suit += 1;
    }
  }
  
*/
  
//////////////////// Force /////////////////////////////////////////////
    if (buffer[0] == 'f' && buffer[1] == 'o' && buffer[2] == 'r' && buffer[3] == 'c' && buffer[4] == 'e'){
    int FSRReading = analogRead(FSR_Pin); 

    uart.print("Force: ");
    uart.print(FSRReading);}

/*
/////////////////// Suit ////////////////////////////////////
if (suit == 4){
  uart.print("The env is perfect for your plant!");
}
if (suit > 0 && suit != 4){
  uart.print("The env is almost perf");
}
if (suit == 0 | suit < 0){
  if (soiltype != 0){
    uart.print("Move your plant!");
  }
}

*/

  }
}



//*********************** SETUPPPPPP *********************************//

void setup() {

  //////////////////////// Bluetooth ////////////////////////////////

  // Configure the Arduino and start advertising with the radio
  
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));

  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  uart.setDeviceName("nulll"); /* 7 characters max! */
  uart.begin();

  ////////////////// moisture sensor /////////////////////////////////

  //Serial.begin(9600);

  ///////////////////  temperature   /////////////////////////////////
  Wire.begin();

  ///////////////////// humidity /////////////////////////////////

  //Serial.begin(9600);

  ////////////////// Motor ////////////////////////////////////////
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  

}

//*********************** LOOP *************************************//

void loop() {

  ///////////////// Bluetooth ///////////////////////////////////////

  uart.pollACI();

//////////////////////// light sensor //////////////////////////

  int LDRReading = analogRead(LDR_Pin);
  //Serial.println(LDRReading);
  //delay(1000);

////////////////////// Temperature /////////////////////////

  float celsius = getTemperature();
  //Serial.print("Celsius:  ");
  //Serial.println(celsius);
  //delay(1000);

//////////////////////// Humidity ////////////////////////

 //To properly caculate relative humidity, we need the temperature.
//float temperature = 25; //replace with a thermometer reading if you have it
  

  //Serial.println(relativeHumidity);

  //delay(100); //just here to slow it down so you can read it

///////////////////// Motor /////////////////////////////////
  

///////////// Temperature - Function //////////////////////
//float getTemperature(){
//  Wire.requestFrom(tmp102Address,2);
////float relativeHumidity  = getHumidity(temperature);
//  byte MSB = Wire.read();
//  byte LSB = Wire.read();
//
//  int TemperatureSum = ((MSB << 8) | LSB) >> 4;

//  float celsius = TemperatureSum*0.0625;
//  return celsius;
//}



}
