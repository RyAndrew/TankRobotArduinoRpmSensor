#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;


//this code measures the difference between two rising edges of the digitalised signal coming from hall sensor and then prints the rpm.

//pin A0 & A1 is the signal pin

float motorPulsePerRev = 75; // this is specific to your motor hall sensor setup.
//I am using a motor with 1 tick per hall sensor revolution with a 75:1 gear output

int edgeTransitionThreshold = 500; //for converting the analog signal coming from hall sensor to digital through arduino code

 
unsigned int motorA_pulseCounter = 0;
bool motorA_edgeReading;
bool motorA_edgeReadingPrevious = 0;
float motorA_Rpm = 0;
bool motorA_LastSerialOutputZero = 0;


unsigned int motorB_pulseCounter = 0;
bool motorB_edgeReading;
bool motorB_edgeReadingPrevious = 0;
float motorB_Rpm = 0;
bool motorB_LastSerialOutputZero = 0;


char floatToStringBuffer[9];

unsigned long pulseAvgOutputTime = millis();
unsigned long pulseAvgInterval = 200; //ms
float pulseAvgIntervalMinute = 300; // 60,000 / 200 = 300 intervals per minute (used for rpm calculation)


unsigned long batteryReadTime = millis();
unsigned long batteryReadInterval = 100; //ms

unsigned long batteryOutputTime = millis();
unsigned long batteryOutputInterval = 1000; //ms

int batteryAverageCount = 0;

float batteryPowerAverageTotal;
//float batteryPowerAverage;
float batteryLoadVoltageAverageTotal;
//float batteryLoadVoltageAverage;

void readBatterySensor()
{
batteryAverageCount++;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
//float power_mW = 0;

shuntvoltage = ina219.getShuntVoltage_mV();
busvoltage = ina219.getBusVoltage_V();

current_mA = ina219.getCurrent_mA();
//power_mW = ina219.getPower_mW();
batteryPowerAverageTotal += current_mA;

loadvoltage = busvoltage + (shuntvoltage / 1000);
batteryLoadVoltageAverageTotal += loadvoltage;

//Serial.println("readBatterySensor");
//
//Serial.print(" power_mW=");
//Serial.print(power_mW);
//Serial.print(" loadvoltage=");
//Serial.print(loadvoltage);
//
//Serial.print(" batteryPowerAverageTotal=");
//Serial.print(batteryPowerAverageTotal);
//
//Serial.print(" batteryLoadVoltageAverageTotal=");
//Serial.print(batteryLoadVoltageAverageTotal);
//
//Serial.print(" batteryAverageCount=");
//Serial.print(batteryAverageCount);
//Serial.println();
}
 void setup()
 {   
//   Serial.begin(115200);
   Serial1.begin(115200);
   pinMode(A0, INPUT);//Motor B
   pinMode(A1, INPUT);//Motor A
//   delay(2000);
//   Serial1.println("ready");

  ina219.begin();
  //ina219.setCalibration_16V_400mA();
  ina219.setCalibration_32V_1A();
 }
 void loop()
 {
  int reading;

  //Read Motor A
   reading = analogRead(A1); //read raw value of hall sensor
   if (reading > edgeTransitionThreshold) {
    motorA_edgeReading = 1; //convert it to digital 0,1 form
   }else{
    motorA_edgeReading = 0;
   }
   if(motorA_edgeReadingPrevious==0 && motorA_edgeReading==1) {//check for rising edge
   //if (valA1previous != valA1 ) { //check for transition
    motorA_pulseCounter++;
   }
   motorA_edgeReadingPrevious = motorA_edgeReading;


  //Read Motor B
   reading = analogRead(A0); //read raw value of hall sensor
   if (reading > edgeTransitionThreshold) {
    motorB_edgeReading = 1; //convert it to digital 0,1 form
   }else{
    motorB_edgeReading = 0;
   }
   if(motorB_edgeReadingPrevious==0 && motorB_edgeReading==1) {//check for rising edge
   //if (valA1previous != valA1 ) { //check for transition
    motorB_pulseCounter++;
   }
   motorB_edgeReadingPrevious = motorB_edgeReading;

  if(millis() - batteryReadTime > batteryReadInterval){
    batteryReadTime = millis();
    readBatterySensor();
  }
  if(millis() - batteryOutputTime > batteryOutputInterval){
    batteryOutputTime = millis();
    batteryPowerAverageTotal = batteryPowerAverageTotal / batteryAverageCount;
    batteryLoadVoltageAverageTotal = batteryLoadVoltageAverageTotal / batteryAverageCount;
    
    Serial1.print("BP=");
    Serial1.print(batteryPowerAverageTotal);
    Serial1.print("\n");
    
    Serial1.print("BV=");
    Serial1.print(batteryLoadVoltageAverageTotal);
    Serial1.print("\n");
    
//    Serial.print("BP=");
//    Serial.print(batteryPowerAverageTotal);
//    Serial.print("\n");
//    
//    Serial.print("BV=");
//    Serial.print(batteryLoadVoltageAverageTotal);
//    Serial.print("\n");
    
    batteryAverageCount = 0;
    batteryPowerAverageTotal = 0;
    batteryLoadVoltageAverageTotal = 0;
  }

  if(millis() - pulseAvgOutputTime > pulseAvgInterval){
    pulseAvgOutputTime = millis();
    
    if(!motorA_LastSerialOutputZero || motorA_pulseCounter > 0){
      if(motorA_pulseCounter == 0){
        motorA_LastSerialOutputZero = 1;
      }else{
        motorA_LastSerialOutputZero = 0;
      }
//      Serial1.println("--------------");
//      Serial1.print("pulseCounterA=");
//      Serial1.println(pulseCounterA);
      motorA_Rpm = (float)motorA_pulseCounter / motorPulsePerRev; //convert pulses to rotations
//      Serial1.print("pulseCounterA rotations="); 
//      Serial1.println(pulseCounterArpm);
      motorA_Rpm = motorA_Rpm * pulseAvgIntervalMinute; //convert rotations minute
      //format float with 3 places and .1 precision
      dtostrf (motorA_Rpm, 3, 1, floatToStringBuffer);

      Serial1.print("MA=");
      Serial1.print(floatToStringBuffer);
      Serial1.print("\n");
      
      //Serial1.print("MA=");
      //Serial1.print(floatToStringBuffer);
      //Serial1.print("\n");

       //delay(20);
    }
    
    if(!motorB_LastSerialOutputZero || motorB_pulseCounter > 0){
      if(motorB_pulseCounter == 0){
        motorB_LastSerialOutputZero = 1;
      }else{
        motorB_LastSerialOutputZero = 0;
      }
//      Serial1.println("--------------");
//      Serial1.print("pulseCounterA=");
//      Serial1.println(pulseCounterA);
      motorB_Rpm = (float)motorB_pulseCounter / motorPulsePerRev; //convert pulses to rotations
//      Serial1.print("pulseCounterA rotations="); 
//      Serial1.println(pulseCounterArpm);
      motorB_Rpm = motorB_Rpm * pulseAvgIntervalMinute; //convert rotations minute
      //format float with 3 places and .1 precision
      dtostrf (motorB_Rpm, 3, 1, floatToStringBuffer);

      Serial1.print("MB=");
      Serial1.print(floatToStringBuffer);      
      Serial1.print("\n");
      
      //Serial1.print("MB=");
      //Serial1.print(floatToStringBuffer);      
      //Serial1.print("\n");
    }
    motorA_pulseCounter = 0;
    motorB_pulseCounter = 0;
  }
 }
