#include <PID_v1.h>

//INPUTS
const int currentPin = A2;
const int WHEATSTONE_CONSTANT = A1;
const int WHEATSTONE_SENSOR = A0;

//OUTPUTS
int heaterControl= 3;

//TARGETS
const int TEMPERATURE_TARGET = 34;

//PID VARIABLES FOR TEMP
double Kp=1, Ki=0.8,Kd=0; //NOT FINAL
double TempSetpoint, TempInput, TempOutput;
PID myTempPID(&TempInput, &TempOutput, &TempSetpoint, Kp, Ki, Kd, DIRECT);

void setup(){
Serial.begin(9600);
  pinMode(WHEATSTONE_CONSTANT,INPUT);
  pinMode(WHEATSTONE_SENSOR,INPUT);
  pinMode(heaterControl, OUTPUT);
  pinMode(currentPin, INPUT);

//PID SETUP FOR TEMP
myTempPID.SetMode(AUTOMATIC);
TempSetpoint = TEMPERATURE_TARGET;


void loop(){
//PID CONTROL FOR TEMPERATURE
TempInput = getTemperature();
myTempPID.Compute(); //this figures out the TempOutput
//Serial.println(TempOutput);
analogWrite(heaterControl,255-TempOutput);
getCurrent();
delay(100);
}

float getTemperature(){
    int sensor1 = analogRead(WHEATSTONE_CONSTANT);
    float ConstantVoltage = sensor1 * (5.0 / 1023.0)+0.06;
    int sensor2 = analogRead(WHEATSTONE_SENSOR);
    float ChangingVoltage = sensor2 * (5.0 / 1023.0);
    float x = ConstantVoltage-ChangingVoltage;
    float mappedTemp = fmap(x,0.09,0.9,21,37);
    Serial.print("Temp:, ");
    Serial.print(mappedTemp);
    Serial.print(", dVoltage:, ");
    Serial.print(x);
    Serial.print(", Resistance:, ");
    Serial.println((10000*ChangingVoltage)/(8.7-ChangingVoltage));
return mappedTemp;
}

float fmap(float x, float x1, float x2, float y1, float y2){
 return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}

float getCurrent(){
  float x = 0;
  float current;
for(int i = 0; i < 200; i++){ //creates average 
 x = x +analogRead(currentPin);
delay(1);
        }  
 current = x*(5.0 / 1023.0)/199;

Serial.print("Current:, ");
Serial.println(current/95*1000); //voltage to current conversion
return current95*1000; 
}
