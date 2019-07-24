#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#include "Relays.h"

//PIN 32 USZKODZONY
//PIN 31 USZKODZONY

#define DEBUG 0

#define distanceOn 1
#define moveSensorActivated 1
#define moveSensorActivated2 1

#define MOVESENSOR_1 1
#define MOVESENSOR_2 1

#define WINDOWSENSORFLOOR0 1
#define WINDOWSENSORFLOOR2 1



#define termometrsOn 1
//MAX FLOOR TEMPERATURE
#define MAX_TEMPERATURE 32.00
//MIN FLOOR TEMPERATURE
#define MIN_TEMPERATURE 18.00
#define temperatureFloor 26.00
#define temperatureFloorMin 21.00
long timeAlarmRed = 1000;
long timeAlarmDefault = 60000;
long timeAlarmBlue = 1000;
long DefaultUsageTimeAlarm = timeAlarmDefault;

unsigned long tryFixFloorTimer = 0;
bool tryFixFloorBool = false;


#define delayTimeError 60

#define PIN 7
#define LICZBADIOD 24

#define NONE 40 //Jasno Niebieski nie uzywany kontakt w sypialni
#define WindowSensorFloor0 52 //jasno brazowy dolutowany zielony
#define WindowSensorFloor2 42 //Niebieski czujnik otwarcia okna góra
#define PinLightFloor3 43 //jasno pomaranczowy
#define PinLightFloor2 41  //pomaranczowy
#define PinLightFloor1 39 //zielony
#define PinLightFloor0 37//niebieski
#define moveLight 29  //niebieski
#define insineMoveLIght 53 //pomaranczowy
#define echoPin 48  //Jasno brazowy
#define trigPin 50  //Brazowy
#define moveSensor 46
#define moveSensor2 38 //brazowyc
#define PinLightFloor_2 33 //jasno pomaranczowy przedsionek szafa
#define PinLightFloor_1 35 //pomaranczowy korytarz
#define moveSensorTerrace 45  // jasno brazowy
#define doorSensorTerrace 47  //pomarańczowy
#define floorHeating1 36 //LOW WL
#define floorHeating2 34 //HIGH WL
#define floorHeating3 30 //LOW WYL HIGH WL 

#define doorHalfFloor 51 //jasno pomaranczowy przedluzenie zielony

#define alarms 49 //zółty

void digitalReadAll(){
  Serial.print("PinLightFloor3: ");
  Serial.println(digitalRead(PinLightFloor3));
  Serial.print("PinLightFloor2: ");
  Serial.println(digitalRead(PinLightFloor2));
  Serial.print("PinLightFloor1: ");
  Serial.println(digitalRead(PinLightFloor1));
  Serial.print("PinLightFloor0: ");
  Serial.println(digitalRead(PinLightFloor0));
  Serial.print("moveLight: ");
  Serial.println(digitalRead(moveLight));
  Serial.print("insineMoveLIght: ");
  Serial.println(digitalRead(insineMoveLIght));
  Serial.print("PinLightFloor_2: ");
  Serial.println(digitalRead(PinLightFloor_2));
  Serial.print("PinLightFloor_1: ");
  Serial.println(digitalRead(PinLightFloor_1));
  
}


#define moveSensor_1 28 //korytarz brazowy
#define moveSensor_2 27 //przedsionek

#define moveSensorLR 44 //czujnik ruchu salon czarny 

#define lightTerrace 26



#define ONE_WIRE_BUS 6
#define TEMPERATURE_PRECISION 9

#define analogMQ2 A8




// Number of sensors
const byte SENSORS_NUM = 5;
int readLightsTemperatureAlarm[4];
int rgbAlarm[3];
bool readOnce = true;

unsigned long timeTryFix = 0;

 DeviceAddress t1={
0x28, 0xFF, 0xB6, 0xE4, 0x30, 0x17, 0x04, 0xE1  };
 DeviceAddress t2= {
0x28, 0xFF, 0xBE, 0x22, 0x31, 0x17, 0x04, 0x44  };
 DeviceAddress t3= {
0x28, 0xFF, 0xD9, 0xE1, 0x30, 0x17, 0x04, 0x1E  };
 DeviceAddress t4= {
0x28, 0xFF, 0x95, 0x1F, 0x31, 0x17, 0x04, 0x19  };
DeviceAddress t5 = {
0x28, 0xFF, 0xA7, 0x20, 0x31, 0x17, 0x04, 0xCC};



int pins3[] = {22,23,24,25,lightTerrace,29,37,39,41,43,53};
bool gasAlarm = false;
bool gasAlarmOnce = true;

bool canOffOnLRlight = true;

bool timeLightOff30 = false;


//Communication variables
//WIFI Nodemcu
String sRec1 = "";
int value1 = 0;
//Arduino nano
String sRec = "";
int value = 0;
//-------------------
//Near/Far gesture
bool NearFar = false;
//Relay Counter max is size array
int counter = 0;
//Temp variable save previous state
int tmp = 0;
//Relays object
const Relays* relay;
//MH SENSOR variables
int valueA;
bool valueD;
//Helpfull counter in switch for first light
byte internalSwitchCounter = 1;
//RGB 
int led_blue = 2;           
int led_red = 3;
int led_green = 4;
//Checker one initialize led light
bool initializeLed = true;
//Time reset initializer
unsigned long timer = 0;
//Led switcher
byte internalLedCounter = 1;
//RGB array
int rgb[3] = {0,0,0};
//Can change rgb value
bool canChangeRGB = false;
//For distance sensor RGB
byte currentLed;
//Bug if first is right move gessture
bool firstRight = true;
//Choose site
int LeftRight = 0;
//Begin choose
bool firstChos = true;
//startOnce 
bool startOnce = true;
//Light distanc
bool lightManipulation = false;
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LICZBADIOD, PIN, NEO_GRB + NEO_KHZ800);
bool alarmTemperature = false;
bool alarmTemperatureC = false;
bool alarmTemperatureH = false;
byte tempRGBread[3];

int ifAlarmWasActivatedTarrace = 100;

bool alarm__ = false;


void checkColorsRGB(){

       analogWrite(led_red,rgb[0]);
      analogWrite(led_green,rgb[1]);
      analogWrite(led_blue,rgb[2]);
      Serial.print(" R: ");
      Serial.print(rgb[0]);
      Serial.print(" G: ");
      Serial.print(rgb[1]);
      Serial.print(" B: ");
      Serial.println(rgb[2]);

}

void currentLight(int* acctualPin){
  int tmp = relay->RelayState(acctualPin);
  for(int i=0;i<2;i++){
    relay->OffRelays(acctualPin);
    delay(100);
    relay->OnRelays(acctualPin);
    delay(100);
  }
  if(tmp){
    relay->OffRelays(acctualPin);
  }else{
    relay->OnRelays(acctualPin);
  }
}
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

float temperatureArray[SENSORS_NUM];

void printTemperature(DeviceAddress deviceAddress,int num)
{
  float tempC = sensors.getTempC(deviceAddress);

  temperatureArray[num] = tempC;
  

}



void printData(DeviceAddress deviceAddress,int num)
{
  
 
  printTemperature(deviceAddress,num);

}
int pins[] = {22,23,24,25,lightTerrace};
byte tempPinsValues[(sizeof(pins)/sizeof(int))-2];

int runLights[sizeof(pins)/sizeof(int)];

bool mov = true;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
   Serial2.begin(9600);
  //Initialize distance sensor MH
  pinMode(A0,INPUT);
  pinMode(8,OUTPUT);
  pinMode(insineMoveLIght,OUTPUT);
  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(moveSensor,INPUT);
  pinMode(moveSensor2,INPUT);
  pinMode(moveLight,OUTPUT);
  pinMode(PinLightFloor0,OUTPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(PinLightFloor1,OUTPUT);
  pinMode(PinLightFloor2,OUTPUT);
  pinMode(PinLightFloor3,OUTPUT);
  pinMode(PinLightFloor_1,OUTPUT);
  pinMode(PinLightFloor_2,OUTPUT);
  digitalWrite(PinLightFloor1,LOW);
  digitalWrite(PinLightFloor2,LOW);
  digitalWrite(PinLightFloor0,LOW);
  digitalWrite(PinLightFloor3,LOW);
  digitalWrite(PinLightFloor_1,LOW);
  digitalWrite(PinLightFloor_2,LOW);
  pinMode(floorHeating1,OUTPUT);
  pinMode(floorHeating2,OUTPUT);
  pinMode(floorHeating3,OUTPUT);
  digitalWrite(floorHeating1,LOW);
  digitalWrite(floorHeating2,LOW);
  digitalWrite(floorHeating3,LOW);
  pinMode(moveSensor_1,INPUT);
  pinMode(moveSensor_2,INPUT);
  pinMode(moveSensorLR,INPUT);
  pinMode(analogMQ2,INPUT);
  pinMode(doorSensorTerrace,INPUT);
  pinMode(moveSensorTerrace,INPUT);
  pinMode(WindowSensorFloor2,INPUT_PULLUP);
  pinMode(WindowSensorFloor0,INPUT);
  pinMode(doorHalfFloor,INPUT_PULLUP);
  pinMode(alarms,OUTPUT);
  digitalWrite(alarms,HIGH);

  
 // pinMode(WindowSensorFloor0,HIGH);
  //pinMode(doorHalfFloor,LOW);
  //pinMode(WindowSensorFloor2,HIGH);
  

  //pinMode(doorHalfFloor,HIGH);
  
 // digitalWrite(doorHalfFloor,LOW);
  //digitalWrite(doorHalfFloor,LOW);UTPUT
  //digitalWrite(doorHalfFloor,LOW);
  //pinMode(doorHalfFloor,HIGH);
  //digitalWrite(doorHalfFloor,HIGH);
  //pinMode(WindowSensorFloor0,HIGH);
  //digitalWrite(doorHalfFloor,HIGH);
  

  
//for(int i=0;i<sizeof(pins3)/sizeof(int);i++){
//  pinMode(pins3[i],OUTPUT);
//}
  sensors.begin();
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  if (!sensors.getAddress(t1, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(t2, 1)) Serial.println("Unable to find address for Device 1");
 if (!sensors.getAddress(t3, 2)) Serial.println("Unable to find address for Device 2");
  if (!sensors.getAddress(t4, 3)) Serial.println("Unable to find address for Device 3");
  if (!sensors.getAddress(t5, 4)) Serial.println("Unable to find address for Device 4");
 
  #if termometrsOn
  while(!sensors.getAddress(t1, 0) || !sensors.getAddress(t2, 1) || !sensors.getAddress(t3, 2) || !sensors.getAddress(t4, 3) || !sensors.getAddress(t5, 4)  ){
    Serial.println("Finding Devices... ");      
  }
#endif
  
  sensors.setResolution(t1, TEMPERATURE_PRECISION);
  sensors.setResolution(t2, TEMPERATURE_PRECISION);

  sensors.setResolution(t3, TEMPERATURE_PRECISION);
  sensors.setResolution(t4, TEMPERATURE_PRECISION);
    sensors.setResolution(t5, TEMPERATURE_PRECISION);
  //Define object Relays
 
  int sizeArray = sizeof(pins)/sizeof(int);
  relay = new Relays(pins,sizeArray);
  relay->OffRelays();
  
  digitalWrite(insineMoveLIght,LOW);
  rgb[0] = 255;
  rgb[1] = 255;
  rgb[2] = 255;

  
//   pixels.begin();
  for(int i=0;i<10;i++){
    Serial.println(EEPROM.read(i));
  }


  // The first requests to all sensors for measurement

}
int tempRGB[3];
unsigned long del = 0;
int val;
//bool wasSaved = false;
long duration;
int distance;

byte dist80 = 0;
byte dist180 = 0;

unsigned long timerDist = 0;

bool activationLightFloor = false;

bool lightOnFloor = false;
unsigned long timLightFloor1 = 0;
unsigned long timLightFloor2 = 0;
unsigned long timeLightFloor2on = 0;
bool lightDelayoff = false;
unsigned long timeOffLight = 0;
unsigned long timeLightFloor0 = 0;
bool inLightMoveSensor = false;
bool moveSensor2Detected = false;
unsigned long timerFloor3 = 0;
bool activatedFloor0 = false;
unsigned long timeFloor2From3 = 0;
unsigned long floor3From2 = 0;
bool sawOnFloor2 = false;
bool lightDelayoff2 = false;
bool inLightMoveSensor2 = false;
unsigned long timeOffLight2 = 0;
bool from3to2 = false;
bool setLightLRled = false;

unsigned long readTemperatureTime = 0;

bool moveSensor_1Activated = false;

unsigned long moveSensor_1Time = 0;

bool moveSensor_2Activated = false;

unsigned long moveSensor_2Time = 0;

unsigned long lightLEDtime = 0;

unsigned long lightLRtime = 0;

unsigned long gasAlarmGreen200_300 = 0;

unsigned long gasAlarmGreen300 = 0;

bool canOffLedTimes = true;

bool greenLedAlarm200 = false;
bool greenLedAlarm300 = false;

bool readOnceGreen200 = true;
bool readOnceGreen300 = true;

bool readOnceMaxTemp = true;
bool readOnceMinTemp = true;

bool doorsClosed = false;
bool doorsOpen = false;

bool window2Closed = false;
bool windows2Open = false;

bool window0Closed = false;
bool windows0Open = false;

bool window1Closed = false;
bool windows1Open = false;


bool moveSensorTerraceActivated = false;
unsigned long moveSensorTerraceTime = 0;

bool sensorWasAcctivated = true;

unsigned long doorTwice = 0;
unsigned long window2Twice = 0;
unsigned long window0Twice = 0;


int firstBad0_1 = 0;
int firstBad0_2 = 0;

int firstBad1_1 = 0;
int firstBad1_2 = 0;

int firstBad2_1 = 0;
int firstBad2_2 = 0;

int firstBad3_1 = 0;
int firstBad3_2 = 0;

bool bebop1 = false;
bool bebop2 = false;

int firstBad4_1 = 0;
int firstBad4_2 = 0;

unsigned long slowbebop = 0;

int firstBad5_1 = 0;
int firstBad5_2 = 0;

unsigned long window1Twice = 0;

int firstBad6_0 = 0;
int firstBad7_0 = 0;
int firstBad8_0 = 0;
int firstBad9_0 = 0;
int firstBad10_0 = 0;

unsigned long timeToOutHouse = 0;

int NEARTWICE = 0;

bool AlarmSoundActivated = false;
bool AlarmFullyActivated = false;
bool beebOnceSound = false;

unsigned long moveSensorLRtime = 0;

bool moveSensorRMactivated = false;

bool TarraceSAlarm = false;

unsigned long TarraceSAlarmTime = 0;


bool InsideHouseSAlarm = false;

unsigned long InsideHouseSAlarmTime = 0;

unsigned long TimeToCloseWIndows = 0;

bool preActivated = false;

bool blinkOnce = false;

bool systemActive = false;



void AlarmSActive(){
if(preActivated){
  Serial.println("test");
  if(CheckWindowClosed()){
    AlarmSoundActivated = true;
    analogWrite(led_red,0);
    analogWrite(led_green,0);
    analogWrite(led_blue,0);
  
    AlarmSoundBebop(2);
  }else{
    if(!beebOnceSound){
      beebOnceSound = true;
      AlarmSoundBebop(4);
    }
  }
}
    
}


void AlarmSDeactive(){
  AlarmSoundBebop(3);
  AlarmSoundActivated = false;
  beebOnceSound = false;
  AlarmFullyActivated = false;
  InsideHouseSAlarm = false;
  preActivated = false;
  NEARTWICE = 0;
  digitalWrite(alarms,HIGH);
}


void AlarmSoundBebop(int count){
  for(int i=0;i<count;i++){
   
  digitalWrite(alarms,LOW);
  delay(100);
  digitalWrite(alarms,HIGH);
   delay(100);
  }
}

bool CheckWindowClosed(){

  bool Closed = true;

  if(!digitalRead(WindowSensorFloor0)){
    int pins1[] = {PinLightFloor0,insineMoveLIght,moveLight};
    blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
    Serial.println("WindowSensor0");
    Closed = false;
  }

  if(!digitalRead(WindowSensorFloor2)){
    int pins1[] = {PinLightFloor3,PinLightFloor2};
    blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
    Serial.println("WindowSensor2");
    Closed = false;
  }
  

  if(!digitalRead(doorSensorTerrace)){
    blinkLight(lightTerrace,2,100);
    Serial.println("doorSensroTerrace");
    Closed = false;
    
    
    
  }

  if(!digitalRead(doorHalfFloor)){
    int pins1[] = {PinLightFloor1,PinLightFloor2};
    blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
    Serial.println("doorHalfFloor");
    Closed = false;
    
  }

  if(!Closed){
    return Closed;
    
  }else{
    return true;
    
  }
  
  
}
bool lastSensor2 = false;
bool checkAllSensorOff(){


  
  if(moveSensor_2Activated){
    lastSensor2 = false;
    return true;
  }
  if(moveSensorRMactivated){
    lastSensor2 = false;
    return true;
    
  }
  if(moveSensor_1Activated){
    lastSensor2 = false;
    return true;
  }
  if(inLightMoveSensor2){
    lastSensor2 = true;
    return true;
  }
  if(inLightMoveSensor){
    lastSensor2 = false;
    return true;
  }


  return false;
  
  
}

unsigned long timeAlarmToStop = 0;

bool lastSensorOnFloor2 = false;
void CheckSensorInsideHouse(){

if(AlarmFullyActivated){
    

if(digitalRead(moveSensor)){
    InsideHouseSAlarm = true;
    //InsideHouseSAlarmTime = millis();
  lastSensorOnFloor2 = false;
    return;
    
    
  }
  if(digitalRead(moveSensor2)){
     InsideHouseSAlarm = true;
    //InsideHouseSAlarmTime = millis();
    lastSensorOnFloor2 = true;
    return;
  }
  if(digitalRead(moveSensor_1)){
     InsideHouseSAlarm = true;
    //InsideHouseSAlarmTime = millis();
    lastSensorOnFloor2 = false;
    return;
  }
  if(digitalRead(moveSensorLR)){
     InsideHouseSAlarm = true;
    //InsideHouseSAlarmTime = millis();
    lastSensorOnFloor2 = false;
    return;
  }
  if(digitalRead(moveSensor_2)){
     InsideHouseSAlarm = true;
   // InsideHouseSAlarmTime = millis();
   // digitalWrite(alarms,LOW);
   lastSensorOnFloor2 = false;
    return;
  }

 
  
}  
}



void saveTempStateLights(){
         rgbAlarm[0] = rgb[0];
         rgbAlarm[1] = rgb[1];
         rgbAlarm[2] = rgb[2];
  
}

bool rgbAlarmActivated(){
  for(int i=0;i<sizeof(rgb)/sizeof(int);i++){
    if(rgb[i] == 0){
      return true;
    }
  }
  
  for(int i=0;i<sizeof(rgbAlarm)/sizeof(int);i++){
    if(rgbAlarm[i] != 0){
      return true;
    }
  }
  return false;
}

int floorTemperaturePin(int pin){
  int searchedPin = -1;
  switch(pin){
    case 0:
    searchedPin = floorHeating2;
    break;
    case 2:
    searchedPin = floorHeating1;
    break;
    case 3:
    searchedPin = floorHeating3;
    break;
  }
  return searchedPin;
  
}

int counterRedFloor = 0;
bool redAlarm = false;
bool blueAlarm = false;
float temperatureArrayCopy[SENSORS_NUM];

byte oneNu = 0;
byte oneNu1 = 0;
bool no_change(){
  
 for(int i=0;i<SENSORS_NUM;i++){
  Serial.println(redAlarm);
  Serial.println(blueAlarm);
  if(redAlarm){
    if(temperatureArray[i]>=temperatureArrayCopy[i]){
      return false;
    }
  }
  if(blueAlarm){
    if(temperatureArray[i]<=temperatureArrayCopy[i]){
      return false;
    }
    
  }
  }
  return true;
}

void coppyArray(float* start,float* enda,int sizee){
  for(int i=0;i<sizee;i++){
    enda[i] = start[i];
  }
  
}
unsigned long alarmSaveTempArray = 0;

void tryFixFloor(int pin){
  
  if(tryFixFloorTimer+(timeAlarmDefault-1)<millis()){
   
    if(!no_change()){
    
     if(tryFixFloorBool){
     
        digitalWrite(floorHeating1,!digitalRead(floorHeating1));
        digitalWrite(floorHeating2,!digitalRead(floorHeating2));
        digitalWrite(floorHeating3,!digitalRead(floorHeating3));
        
     
     }
     digitalWrite(pin,!digitalRead(pin));
    tryFixFloorBool = !tryFixFloorBool;
   
    }
    
    counterRedFloor++;
  }
  //if(tryFixFloorTimer+20000>millis()){
      
    //    coppyArray(temperatureArray,temperatureArrayCopy,SENSORS_NUM);
       
       
  //}
    if(counterRedFloor%10 == 0){
  
    tryFixFloorTimer = millis();
     coppyArray(temperatureArray,temperatureArrayCopy,SENSORS_NUM);
    counterRedFloor++;
  
 
  }

  digitalWrite(floorHeating3,LOW);
  digitalWrite(floorHeating1,LOW); 
    digitalWrite(floorHeating2,LOW);
  
}

void readTempStateLight(){
  rgb[0] = rgbAlarm[0];
  rgb[1] = rgbAlarm[1];
  rgb[2] = rgbAlarm[2];
  analogWrite(led_red,rgb[0]);
  analogWrite(led_green,rgb[1]);
  analogWrite(led_blue,rgb[2]);

  
  
}
void blinkLight(int pin,int times,int del){
  byte state = digitalRead(pin);
  for(int i=0;i<times;i++){
    digitalWrite(pin,LOW);
    delay(del);
    digitalWrite(pin,HIGH);
    delay(del);
  }
  digitalWrite(pin,state);
}
void blinkLight(int pin[],int count,int times,int del){
  byte state[count];
  for(int j=0;j<count;j++){
      state[j] = digitalRead(pin[j]);
     
    }
  for(int i=0;i<times;i++){
    for(int j=0;j<count;j++){
      digitalWrite(pin[j],LOW);

      
    }
    delay(del);
    for(int j=0;j<count;j++){
      digitalWrite(pin[j],HIGH);
      
    }
    delay(del);
  }
    for(int j=0;j<count;j++){
      digitalWrite(pin[j],state[j]);
      
    }
 
}

void blinkLed(int colors[],int timeDelay,int countBlink){
  
  rgb[0] = colors[0];
  rgb[1] = colors[1];
  rgb[2] = colors[2];
  
  for(int i=0;i<countBlink;i++){
    analogWrite(led_red,rgb[0]);
    analogWrite(led_green,rgb[1]);
    analogWrite(led_blue,rgb[2]);
    delay(timeDelay);
    analogWrite(led_red,0);
    analogWrite(led_green,0);
    analogWrite(led_blue,0);
    delay(timeDelay);
    
  }
  
    analogWrite(led_red,rgb[0]);
    analogWrite(led_green,rgb[1]);
    analogWrite(led_blue,rgb[2]);
}

void lightRMChange(){
    lightLEDtime = millis();
  lightLRtime = millis();
  if(setLightLRled){
    analogWrite(led_red,rgb[0]);
    analogWrite(led_green,rgb[1]);
    analogWrite(led_blue,rgb[2]);
    for(int i=0;i<sizeof(pins)/sizeof(int);i++){
      digitalWrite(pins[i],runLights[i]);
    }
  }
  canOffOnLRlight = true;
  timeLightOff30 = false;
}


int countBad = 0;
void cahngeCanLed(){
    
  if(canOffOnLRlight){
   for(int i=0;i<sizeof(pins)/sizeof(int);i++){
    runLights[i] = digitalRead(pins[i]);
  }
    
}

}

int errorn = 0;

bool firstCheckTimes = true;

unsigned long timeTo100_1 = 0;
unsigned long timeTo100_2 = 0;

unsigned long timeTo100_3 = 0;
unsigned long timeTo100_4 = 0;

unsigned long timeTo100_5 = 0;
unsigned long timeTo100_6 = 0;

unsigned long timeTo100_7 = 0;
unsigned long timeTo100_8 = 0;

unsigned long timeTo100_9 = 0;

int checkSum = 50;
int checkSum1 = 50;

unsigned long lightCellTime = 0;

bool floor2Cell = false;
bool floor1Cell = false;
bool floorCellTarrace = false;

unsigned long timeToOffAlarm = 0;

String inbytes = "";

int checkCell(){

    int temp = digitalRead(lightTerrace);
    digitalWrite(lightTerrace,HIGH);
    delay(100);
    unsigned long tempLight = 0;
    
    for(int i=0;i<20;i++){
      int tempP = analogRead(A1);
      tempLight = tempLight+tempP;
    }
   // Serial.println("----SUMA---");
    digitalWrite(lightTerrace,temp);
    Serial.print("Pomiar: ");
   // 
    tempLight = tempLight/20;
   // tempLight = (inbytes.toInt());
    Serial.println(tempLight);
    if(tempLight>700){
      floor2Cell = false;
      floor1Cell = false;
      floorCellTarrace = false;

    }
    if(tempLight<500){
      floor2Cell = true;
      floor1Cell = true;
    }
    if(tempLight<400){
      floorCellTarrace = true;
      
    }
   
   
    
  return tempLight;
  
}

unsigned long minuteAfterLight = 0;

void setRGB(int r,int g,int b){
  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;
  rgbAlarm[0] = r;
  rgbAlarm[1] = g;
  rgbAlarm[2] = b;
  analogWrite(led_red,r);
  analogWrite(led_green,g);
  analogWrite(led_blue,b);
}

bool handChange = false;

bool handUpChanged = false;

bool setOnceLedStart = false;

bool onceCheck = false;

unsigned long timeVlaueCheck = 0;

unsigned long timeBurg = 0;


int valueCheckCell = 0;

unsigned long cellCheckTime = 0;

unsigned long cellCheckTimeFirst = 10000;

void loop() {

// Serial.print(floor2Cell);
// Serial.print(" | ");
// Serial.print(floor1Cell);
// Serial.print(" | ");
// Serial.println(floorCellTarrace);

  int lightCell = analogRead(A1);

  /* 
        if(Serial.available() > 0){
            inbytes = "";
          inbytes = Serial.readString();
          //Serial.println(inbytes);
      
        }
*/
if(cellCheckTime+cellCheckTimeFirst<millis()){
  cellCheckTimeFirst = 600000;
  cellCheckTime = millis();
  valueCheckCell = checkCell();
}
        

if(canOffOnLRlight){
   
  if(minuteAfterLight+10000<millis()){
   /*
      Serial.print(!handChange);
      Serial.print(" : ");
      Serial.print(!setOnceLedStart);
      Serial.print(" : ");
      Serial.println(handUpChanged); 
   */
      if(!handChange){

     if(!timeLightOff30 && canOffLedTimes){
        
      if(valueCheckCell>800){
        setRGB(0,0,0);
        
   
      }
      else if(valueCheckCell>700){
        setRGB(0,255,0);

      }
      else if(valueCheckCell>600){
        setRGB(0,0,255);

      }
      else if(valueCheckCell>500){
        setRGB(255,0,0);

      }
      else if(valueCheckCell>400){
        setRGB(255,0,255);

      }
      else if(valueCheckCell>300){
        setRGB(0,255,255);

      }
      else if(valueCheckCell>200){
        setRGB(255,255,0);
      }
      else if(valueCheckCell<200){
        setRGB(255,255,255);
      }


      minuteAfterLight = millis();
     }
      }
    
    
    }
}
  

  
  

  

//Serial.println(millis());


CheckSensorInsideHouse();

if(InsideHouseSAlarm && timeToOffAlarm+10000<millis()){
  if(firstCheckTimes){
    Serial.println(InsideHouseSAlarm);
  firstCheckTimes = false;
  timeToOffAlarm = millis();
  return;
  
  }
  InsideHouseSAlarmTime = millis();
   firstCheckTimes = true;
   InsideHouseSAlarm = false;
   alarm__ = true;
  
}

if(InsideHouseSAlarmTime+(20000-10)>millis() && alarm__ && AlarmFullyActivated &&  !lastSensorOnFloor2){
  digitalWrite(alarms,LOW);
  int pins1[] = {PinLightFloor1,PinLightFloor2,PinLightFloor0,insineMoveLIght,moveLight,PinLightFloor3,22,23,24,25,lightTerrace,PinLightFloor_1,PinLightFloor_2};
  blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
  
}else{
  digitalWrite(alarms,HIGH);
  alarm__ = false;
}


if(!AlarmFullyActivated && timeToOutHouse+10000<millis() && AlarmSoundActivated){
 
 if(checkAllSensorOff() == true){
 timeToOutHouse = millis();
 AlarmSoundBebop(1);
 }else{
  Serial.println("TESS");
 AlarmFullyActivated = true;
  timeToOffAlarm = millis();
 AlarmSoundBebop(2);
 }
}

 




cahngeCanLed();

  if(lightLEDtime+900000 < millis()){
    analogWrite(led_red,0);
    analogWrite(led_green,0);
    analogWrite(led_blue,0);
    canOffLedTimes = false;
    timeLightOff30 = true;
    
    
  }
  if(lightLRtime+1800000 < millis()){

    systemActive = false;

    handChange = false;
    
    for(int i=0;i<sizeof(pins)/sizeof(int);i++){
      digitalWrite(pins[i],HIGH);
    }
    if(moveSensorTerraceActivated){
      if(floorCellTarrace){
      digitalWrite(lightTerrace,LOW);
      }
    }
    canOffOnLRlight = false;
    timeLightOff30 = true;
    
  }

    //Window Sensor 1
    /*
String s = "";

if(!digitalRead(doorHalfFloor)){

  unsigned long timeFloorHalf = millis();

  while(timeFloorHalf+4000>millis()){
    
    s=s+String(digitalRead(doorHalfFloor));


  }
}

if(s != ""){
Serial.println(s);  
s = "";
}
*/
    
     if(!digitalRead(doorHalfFloor) && window1Twice+2000<millis() ){
      if(timeTo100_3+delayTimeError>millis()){
        firstBad5_1++;   
        
      }else{
        timeTo100_3 = millis();
        firstBad5_1 = 0;
      }
     
     if(firstBad5_1>=100 && timeTo100_3+delayTimeError>millis()){

      window1Closed = true;
      window1Twice = millis();
      firstBad5_1 = 0;
      return;
      
     }
  
   
    
  }else{
    firstBad5_1 = 0;
  }


  if(digitalRead(doorHalfFloor) && window1Twice+2000<millis()){
    if(timeTo100_4+delayTimeError>millis()){
    firstBad5_2++;
   
    }else{
      timeTo100_4 = millis();
      firstBad5_2 = 0;
    }
    if(firstBad5_2>=100 && timeTo100_4+delayTimeError>millis()){
    window1Twice = millis();
    firstBad5_2 = 0;
    return;
    }

  }else{
    firstBad5_2 = 0;
  }

  if(windows1Open){
     if(!digitalRead(doorHalfFloor)){
     Serial.println("Doors half floor CL");
     AlarmSoundBebop(1);
       int pins1[] = {PinLightFloor1,PinLightFloor2};
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
     windows1Open = false;
     window1Closed = true;
    //digitalWrite(doorHalfFloor,LOW);
     }
     
    
  }
  if(window1Closed){
    if(digitalRead(doorHalfFloor)){
 Serial.println("Doors half floor OP");
 AlarmSoundBebop(1);
      int pins1[] = {PinLightFloor1,PinLightFloor2};
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
      window1Closed = false;      
      windows1Open = true;
      //digitalWrite(doorHalfFloor,HIGH);
    }
  }
    


  
  //Window Sensor 0

  #if WINDOWSENSORFLOOR0  
  
  if(!digitalRead(WindowSensorFloor0) && window0Twice+2000<millis()){
        if(timeTo100_5+delayTimeError>millis()){
        firstBad1_1++;
        }else{
          timeTo100_5 = millis();
          firstBad1_1 = 0;
        }
        if(firstBad1_1>=100){
        window0Closed = true;
        window0Twice = millis();
        firstBad1_1 = 0;
        return; 
        
        }
  
  
  }else{
    firstBad1_1 = 0;
  }
  if(digitalRead(WindowSensorFloor0) && window0Twice+2000<millis()){
    if(timeTo100_6+delayTimeError>millis()){
    firstBad1_2++;
    }else{
      timeTo100_6 = millis();
      firstBad1_2 = 0;
    }
    if(firstBad1_2>=100){
    windows0Open = true;
    window0Twice = millis();
    firstBad1_2=0;
    return;
    }
 
  }else{
    firstBad1_2 = 0;
  }
  
  #endif
   if(windows0Open){

     if(!digitalRead(WindowSensorFloor0)){
        Serial.println("Doors floor 0");
        AlarmSoundBebop(1);
     int pins1[] = {PinLightFloor0,insineMoveLIght,moveLight};
    
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
     windows0Open = false;
     }
      
   }
 

  if(window0Closed){
      if(digitalRead(WindowSensorFloor0)){
        Serial.println("Doors floor 0");
        AlarmSoundBebop(1);
      int pins1[] = {PinLightFloor0,insineMoveLIght,moveLight};
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
      window0Closed = false;      

      }
    
  }
  
  
  //Window Sensor 2
  /*
  if(digitalRead(WindowSensorFloor2)){
    Serial.println(firstBad0_1);
    firstBad0_1=0;
        
  }else{
    firstBad0_1++;
  }
  */

  #if WINDOWSENSORFLOOR2
 
  
  if(!digitalRead(WindowSensorFloor2) && window2Twice+2000<millis() ){
    if(timeTo100_1+delayTimeError>millis()){
     firstBad2_1++;
    // Serial.println(firstBad2_1);
    }else{
      timeTo100_1 = millis();
      firstBad2_1 = 0;
    }
    
     if(firstBad2_1>=100){
      //Serial.print("Closed: ");
      //Serial.println(millis());
      window2Closed = true;
      window2Twice = millis();
      firstBad2_1 = 0;
      return;
      
     }
  
   
    
  }else{
    firstBad2_1 = 0;
  }
  if(digitalRead(WindowSensorFloor2) && window2Twice+2000<millis()){
    if(timeTo100_2+delayTimeError>millis()){
      firstBad2_2++;
      //Serial.println(firstBad2_2);
    }else{
      timeTo100_2 = millis();
      firstBad2_2 = 0;
    }
    if(firstBad2_2>=100){
      //Serial.print("Open: ");
      //Serial.println(millis());
    windows2Open = true;
    window2Twice = millis();
    firstBad2_2 = 0;
    return;
    }

  }else{
    firstBad2_2 = 0;
  }
  #endif
  
   if(windows2Open){
     if(!digitalRead(WindowSensorFloor2)){
        Serial.println("Doors floor 2 OP");
       
     AlarmSoundBebop(1);
       int pins1[] = {PinLightFloor3,PinLightFloor2};
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
     windows2Open = false;
   
     }
    
  }
  
  if(window2Closed){
    if(digitalRead(WindowSensorFloor2)){
      Serial.println("Doors floor 2 CL");
      
      AlarmSoundBebop(1);
      int pins1[] = {PinLightFloor3,PinLightFloor2};
      blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
    window2Closed = false;
    }
  }
  
  
  /*
Serial.println("-----------------------");
Serial.print("Przekaźnik: "+String(digitalRead(lightTerrace)));
Serial.print("Czujnik ruchu: "+String(digitalRead(moveSensorTerrace)));
Serial.println("Czujnik magnetyczny: "+String(digitalRead(doorSensorTerrace)));  
Serial.println("-----------------------");
  */
  //Terrace
 
  
  if(!digitalRead(doorSensorTerrace)  && doorTwice+2000<millis()){

     if(timeTo100_7+delayTimeError>millis()){
      firstBad3_2++;
     }else{
      timeTo100_7 = millis();
      firstBad3_2 = 0;
     }
     
 
      if(firstBad3_2>=100){
        //digitalReadAll();
      doorsOpen=true;
      doorTwice=millis();
       firstBad3_2 = 0;
      return;         
      }

      
  }else{
  firstBad3_2 = 0;
  }
  
  if(doorsOpen){
    if(digitalRead(doorSensorTerrace)){
    
          Serial.println("Doors floor tarrace OP");
          
          
        AlarmSoundBebop(1);
       
          if(!sensorWasAcctivated){
           blinkLight(lightTerrace,2,100);
           delay(300);
           blinkLight(lightTerrace,1,100);
          }else{
             blinkLight(lightTerrace,2,100);
          }
          sensorWasAcctivated = true;
      doorsOpen = false;

    }
  }
  if(digitalRead(doorSensorTerrace)  && doorTwice+2000<millis()){
  
      if(timeTo100_8+delayTimeError>millis()){
      firstBad3_1++;
      }else{
        timeTo100_8 = millis();
        firstBad3_1 = 0;
      }
      if(firstBad3_1>=100){
        //digitalReadAll();
      doorsClosed=true;
      doorTwice=millis();
      firstBad3_1 = 0;
      return ;            
      }

    
    
  }else{
    firstBad3_1 = 0;
  }
  
  if(doorsClosed){
    if(!digitalRead(doorSensorTerrace)){
      
      Serial.println("Doors floor tarrace CL");
      
   AlarmSoundBebop(1);
   
     
       if(!sensorWasAcctivated){
           blinkLight(lightTerrace,2,100);
            delay(300);
            blinkLight(lightTerrace,1,100);
          }else{
            blinkLight(lightTerrace,2,100);
          }
          sensorWasAcctivated = true;
      
      doorsClosed = false;
   
      
    }
  }
if(moveSensorTerraceActivated && !digitalRead(moveSensorTerrace)){
  if(moveSensorTerraceTime+30000<millis()){

    digitalWrite(lightTerrace,HIGH);
 
    moveSensorTerraceActivated = false;
  }
}

if(moveSensor_1Activated && !digitalRead(moveSensor_1)){
 if(moveSensor_1Time+30000<millis()){
  digitalWrite(PinLightFloor_1,LOW);
  digitalWrite(PinLightFloor0,LOW);
  digitalWrite(PinLightFloor_2,LOW);
  moveSensor_1Activated = false;
 }
}
if(moveSensor_2Activated && !digitalRead(moveSensor_2)){
  if(moveSensor_2Time+40000<millis()){
    digitalWrite(PinLightFloor_1,LOW);
    digitalWrite(PinLightFloor_2,LOW);
    moveSensor_2Activated = false;
  }
}


  
//Serial.println(distance);
#if moveSensorActivated
if(lightDelayoff){
  if(!digitalRead(moveSensor) && !inLightMoveSensor){
    if(floor2Cell){
    digitalWrite(moveLight,HIGH);
    }
    inLightMoveSensor = true;
    timeOffLight = millis();
   // Serial.println("END 1");
  }
  if(inLightMoveSensor && timeOffLight+10000<millis()){
    
        lightDelayoff = false;
        inLightMoveSensor = false;
        digitalWrite(moveLight,LOW);
        digitalWrite(PinLightFloor1,LOW);
        activationLightFloor = false;
     //   Serial.println("END");
  }
}
#endif
#if moveSensorActivated2
if(lightDelayoff2){
  if(!digitalRead(moveSensor2) && !inLightMoveSensor2){
    inLightMoveSensor2 = true;
    timeOffLight2 = millis();
  }
  if(inLightMoveSensor2 && timeOffLight2+30000<millis()){
    lightDelayoff2 = false;
    inLightMoveSensor2 = false;
    digitalWrite(PinLightFloor2,LOW);
    digitalWrite(PinLightFloor3,LOW);
    floor3From2 = false;
    lightOnFloor = false;
    from3to2 = false;
  }
}
#endif
#if moveSensorActivated2

#endif


  #if distanceOn
 
  if(from3to2 && digitalRead(moveLight)){
    if(floor2Cell){
    digitalWrite(PinLightFloor1,HIGH);
    }else{
      //blinkLight(PinLightFloor1,1,100);
      
    }
  }
  if(lightDelayoff2){
   if(floor2Cell){
      digitalWrite(PinLightFloor2,HIGH);
    digitalWrite(PinLightFloor3,HIGH);
   }
  }
  
  if(timeLightFloor0+5000<millis() && activatedFloor0 && !moveSensor_1Activated){
    digitalWrite(PinLightFloor0,LOW);
    digitalWrite(PinLightFloor_1,LOW);
    timeLightFloor0 = millis();
    activatedFloor0 = false;
  }
  
  
// Serial.println(digitalRead(30));
//Serial.print("Activation light floor: ");
//Serial.println(activationLightFloor);
  
  
  if(timerDist+80 <= millis()){
      digitalWrite(trigPin,LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin,HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin,LOW);
      duration = pulseIn(echoPin,HIGH);
      distance = (duration/2)/29.1;
      timerDist = millis();
  
  
  if(distance>10 && distance<80){
    //22,23,24
    if(activationLightFloor){
      if(floor1Cell){
        digitalWrite(PinLightFloor0,HIGH);
      }
        timeLightFloor0 = millis();
        activatedFloor0 = true;
      //  activationLightFloor = 3;
       digitalWrite(PinLightFloor_1,HIGH);
       lightRMChange();
       canOffLedTimes = true;
    
      }
    
     

    return;
    
    }
  
  
  if(distance>120 && distance<200){
    
    dist180++;
    /*
        Serial.print("Swiatlo2: ");
    Serial.print(dist180);
    Serial.print(" ");
      Serial.println(distance);*/
   // if(dist180 >= 3){
      if(activationLightFloor){
        if(floor2Cell){
        digitalWrite(PinLightFloor1,HIGH);
        }else{
         // blinkLight(PinLightFloor1,1,100);
        }
       lightRMChange();
       canOffLedTimes = true;

        lightOnFloor = true;
        
        // activationLightFloor--;
      }
   //}
     
   return;
    //}
  
  }else{
    //dist180 = 0;
   //  distance = 0;
    
  }
  }
  #endif
  
  //Read value for MH distance sensor
 // valueA = analogRead(A0)/3.137;
 // valueD = digitalRead(8);
//  Serial.println(valueA);
/*if(currentLed == 1){
analogWrite(led_red,rgb[0]);  
analogWrite(led_green,0);
analogWrite(led_blue,0);

}*/
  
/*if(lightManipulation && valueA<=255 && ){
  
}*/



//Read AX/TX data from another arduino
 while(Serial2.available()){

  value = Serial2.read();
  sRec= sRec+(char)value;
 
  
 }
 //Read AX/TX data from Nodemcu
 while(Serial1.available()){
  value1 = Serial1.read();
  sRec1 = sRec1+(char)value1;
 }
 //Clean variable some mistake 
 if(sRec1.length()>6){
  sRec1 = "";
 }
if(sRec.length()>6){
  sRec="";
}
//Serial.println(sRec1);
if(timer+60000<millis()){
  initializeLed = true;
  counter = 0;
  timer = millis();
   canChangeRGB = false;
   sRec = "";
   sRec1 = "";
   

}

 
if(sRec == "UP" || sRec1 == "UP"){
  //Clean variables
  cahngeCanLed();

  
  firstRight = false;
  setLightLRled = true;

         
      
  
 
  
    sRec = "";
    sRec1 = "";
    
    if(preActivated){
      AlarmSDeactive();
      return;
      
    }
  
    
    counter++;
   timer = millis();
    if(counter > relay->CountRelays()){
      counter = 1;
       firstChos = true;
      canChangeRGB = false;
    }
    if(!relay->m_CheckOnRelays() && mov){ //!relay->m_CheckOnRelays() && !digitalRead(5) && mov
 
      counter = 0;
      if(initializeLed)
      { 
         minuteAfterLight = millis();
         
        analogWrite(led_red,0);
        analogWrite(led_green,0);
        analogWrite(led_blue,0);
        delay(30);
 for (int i=0; i <= 255; i++){
      analogWrite(led_blue, i);
      delay(2);
 }
delay(50);
for (int i=0; i <= 255; i++){
      analogWrite(led_blue, 255-i);
      analogWrite(led_red, i);
      delay(2);
 }
 delay(200);
for (int i=0; i <= 255; i++){
      analogWrite(led_green, i);
      analogWrite(led_blue, i);
      delay(2);
 }
 
  
initializeLed = false;
delay(1000);
     analogWrite(led_green, rgb[1]);
      analogWrite(led_blue, rgb[2]);
      analogWrite(led_red,rgb[0]);

}
  if(!handUpChanged){
    
    if(valueCheckCell>800){
      relay->OnRelays(24);
    }
    else if(valueCheckCell>500){
      relay->OnRelays(23);
       digitalWrite(insineMoveLIght,HIGH);
       
    }
    else if(valueCheckCell>300){
      relay->OnRelays(22);
       relay->OnRelays(24);
        relay->OnRelays(25);
    }
    else if(valueCheckCell<300){
      relay->OnRelays();
      digitalWrite(insineMoveLIght,HIGH);
    }
    handUpChanged = true;
  }
  
  
  
  relay->OffRelays(lightTerrace);
 // digitalWrite(5,HIGH);

  if(startOnce){

  startOnce = false;
  }
   if(!rgb[0] && !rgb[1] && !rgb[2]){
       rgb[0] = tempRGB[0];
       rgb[1] = tempRGB[1];
       rgb[2] = tempRGB[2]; 
    if(!tempRGB[0] && !tempRGB[1] && !tempRGB[2]){
      rgb[0] = 255;
      rgb[1] = 255;
      rgb[2] = 255;
      
    }
      
      }
      checkColorsRGB();
      return;
    }
    //Here color checker
    mov = false;
    checkColorsRGB();
     if(counter == relay->CountRelays()){
      lightManipulation = false;
      for(int i=0;i<2;i++){
          analogWrite(led_green, 0);
      analogWrite(led_blue, 0);
      analogWrite(led_red, 0);
      delay(100);
       analogWrite(led_green, 255);
      analogWrite(led_blue, 255);
      analogWrite(led_red, 255);
      delay(100);
      }
      checkColorsRGB();
      
      return;
    
    }
    
   
    
    if(counter == relay->CountRelays()-2){
     // int tmp1 = digitalRead(5);
      
       for(int i =0;i<2;i++){
        digitalWrite(insineMoveLIght,LOW);
        delay(100);
        digitalWrite(insineMoveLIght,HIGH);
        delay(100);
       }
  //     Serial.println(tmp1);
   //    digitalWrite(5,tmp1);
       return;
    }
    if(counter == relay->CountRelays()-1){
      int tmp = relay->RelayState(lightTerrace);
    for(int i=0;i<2;i++){
      relay->OffRelays(lightTerrace);
      delay(100);
      relay->OnRelays(lightTerrace);
      delay(100);
    }
    digitalWrite(lightTerrace,tmp);
    }
 
    if(counter == 1){
       int tmp1 = relay->RelayState(relay->RelayPinPosition(counter));
      
      int tmp2 = relay->RelayState(relay->RelayPinPosition(counter+1));
  
      int tmp3 =  relay->RelayState(relay->RelayPinPosition(counter+2));
      
       for(int i=0;i<2;i++){
        for(int j=0;j<3;j++){
          digitalWrite(relay->RelayPinPosition(counter+j),HIGH);
        }
        delay(100);
         for(int j=0;j<3;j++){
          digitalWrite(relay->RelayPinPosition(counter+j),LOW);
        }
        delay(100);
       }
       digitalWrite(relay->RelayPinPosition(counter),tmp1);
       digitalWrite(relay->RelayPinPosition(counter+1),tmp2);
       digitalWrite(relay->RelayPinPosition(counter+2),tmp3);
    }
    else{
        relay->RelayState(relay->RelayPinPosition(counter+2));
        currentLight(relay->RelayPinPosition(counter+2));
    }

        
cahngeCanLed();


/*
    if(!handUpChanged){
      if(checkCell()>800){
        digitalWrite(22,HIGH);
        digitalWrite(23,LOW);
        digitalWrite(24,LOW);
        digitalWrite(25,LOW);
        digitalWrite(lightTerrace,LOW);
        return;
      }
      if(checkCell()<300){
        return;
      }
      if(checkCell()>500){
        digitalWrite(23,HIGH);
        //digitalWrite
      }
      handUpChanged = true;
    }
*/


   
}
if(sRec == "LEFT" || sRec1 == "LEFT"){
    sRec = "";
    sRec1 = "";
    if(preActivated){
      AlarmSDeactive();
      return;
      
    }
  
    
    timer = millis();
    if(firstRight){
      digitalWrite(insineMoveLIght,LOW);
      return;
    }
   
    if(counter == relay->CountRelays()-2){
      //EEPROM.write(relay->CountRelays()-2,0);
      digitalWrite(insineMoveLIght,LOW);
      lightManipulation = false;
      //EEPROM.write(counter+1,0);
      return;
    }
    if(counter == relay->CountRelays()-1){
   //   EEPROM.write(relay->CountRelays()-1,0);
      relay->OffRelays(lightTerrace);
     // lightManipulation = false;
      EEPROM.write(counter+1,digitalRead(lightTerrace));
      return;
    }
    
    if(counter == 1){
         int tmp = relay->RelayState(relay->RelayPinPosition(internalSwitchCounter-1));
    if(tmp){
      relay->OnRelays(relay->RelayPinPosition(internalSwitchCounter-1));
    }else{
      relay->OffRelays(relay->RelayPinPosition(internalSwitchCounter-1));
    }
    EEPROM.write(internalSwitchCounter-1,digitalRead(relay->RelayPinPosition(internalSwitchCounter-1)));
    }else{
       relay->OffRelays(relay->RelayPinPosition(counter+2));
       EEPROM.write(counter+1,digitalRead(relay->RelayPinPosition(counter+2)));
    }


}
if(sRec == "DOWN" || sRec1 == "DOWN"){
    sRec = "";
    sRec1 = "";
    handUpChanged = false;
  

    if(preActivated){
      AlarmSDeactive();
      return;
      
    }
  
    
   if(moveSensorTerraceActivated){

      digitalWrite(lightTerrace,HIGH);
    
      sensorWasAcctivated = false;
      moveSensorTerraceActivated = false;
      return;
    }
    
    if(!relay->m_CheckOnRelays()){
      firstRight = true;
    }
   
    relay->OffRelays();
   
    digitalWrite(insineMoveLIght,LOW);
//digitalWrite(5,LOW);
    
  //  digitalWrite(5,LOW);
    mov = true;
    counter = 0;
    timer = millis();

   
    

}
if(sRec == "RIGHT" || sRec1 == "RIGHT"){
    sRec = "";
    sRec1 = "";
    if(preActivated){
      AlarmSDeactive();
      return;
      
    }
  
    
    if(firstRight){
      digitalWrite(insineMoveLIght,HIGH);
      return;
    }
     if(counter == relay->CountRelays()-1){
      relay->OnRelays(lightTerrace);
       //
       
      // lightManipulation = true;
      //EEPROM.write(counter+1,digitalRead(lightTerrace));
      return;
    }
    timer = millis();
    
      if(counter == relay->CountRelays()-2){
        lightManipulation = true;
      digitalWrite(insineMoveLIght,HIGH);
    //  EEPROM.write(counter+1,1);
      return;
    }
       if(firstChos){
      LeftRight = 2;
      firstChos = false;
    }
    
    if(counter == relay->CountRelays() && LeftRight == 2){
      if(!rgb[0] && !rgb[1] && !rgb[2]){
       rgb[0] = tempRGB[0];
       rgb[1] = tempRGB[1];
       rgb[2] = tempRGB[2]; 
      }
      if(internalLedCounter>8){
         internalLedCounter = 1;
      }
      canChangeRGB = true;
      
      switch(internalLedCounter){
        case 1:
        handChange = true;
        currentLed = 1;
        rgb[0] = 255;
        rgb[1] = 0;
        rgb[2] = 0;
        saveTempStateLights();
          if(!rgb[0]){
        analogWrite(led_red,255);
        }else{
          analogWrite(led_red,rgb[0]);
        }
        analogWrite(led_blue,0);
        analogWrite(led_green,0);
        for(int i=0;i<2;i++){
        analogWrite(led_red,255);  
        delay(100);
        analogWrite(led_red,0);
        delay(100);
        }
            if(!rgb[0]){
        analogWrite(led_red,255);
        }else{
          analogWrite(led_red,rgb[0]);
        }
   //  Serial.println("CZerwony");
       break;
       case 2:
       handChange = true;
       currentLed = 2;
         rgb[1] = 255;
             rgb[0] = 0;
        rgb[2] = 0;
        saveTempStateLights();
          if(!rgb[1]){
        analogWrite(led_green,255);
        }else{
          analogWrite(led_green,rgb[1]);
        }
          analogWrite(led_red,0);
        analogWrite(led_blue,0);
        for(int i=0;i<2;i++){
        analogWrite(led_green,255);  
        delay(100);
        analogWrite(led_green,0);
        delay(100);
        }
               if(!rgb[1]){
        analogWrite(led_green,255);
        }else{
          analogWrite(led_green,rgb[1]);
        }
     
     //     Serial.println("Zielony");
       break;
       case 3:
       handChange = true;
       currentLed = 3;
         rgb[2] = 255;
             rgb[1] = 0;
        rgb[0] = 0;
        saveTempStateLights();
        
         if(!rgb[2]){
        analogWrite(led_blue,255);
        }else{
          analogWrite(led_blue,rgb[2]);
        }
             analogWrite(led_red,0);
        analogWrite(led_green,0);
        for(int i=0;i<2;i++){
        analogWrite(led_blue,255);  
        delay(100);
        analogWrite(led_blue,0);
        delay(100);
        }
           if(!rgb[2]){
        analogWrite(led_blue,255);
        }else{
          analogWrite(led_blue,rgb[2]);
        }
       // Serial.println("Niebieski");
       break;
       case 4:
       handChange = true;
       currentLed = 4;
         rgb[2] = 255;
             rgb[1] = 255;
        rgb[0] = 0;
        saveTempStateLights();
         if(!rgb[2] && !rgb[1]){
        analogWrite(led_blue,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_blue,rgb[2]);
          analogWrite(led_green,rgb[1]);
        }
             analogWrite(led_red,0);
        
        for(int i=0;i<2;i++){
        analogWrite(led_blue,255); 
        analogWrite(led_green,255); 
        delay(100);
        analogWrite(led_blue,0);
        analogWrite(led_green,0);
        delay(100);
        }
         if(!rgb[2] && !rgb[1]){
        analogWrite(led_blue,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_blue,rgb[2]);
          analogWrite(led_green,rgb[1]);
        }
       // Serial.println("Niebieski");
       break;
        case 5:
        handChange = true;
       currentLed = 5;
         rgb[2] = 0;
             rgb[1] = 255;
        rgb[0] = 255;
        saveTempStateLights();
         if(!rgb[0] && !rgb[1]){
        analogWrite(led_red,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_green,rgb[1]);
        }
             analogWrite(led_blue,0);
        
        for(int i=0;i<2;i++){
        analogWrite(led_red,255); 
        analogWrite(led_green,255); 
        delay(100);
        analogWrite(led_red,0);
        analogWrite(led_green,0);
        delay(100);
        }
         if(!rgb[0] && !rgb[1]){
        analogWrite(led_red,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_green,rgb[1]);
        }
       // Serial.println("Niebieski");
       break;
       case 6:
       handChange = true;
       currentLed = 6;
              rgb[2] = 255;
              rgb[1] = 0;
              rgb[0] = 255;
              saveTempStateLights();
         if(!rgb[0] && !rgb[2]){
        analogWrite(led_red,255);
        analogWrite(led_blue,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_blue,rgb[2]);
        }
             analogWrite(led_green,0);
        
        for(int i=0;i<2;i++){
        analogWrite(led_red,255); 
        analogWrite(led_blue,255); 
        delay(100);
        analogWrite(led_red,0);
        analogWrite(led_blue,0);
        delay(100);
        }
         if(!rgb[0] && !rgb[2]){
        analogWrite(led_red,255);
        analogWrite(led_blue,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_blue,rgb[2]);
        }
       // Serial.println("Niebieski");
       break;
            case 7:
            handChange = true;
       currentLed = 7;
              rgb[2] = 255;
              rgb[1] = 255;
              rgb[0] = 255;
              saveTempStateLights();
              
         if(!rgb[0] && !rgb[2] && !rgb[1]){
        analogWrite(led_red,255);
        analogWrite(led_blue,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_blue,rgb[2]);
          analogWrite(led_green,rgb[1]);
        }
           
        
        for(int i=0;i<2;i++){
        analogWrite(led_red,255); 
        analogWrite(led_blue,255); 
        analogWrite(led_green,255); 
        delay(100);
        analogWrite(led_red,0);
        analogWrite(led_blue,0);
         analogWrite(led_green,0); 
        delay(100);
        }
            if(!rgb[0] && !rgb[2] && !rgb[1]){
        analogWrite(led_red,255);
        analogWrite(led_blue,255);
        analogWrite(led_green,255);
        }else{
          analogWrite(led_red,rgb[0]);
          analogWrite(led_blue,rgb[2]);
          analogWrite(led_green,rgb[1]);
        }
       // Serial.println("Niebieski");
       break;
          case 8:
          handChange = true;
       currentLed = 8;
              rgb[2] = 0;
              rgb[1] = 0;
              rgb[0] = 0;
              saveTempStateLights();
          analogWrite(led_red,rgb[0]);
        analogWrite(led_green,rgb[1]);
        analogWrite(led_blue,rgb[2]);
       

        
       // Serial.println("Niebieski");
       break;
      }
   
       internalLedCounter++;    
      return;
    }
    if(counter == 1){
         if(internalSwitchCounter>3){
          internalSwitchCounter = 1;
        }
        currentLight(relay->RelayPinPosition(internalSwitchCounter));
        internalSwitchCounter++;
    }else{
      relay->OnRelays(relay->RelayPinPosition(counter+2));
      EEPROM.write(counter+1,digitalRead(relay->RelayPinPosition(counter+2)));
    }
   
}

if(NEARTWICE == 2 && TimeToCloseWIndows+2000 < millis() && !AlarmSoundActivated && preActivated){
  TimeToCloseWIndows = millis();
  AlarmSActive();
}

if(sRec == "NEAR"){

  sRec = "";

   if(NEARTWICE < 2 && !AlarmSoundActivated){
    NEARTWICE++;
    AlarmSoundBebop(1);
    if(NEARTWICE == 2){
      preActivated = true;
    }
  }

  
      if(counter == relay->CountRelays()){
  

      
        tempRGB[0] = rgb[0];
        tempRGB[1] = rgb[1];
        tempRGB[2] = rgb[2];
        rgb[0]=0;
        rgb[1]=0;
        rgb[2]=0;
        saveTempStateLights();
      analogWrite(led_red,rgb[0]);
      analogWrite(led_green,rgb[1]);
      analogWrite(led_blue,rgb[2]);

    }
}
if(sRec == "FAR"){
  
  sRec = "";

}
/*if(digitalRead(9)){
  if(activationLightFloor == 2){
        return;
       }
       activationLightFloor--;
}*/
#if moveSensorActivated
if(digitalRead(moveSensor)){
        firstBad7_0++;
        if(firstBad7_0>=5){
          if(floor2Cell){
       digitalWrite(moveLight,HIGH);     
          }
       
  //     lightDetected = true;
       timeOffLight = millis();
       lightDelayoff = true;
   
       activationLightFloor = true;
    //   Serial.println("Sensor");
     //  Serial.println(digitalRead(moveLight));
        firstBad7_0 = 0;
        }
     }else{
      firstBad7_0 = 0;
     }
#endif
//Serial.println(digitalRead(9));
#if moveSensorActivated2
if(digitalRead(moveSensor2)){
  firstBad6_0++;
  if(firstBad6_0>=5){
  moveSensor2Detected = true;
  lightDelayoff2 = true;
  from3to2 = true;
  firstBad6_0 = 0;
  }
}else{
  firstBad6_0 = 0;
  moveSensor2Detected = false;
}

#endif
//Serial.println(internalLedCounter);
#if MOVESENSOR_1
if(digitalRead(moveSensor_1)){
  firstBad0_1++;
  if(firstBad0_1>=5){
  //Serial.println("Move Sensor 1 activated");
  moveSensor_1Activated = true;
  moveSensor_1Time = millis();
  if(floor1Cell){
    digitalWrite(PinLightFloor_2,HIGH);
  }
  digitalWrite(PinLightFloor_1,HIGH);
  if(floor1Cell){
  digitalWrite(PinLightFloor0,HIGH);
  }
 firstBad0_1=0;
  }
}else{
  firstBad0_1 = 0;
}
#endif

#if MOVESENSOR_2
if(digitalRead(moveSensor_2)){
  firstBad0_2++;
  if(firstBad0_2>=5){
  //  Serial.println("Move Sensor 2 activated");
  moveSensor_2Activated = true;
  moveSensor_2Time = millis();
  if(floor1Cell){
  digitalWrite(PinLightFloor_2,HIGH);
  }
  digitalWrite(PinLightFloor_1,HIGH);
  
 
  
 
  
  firstBad0_2 = 0;
  
  }
}else{
  firstBad0_2 = 0;
}

#endif

if(digitalRead(moveSensorTerrace) && sensorWasAcctivated){
  if(timeTo100_9+delayTimeError>millis()){
     firstBad4_1++; 
    }else{
       firstBad4_1 = 0;
       timeTo100_9 = millis();
    }
 
  
  
  if(firstBad4_1>=100){
  moveSensorTerraceActivated = true;
  moveSensorTerraceTime = millis();
  if(floorCellTarrace){
    digitalWrite(lightTerrace,LOW);
  }

    firstBad4_1=0;
  }

  
}else{
  firstBad4_1 = 0;
}

if(digitalRead(moveSensorLR)){
  firstBad10_0++;
  if(firstBad10_0>=5){
lightRMChange();
canOffLedTimes = true;
moveSensorRMactivated = true;
moveSensorLRtime = millis();

  firstBad10_0 = 0;
  }
}else{
  firstBad10_0 = 0;
  
}

if(moveSensorLRtime+60000<millis() && !digitalRead(moveSensorLR)){
  moveSensorRMactivated = false;
}

int analogMQ2Read = analogRead(analogMQ2);
//Serial.println(analogMQ2Read);

if(gasAlarmGreen300+5000<millis()){
if(analogMQ2Read>300){
  if(readOnceMinTemp && readOnceMaxTemp && readOnceGreen200 && readOnceGreen300){
      saveTempStateLights();
      readOnceGreen300 = false;
  }
  
  int colorsToBlink[] = {0,255,0};
  //TODO OTHER LAMP
  int pins1[] = {PinLightFloor1,PinLightFloor2,PinLightFloor0,insineMoveLIght,moveLight,PinLightFloor3,lightTerrace,PinLightFloor_1,PinLightFloor_2};
  blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
  blinkLed(colorsToBlink,100,2);
  gasAlarmGreen300 = millis();
  greenLedAlarm300 = true;
  
    
    
  
}else{
  greenLedAlarm300 = false; 
  readOnceGreen300 = true;
}
}


if(gasAlarmGreen200_300+20000<millis()){
if(analogMQ2Read>=210 && analogMQ2Read<300){
  if(readOnceMinTemp && readOnceMaxTemp && readOnceGreen200 && readOnceGreen300){
    saveTempStateLights();
    readOnceGreen200 = false;
  }
  int colorsToBlink[] = {0,255,0};
  //TODO OTHER LAMP
  int pins1[] = {PinLightFloor1,PinLightFloor2,PinLightFloor0,insineMoveLIght,moveLight,PinLightFloor3,lightTerrace,PinLightFloor_1,PinLightFloor_2};
  blinkLight(pins1,sizeof(pins1)/sizeof(int),2,100);
  blinkLed(colorsToBlink,50,1);
  gasAlarmGreen200_300 = millis();
  greenLedAlarm200 = true;
 
    

  
}else{
  if(rgbAlarmActivated() && !canChangeRGB && readOnceMinTemp && readOnceMaxTemp && canOffLedTimes){
     readTempStateLight();
  //  Serial.println("test1");
  
  }
  
  greenLedAlarm200 = false;
  readOnceGreen200 = true;
  
}
}

#if termometrsOn

  if(readTemperatureTime+DefaultUsageTimeAlarm<millis() || alarmTemperature){  
  sensors.requestTemperatures();

  printData(t1,0);
  delay(100);
  printData(t2,1);
   delay(100);
  printData(t3,2);
   delay(100);
  printData(t4,3);
   delay(100);
   printData(t5,4);
   
  Serial.println("--------------------");
  //dev4 temperatureArray[3] floorHeating3
  //dev2 temperatureArray[1] floorHeating3
  //dev3 floorHeating1 
  //dev1 floorHeating2
  
  Serial.print("dev1: ");
  Serial.println(temperatureArray[0]);
  Serial.print("dev2: ");
  Serial.println(temperatureArray[1]);
  Serial.print("dev3: ");
  Serial.println(temperatureArray[2]);
  Serial.print("dev4: ");
  Serial.println(temperatureArray[3]);
Serial.print("dev5: ");
Serial.println(temperatureArray[4]);
  
  if(temperatureArray[3] == -127.00 || temperatureArray[1] == -127.00){
    return;
  }
/*  if(temperatureArray[1]>=22.50){
    digitalWrite(floorHeating2,HIGH);
  }
  if(temperatureArray[1]<= 22.00){
     digitalWrite(floorHeating2,LOW);
  }*/
  if(temperatureArray[3] == -127.00){
    return;
  }
   if(temperatureArray[2] == -127.00){
    return;
  }
   if(temperatureArray[0] == -127.00){
    return;
  }
  if(temperatureArray[1] == -127.00){
    return;
  }

if(temperatureArray[4] == -127.00){
  return;
}

if(temperatureArray[4]<MAX_TEMPERATURE){
  
  if(temperatureArray[3]>=temperatureFloor){
   digitalWrite(floorHeating3,LOW);
   
  }
  if(temperatureArray[3]<=temperatureFloorMin){
   digitalWrite(floorHeating3,HIGH);
   
  }
  if(temperatureArray[2]>=temperatureFloor){
   digitalWrite(floorHeating1,LOW); 
  }
  if(temperatureArray[2]<=temperatureFloorMin){
   digitalWrite(floorHeating1,HIGH); 
  }
  if(temperatureArray[0]>=temperatureFloor){
    digitalWrite(floorHeating2,LOW);
  }
  if(temperatureArray[0]<=temperatureFloorMin){
    digitalWrite(floorHeating2,HIGH);
  }
  
}else{
     if(temperatureArray[3]>=temperatureFloor){
   digitalWrite(floorHeating3,LOW);
   
  }
    if(temperatureArray[2]>=temperatureFloor){
   digitalWrite(floorHeating1,LOW); 
  }
    if(temperatureArray[0]>=temperatureFloor){
    digitalWrite(floorHeating2,LOW);
  }
          
}
  
  
for(int i=0;i<sizeof(temperatureArray)/sizeof(float);i++){
    if(temperatureArray[i]>(MAX_TEMPERATURE)){ // i == 2 nieaktwny
       DefaultUsageTimeAlarm = timeAlarmRed;
       
       if(readOnceMinTemp && readOnceMaxTemp && readOnceGreen200 && readOnceGreen300){
         saveTempStateLights();
        readOnceMaxTemp = false;
        coppyArray(temperatureArray,temperatureArrayCopy,SENSORS_NUM);
       redAlarm = true; 
       }
       Serial.println(temperatureArray[0]);
       Serial.println(temperatureArrayCopy[0]);
   
         int colorsToBlink[] = {255,0,0};
         blinkLed(colorsToBlink,50,1);
                 //    tryFixFloor(floorTemperaturePin(i));
          
        // tryFixFloor(floorTemperaturePin(i)); //Code for repaier relays
        
       
         
          if(temperatureArray[3]>=temperatureFloor){
   digitalWrite(floorHeating3,LOW);
   
  }

  if(temperatureArray[2]>=temperatureFloor){
   digitalWrite(floorHeating1,LOW); 
  }

  if(temperatureArray[0]>=temperatureFloor){
    digitalWrite(floorHeating2,LOW);
  }

       
       return;
    }
    else if(temperatureArray[i]<(MIN_TEMPERATURE) && i != 1 && i != 4){ //i == 1 nieaktywny i == 2 nieaktwyny
       DefaultUsageTimeAlarm = timeAlarmBlue;
       if(readOnceMinTemp && readOnceMaxTemp && readOnceGreen200 && readOnceGreen300){
        saveTempStateLights();
        readOnceMinTemp = false;
        
        coppyArray(temperatureArray,temperatureArrayCopy,SENSORS_NUM);
        
       blueAlarm = true; 
       }
       
       
     
         int colorsToBlink[] = {0,0,255};
         blinkLed(colorsToBlink,50,1);
         
         tryFixFloor(floorTemperaturePin(i));
         
 
  if(temperatureArray[3]<=temperatureFloorMin){
   digitalWrite(floorHeating3,HIGH);
   
  }

  if(temperatureArray[2]<=temperatureFloorMin){
   digitalWrite(floorHeating1,HIGH); 
  }

  if(temperatureArray[0]<=temperatureFloorMin){
    digitalWrite(floorHeating2,HIGH);
  }
         
       return;
    }else if(sizeof(temperatureArray)/sizeof(float)-1 == i){
      if(rgbAlarmActivated() && !canChangeRGB){
         readTempStateLight();
       
      }
      readOnceMaxTemp = true;
      readOnceMinTemp = true;
      blueAlarm = false;
      redAlarm = false;
      DefaultUsageTimeAlarm = timeAlarmDefault;
    }
   
   
  }


  
  readTemperatureTime = millis();
  }
  #endif







}
