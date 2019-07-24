#include "Alarm.h"

Alarm::Alarm(const int* alarmPins,size_t count){
  _alarmPins = alarmPins;
  _count = count;
  
  
}

void Alarm::ActiveAlarm(){

  _alarmWasActivated = true;
  BebopAlarm(2);
  
  
}

void Alarm::BebopAlarm(int count){


  
}

