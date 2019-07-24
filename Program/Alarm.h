#include "Arduino.h"

class Alarm{
  private:
    const int* _alarmPins;
    bool _alarmWasActivated;
    size_t _count;
    
    
  
  public:
    Alarm(const int* alarmPins,size_t count);
    void ActiveAlarm();
    

  private:
    void BebopAlarm(int count);
    

  
};

