#include "Relays.h"
Relays::Relays(const int* matrix,int columns){
  m_Matrix = matrix;
  m_Columns = columns;
  m_InitializePins();
}
bool Relays::RelayState(int pin) const{
  return digitalRead(pin);
}
void Relays::OnRelays() const{
  for(int i=0;i<m_Columns;i++){
    digitalWrite(m_Matrix[i],LOW);
  }
}
int Relays::CountRelays() const{
  return m_Columns;
}
void Relays::m_InitializePins() const{
  for(int i=0;i<m_Columns;i++){
    pinMode(m_Matrix[i],OUTPUT);
  }
}
int Relays::RelayPinPosition(int positionPin) const{
    return m_Matrix[positionPin-1];
  
}
void Relays::OnRelays(int pin) const{
  digitalWrite(pin,LOW);
}
void Relays::OffRelays(int pin) const{
  digitalWrite(pin,HIGH);
}
void Relays::OffRelays() const{
    for(int i=0;i<m_Columns;i++){
    digitalWrite(m_Matrix[i],HIGH);
  }
}
void Relays::OnRelays(const int* turnPins,int sizeArray) const{
 for(int i=0;i<sizeArray;i++){
  digitalWrite(turnPins[i],LOW);
 }
}
void Relays::OffRelays(const int* turnPins,int sizeArray) const{
 for(int i=0;i<sizeArray;i++){
  digitalWrite(turnPins[i],HIGH);
 }
}
bool Relays::m_CheckOnRelays() const{
  for(int i=0;i<m_Columns;i++){
    if(!digitalRead(m_Matrix[i])){
      return true;
    }
  }
  return false;
}
