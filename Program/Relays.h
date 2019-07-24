#include "Arduino.h"
class Relays{
private:
//Count of relay pins
 int m_Columns;
 //Array with relay pins
 const int* m_Matrix;
public:
//Constructor get array pins usage in relays and count of array
Relays(const int* matrix,int columns);
//Turn on arguments pins multi pins as array
void OnRelays(const int* turnPins,int sizeArray) const;
//Turn off relays
void OnRelays() const;
//Turn on one pin relay
void OnRelays(int pin) const;
//Turn off arguments pins multi pins as array
void OffRelays(const int* turnPins,int sizeArray) const;
//Turn off relays
void OffRelays() const;
//Turn off one pin relay
void OffRelays(int pin) const;
//Get size usage relay
int CountRelays() const;
//Return true if some relays is turn on
bool m_CheckOnRelays() const;
bool RelayState(int pin) const; 
int RelayPinPosition(int positionPin) const;
private:
//Initialize usage pins
void m_InitializePins() const;
};
