#include <wiringPi.h>

int main (void)
{
  wiringPiSetup();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(15, INPUT);
  pinMode(7, INPUT);

  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  
  for(;;)
  {
    digitalWrite(4, HIGH);
    delay(1000);
    digitalWrite(4, LOW);
    delay(500);
    digitalWrite(5, HIGH);
    delay(1000);
  }
}
