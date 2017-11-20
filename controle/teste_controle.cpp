#include <wiringPi.h>   // para controlar GPIO
#include <csignal>      // para tratar o ctrl+C da saída do programa
#include <iostream>

using namespace std;

void exit_from_key (int signum)
{
  cout << "Interrupt signal (" << signum << ") received.\n" ;

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  exit(signum);
}

int main (void)
{
  signal(SIGINT, exit_from_key);

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
    digitalWrite(5, LOW);
    delay(500);
  }
}
