#include <wiringPi.h>   // para controlar GPIO
#include <softPwm.h>
#include <csignal>      // para tratar o ctrl+C da saída do programa
#include <iostream>

using namespace std;

void exit_from_key (int signum)
{
  cout << "Interrupt signal (" << signum << ") received.\n" ;

  softPwmWrite(4, 0);
  softPwmWrite(5, 0);
  delay(100);

  exit(signum);
}

int main (void)
{
  signal(SIGINT, exit_from_key);

  wiringPiSetup();
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  softPwmCreate(4,0,100);
  softPwmCreate(5,0,100);


  for(;;)
  {
    softPwmWrite(4,40);
    /*delay(1000);
    softPwmWrite(4,100);
    delay(1000);
    softPwmWrite(4,20);
    delay(500);
    softPwmWrite(4,0);
    delay(500);
    softPwmWrite(5,30);
    delay(1000);
    softPwmWrite(5,0);
    delay(500);*/
  }
}
