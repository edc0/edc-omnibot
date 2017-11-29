/* If your program is called foobar.cpp then build with:
 * g++ -Wall -pthread -o foobar foobar.cpp -lpigpio -lrt
 */

#include <iostream>
#include <csignal>
#include <pigpio.h>

#include "kinematics.h"
// #include "rotary_encoder.hpp"
#include "rpi_interface.h"


// para interagir com user, set point da velocidade
double inp;

using namespace std;

/* para encerrar com ctrl+c */
// TODO: timeout
void exit_from_key (int signum)
{
  cout << "Interrupt signal (" << signum << ") received.\n" ;
  gpioWrite(M1a, 0);
  gpioWrite(M1b, 0);
  gpioWrite(M2a, 0);
  gpioWrite(M2b, 0);
  gpioWrite(M3a, 0);
  gpioWrite(M3b, 0);
  gpioDelay(10000);
  gpioTerminate(); // desliga motores
  exit(signum);
}

void loop (void)
{
  Motor1.setSetpoint(inp);
  Motor2.setSetpoint(-inp);
}

int main(void)
{
  signal(SIGINT, exit_from_key);
  gpioInitialise();

  // setando pinos para os testes
  int M1a = 2,  M1b = 3,
  M2a = 17, M2b = 27,
  M3a = 16, M3b = 20;

  int E1a = 26, E1b = 19,
  E2a = 25, E2b = 8,
  E3a = 23, E3b = 24;

  OmniRPiInterface Motor1(M1a, M1b, E1a, E1b);
  OmniRPiInterface Motor2(M2a, M2b, E2a, E2b);
  OmniRPiInterface Motor3(M3a, M3b, E3a, E3b);

  // chama função loop() a cada 10ms
  gpioSetTimerFunc(3, 10, loop);

  inp = 0;

  for(;;)
  {
    cin >> inp;
  }

  return 0;
}
