/* If your program is called foobar.cpp then build with:
 * g++ -Wall -pthread -o foobar foobar.cpp -lpigpio -lrt
 */

#include <iostream>
#include <csignal>
#include <pigpio.h>

#include "kinematics.h"

double teste;
int M1a, M1b, M2a, M2b, M3a, M3b;
int E1a, E1b, E2a, E2b, E3a, E3b;
int SP = 128; // rotações por segundo

// ticks para medir velocidade do encoder
uint32_t startTick, endTick;
int diffTick;

// valores para atualizar o controlador
double val_old=0;
double val_new=0;
double erro;


using namespace std;

/* para encerrar com ctrl+c */
// TODO: timeout
void exit_from_key (int signum)
{
  cout << "Interrupt signal (" << signum << ") received.\n" ;
  gpioTerminate(); // desliga motores
  gpioDelay(10000);
  exit(signum);
}

int calcVel(int micros)
{
  int rps;
  rps = 2932/micros;
  return rps;
}

void loop (void)
{
  erro = SP - calcVel(3000); // 3000 para testes, erro positivo


  if(SP > 0)
  {
    gpioPWM(M1a, 0);
    gpioPWM(M1b, SP);
  }
  if(SP <= 0)
  {
    gpioPWM(M1a, SP);
    gpioPWM(M2a, 0);
  }
}

int main(void)
{
  signal(SIGINT, exit_from_key);
  gpioInitialise();

  // setando pinos para os testes
  M1a = 2;
  M2a = 3;
  E1a = 26;
  E1b = 19;

  // setando os pinos de saída:
  gpioSetMode(M1a, PI_OUTPUT);
  gpioSetMode(M2a, PI_OUTPUT);
  gpioSetMode(E1a, PI_INPUT);
  gpioSetMode(E2a, PI_INPUT);

  // chama função loop() a cada 2ms
  gpioSetTimerFunc(3, 2, loop);

  for(;;)
  {}

  return 0;
}

/*

startTick = gpioTick();

// do some processing

endTick = gpioTick();

diffTick = endTick - startTick;

printf("some processing took %d microseconds", diffTick);
*/
