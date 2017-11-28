/* If your program is called foobar.cpp then build with:
 * g++ -Wall -pthread -o foobar foobar.cpp -lpigpio -lrt
 */

#include <iostream>
#include <csignal>
#include <pigpio.h>

#include "kinematics.h"
#include "rotary_encoder.hpp"

double teste;
int M1a, M1b, M2a, M2b, M3a, M3b;
int E1a, E1b, E2a, E2b, E3a, E3b;

static int pos = 0;
static int pos_old;
static int t_pos, t_pos_old;
double rps, rps_0=0, rps_1=0, rps_2=0, rps_3=0, rps_4=0, rps_avg;

// ticks para medir velocidade do encoder
uint32_t startTick, endTick;
int diffTick = 0;

// para interagir com user, set point da velocidade
int inp;

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
  rps = 2932/micros;
  return rps;
}


void dec_callback(int way)
{
  t_pos = gpioTick();
  pos += way;

  rps = 2932/double(t_pos-t_pos_old);

  // média móvel dos ultimos 5 valores
  rps_4=rps_3;
  rps_3=rps_2;
  rps_2=rps_1;
  rps_1=rps_0;
  rps_0 = rps;
  rps_avg = (rps_0+rps_1+rps_2+rps_3+rps_4)/5.0;

  std::cout << "rps=" << rps_avg << std::endl;
  pos_old = pos;
  t_pos_old = t_pos;

}

void loop (void)
{
  erro = inp - rps_avg; // 3000 para testes, erro positivo

  val_new = val_old + 0.02*erro; //erro negativo diminui o valor de acionamento
  if(val_new > 0)
  {
    gpioPWM(M1a, 0);
    gpioPWM(M1b, int(val_new));
  }
  if(val_new < 0)
  {
    gpioPWM(M1a, int(val_new));
    gpioPWM(M1b, 0);
  }

  val_old=val_new;
}

int main(void)
{
  signal(SIGINT, exit_from_key);
  gpioInitialise();


  // setando pinos para os testes
  M1a = 2;
  M1b = 3;
  E1a = 26;
  E1b = 19;

  // setando os pinos de saída:
  gpioSetMode(M1a, PI_OUTPUT);
  gpioSetMode(M1b, PI_OUTPUT);
  gpioSetMode(E1a, PI_INPUT);
  gpioSetMode(E2a, PI_INPUT);

  // chama função loop() a cada 2ms
  gpioSetTimerFunc(3, 10, loop);

  re_decoder dec(E1a, E1b, dec_callback);

  inp = 0;

  for(;;)
  {
    cin >> inp;
  }

  return 0;
}

/*

startTick = gpioTick();

// do some processing

endTick = gpioTick();

diffTick = endTick - startTick;

printf("some processing took %d microseconds", diffTick);
*/
