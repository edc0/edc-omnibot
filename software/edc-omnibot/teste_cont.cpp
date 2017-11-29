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

// roda 1:
static int pos1 = 0;
static int pos1_old;
static int t_pos1, t_pos1_old;
double rps1, rps1_0=0, rps1_1=0, rps1_2=0, rps1_3=0, rps1_4=0, rps1_avg;

// roda 2:
static int pos2 = 0;
static int pos2_old;
static int t_pos2, t_pos2_old;
double rps2, rps2_0=0, rps2_1=0, rps2_2=0, rps2_3=0, rps2_4=0, rps2_avg;

//roda 3:
static int pos3 = 0;
static int pos3_old;
static int t_pos3, t_pos3_old;
double rps3, rps3_0=0, rps3_1=0, rps3_2=0, rps3_3=0, rps3_4=0, rps3_avg;

// para interagir com user, set point da velocidade
double inp;

// valores para atualizar o controlador 1
double val1_old=0;
double val1_new=0;
double erro1;


// valores para atualizar o controlador 2
double val2_old=0;
double val2_new=0;
double erro2;

// valores para atualizar o controlador 3
double val3_old=0;
double val3_new=0;
double erro3;

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

void dec_callback1(int way)
{
  t_pos1 = gpioTick();
  pos1 += way; // para odometria

  rps1 = way*2932/double(t_pos1-t_pos1_old);

  // média móvel dos ultimos 5 valores
  rps1_4=rps1_3;
  rps1_3=rps1_2;
  rps1_2=rps1_1;
  rps1_1=rps1_0;
  rps1_0 = rps1;
  rps1_avg = (rps1_0+rps1_1+rps1_2+rps1_3+rps1_4)/5.0;

  //std::cout << "rps1=" << rps1_avg << std::endl;
  pos1_old = pos1;
  t_pos1_old = t_pos1;

}

void dec_callback2(int way)
{
  t_pos2 = gpioTick();
  pos2 += way; // para odometria

  rps2 = way*2932/double(t_pos2-t_pos2_old);

  // média móvel dos ultimos 5 valores
  rps2_4=rps2_3;
  rps2_3=rps2_2;
  rps2_2=rps2_1;
  rps2_1=rps2_0;
  rps2_0 = rps2;
  rps2_avg = (rps2_0+rps2_1+rps2_2+rps2_3+rps2_4)/5.0;

  //std::cout << "rps2=" << rps2_avg << std::endl;
  pos2_old = pos2;
  t_pos2_old = t_pos2;

}

void dec_callback3(int way)
{
  t_pos3 = gpioTick();
  pos3 += way; // para odometria

  rps3 = way*2932/double(t_pos3-t_pos3_old);

  // média móvel dos ultimos 5 valores
  rps3_4=rps3_3;
  rps3_3=rps3_2;
  rps3_2=rps3_1;
  rps3_1=rps3_0;
  rps3_0 = rps3;
  rps3_avg = (rps3_0+rps3_1+rps3_2+rps3_3+rps3_4)/5.0;

  //std::cout << "rps=" << rps_avg << std::endl;
  pos3_old = pos3;
  t_pos3_old = t_pos3;

}

void loop (void)
{
  erro1 = inp - rps1_avg; // 3000 para testes, erro positivo
  erro2 = inp - rps2_avg;
  erro3 = inp - rps3_avg;

  val1_new = val1_old + erro1; //erro negativo diminui o valor de acionamento
  val2_new = val2_old + erro2;
  val3_new = val3_old + erro3;

  //cout << val_new <<"\n"<<erro<<"\n\n" ;
  if(val1_new > 0)
  {
    gpioPWM(M1a, 0);
    gpioPWM(M1b, int(val1_new));
  }
  if(val1_new < 0)
  {
    gpioPWM(M1a, -int(val1_new));
    gpioPWM(M1b, 0);
  }
  if(val2_new < 0)
  {
    gpioPWM(M2a, 0);
    gpioPWM(M2b, -int(val2_new));
  }
  if(val2_new > 0)
  {
    gpioPWM(M2a, int(val2_new));
    gpioPWM(M2b, 0);
  }
  if(val3_new > 0)
  {
    gpioPWM(M3a, 0);
    gpioPWM(M3b, 0);//int(val3_new));
  }
  if(val3_new < 0)
  {
    gpioPWM(M3a, 0);//-int(val3_new));
    gpioPWM(M3b, 0);
  }

  val1_old = val1_new;
  val2_old = val2_new;
  val3_old = val3_new;
}
int main(void)
{
  signal(SIGINT, exit_from_key);
  gpioInitialise();


  // setando pinos para os testes
  M1a = 2;  M1b = 3;
  M2a = 17; M2b = 27;
  M3a = 16; M3b = 20;

  E1a = 26; E1b = 19;
  E2a = 25; E2b = 8;
  E3a = 23; E3b = 24;

  // setando os pinos de saída:
  gpioSetMode(M1a, PI_OUTPUT);
  gpioSetMode(M1b, PI_OUTPUT);
  gpioSetPWMfrequency(M1a, 10000);
  gpioSetPWMfrequency(M1b, 10000);

  gpioSetMode(M2a, PI_OUTPUT);
  gpioSetMode(M2b, PI_OUTPUT);
  gpioSetPWMfrequency(M2a, 10000);
  gpioSetPWMfrequency(M2b, 10000);

  gpioSetMode(M3a, PI_OUTPUT);
  gpioSetMode(M3b, PI_OUTPUT);
  gpioSetPWMfrequency(M3a, 10000);
  gpioSetPWMfrequency(M3b, 10000);

  gpioSetMode(E1a, PI_INPUT);
  gpioSetMode(E2a, PI_INPUT);

  // chama função loop() a cada 10ms
  gpioSetTimerFunc(3, 10, loop);

  re_decoder dec1(E1a, E1b, dec_callback1);
  re_decoder dec2(E2a, E2b, dec_callback2);
  re_decoder dec3(E3a, E3b, dec_callback3);

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
