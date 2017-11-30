/* If your program is called foobar.cpp then build with:
 * g++ -Wall -pthread kinematics.o rotary_encoder.o rpi_interface.o -o foobar main.cpp -lpigpio -lrt
 */

#include <iostream>
#include <csignal>
#include <pigpio.h>

#include "kinematics.h"
#include "rotary_encoder.hpp"
#include "rpi_interface.h"


// para interagir com user, set point da velocidade
double inp;

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

using namespace std;

/* para encerrar com ctrl+c */
// TODO: timeout
void exit_from_key (int signum)
{
  cout << "Interrupt signal (" << signum << ") received.\n" ;
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  gpioDelay(10000);
  gpioTerminate(); // desliga motores
  exit(signum);
}

void dec_callback1(int way)
{
  Motor1.t_pos = gpioTick();
  Motor1.pos += way; // para odometria

  // atualiza os valores de velocidade
  Motor1.rps[4]=Motor1.rps[3];
  Motor1.rps[3]=Motor1.rps[2];
  Motor1.rps[2]=Motor1.rps[1];
  Motor1.rps[1]=Motor1.rps[0];
  Motor1.rps[0] = way*2932/double(Motor1.t_pos-Motor1.t_pos_old);

  //getAngSpd??
  Motor1.pos_old = Motor1.pos;
  Motor1.t_pos_old = Motor1.t_pos;
}

void dec_callback2(int way)
{
  cout << "callbackenc2\n\n";
  Motor2.t_pos = gpioTick();
  Motor2.pos += way; // para odometria

  // atualiza os valores de velocidade
  Motor2.rps[4]=Motor2.rps[3];
  Motor2.rps[3]=Motor2.rps[2];
  Motor2.rps[2]=Motor2.rps[1];
  Motor2.rps[1]=Motor2.rps[0];
  Motor2.rps[0] = way*2932/double(Motor2.t_pos-Motor2.t_pos_old);

  //getAngSpd??
  Motor2.pos_old = Motor2.pos;
  Motor2.t_pos_old = Motor2.t_pos;
}

void dec_callback3(int way)
{
  Motor3.t_pos = gpioTick();
  Motor3.pos += way; // para odometria

  // atualiza os valores de velocidade
  Motor3.rps[4]=Motor3.rps[3];
  Motor3.rps[3]=Motor3.rps[2];
  Motor3.rps[2]=Motor3.rps[1];
  Motor3.rps[1]=Motor3.rps[0];
  Motor3.rps[0] = way*2932/double(Motor3.t_pos-Motor3.t_pos_old);

  //getAngSpd??
  Motor3.pos_old = Motor3.pos;
  Motor3.t_pos_old = Motor3.t_pos;
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

  Motor1.start();
  Motor2.start();
  Motor3.start();

  re_decoder dec1(E1a, E1b, dec_callback1);
  re_decoder dec2(E2a, E2b, dec_callback2);
  re_decoder dec3(E3a, E3b, dec_callback3);

  // chama função loop() a cada 10ms
  gpioSetTimerFunc(3, 10, loop);

  inp = 0;
  for(;;)
  {
    cin >> inp;
    cout << Motor2.set_point << "\n";
  }

  return 0;
}
