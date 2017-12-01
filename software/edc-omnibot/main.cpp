/* If your program is called foobar.cpp then build with:
 * g++ -Wall -pthread kinematics.o rotary_encoder.o rpi_interface.o -o foobar main.cpp -lpigpio -lrt
 */

#include <iostream>
#include <csignal>
#include <pigpio.h>

#include "kinematics.h"
#include "rotary_encoder.hpp"
#include "rpi_interface.h"


#define L 0.125 // distance between body center and wheel center
#define r 0.029 // wheel radius

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

  // atualiza os valores de velocidade, em rad/s
  Motor1.rps[4]=Motor1.rps[3];
  Motor1.rps[3]=Motor1.rps[2];
  Motor1.rps[2]=Motor1.rps[1];
  Motor1.rps[1]=Motor1.rps[0];
  Motor1.rps[0] = 2000*way/double(Motor1.t_pos-Motor1.t_pos_old);

  //getAngSpd??
  Motor1.pos_old = Motor1.pos;
  Motor1.t_pos_old = Motor1.t_pos;
}

void dec_callback2(int way)
{
  Motor2.t_pos = gpioTick();
  Motor2.pos += way; // para odometria

  // atualiza os valores de velocidade, em rad/s
  Motor2.rps[4]=Motor2.rps[3];
  Motor2.rps[3]=Motor2.rps[2];
  Motor2.rps[2]=Motor2.rps[1];
  Motor2.rps[1]=Motor2.rps[0];
  Motor2.rps[0] = 2000*way/double(Motor2.t_pos-Motor2.t_pos_old);
  //getAngSpd??
  Motor2.pos_old = Motor2.pos;
  Motor2.t_pos_old = Motor2.t_pos;
}

void dec_callback3(int way)
{
  Motor3.t_pos = gpioTick();
  Motor3.pos += way; // para odometria

  // atualiza os valores de velocidade, em rad/s
  Motor3.rps[4]=Motor3.rps[3];
  Motor3.rps[3]=Motor3.rps[2];
  Motor3.rps[2]=Motor3.rps[1];
  Motor3.rps[1]=Motor3.rps[0];
  Motor3.rps[0] = 18410*way/double(Motor3.t_pos-Motor3.t_pos_old);

  //getAngSpd??
  Motor3.pos_old = Motor3.pos;
  Motor3.t_pos_old = Motor3.t_pos;
}

static int tempo, tempo_old, t_diff;

void odometry()
{/*
  tempo_old = tempo;
  tempo = gpioTick();
  t_diff = tempo - tempo_old;
  Vleft = Motor1.getAngSpd()*r;
  Vback = Motor2.getAngSpd()*r;
  Vright= Motor3.getAngSpd()*r;

  forwardKinematicsWorld(); // atualiza Vxw, Vyw e omegap

  xw += Vxw*t_diff;
  yw += Vyw*t_diff;
  theta += omegap*t_diff; // ERRADO*/
}

void loop (void)
{
  Motor1.setSetpoint(Vleft);
  Motor2.setSetpoint(Vback);
  Motor3.setSetpoint(Vright);
  //odometry();
}


int main(void)
{
  signal(SIGINT, exit_from_key);
  gpioInitialise();

  Motor1.stop();
  Motor2.stop();
  Motor3.stop();

  Motor1.start();
  Motor2.start();
  Motor3.start();

  re_decoder dec1(E1a, E1b, dec_callback1);
  re_decoder dec2(E2a, E2b, dec_callback2);
  re_decoder dec3(E3a, E3b, dec_callback3);


  cout << "\nVxw: ";
  cin >> Vxw;
  cout << "Vyw: ";
  cin >> Vyw;
  cout << "omegap: ";
  cin >> omegap;

  inverseKinematicsWorld();


  cout << "\n\nVelocidade da roda 1: " << Vleft;
  cout << "\nVelocidade da roda 2: " << Vback;
  cout << "\nVelocidade da roda 3: " << Vright;
  // chama função loop() a cada 10ms
  gpioSetTimerFunc(3, 10, loop);

  int stop = 1;
  cout << "\nPresione ZERO para parar:\n";
  while (stop!=0)
  {
    cin >> stop;
  }

  // cancela o loop:
  gpioSetTimerFunc(3, 10, NULL);

  odometry();

  // para motores
  Motor1.setSetpoint(0);
  Motor2.setSetpoint(0);
  Motor3.setSetpoint(0);

  odometry();

  cout << "\nDistância em X: " << xw << "\n";
  cout << "Distância em Y: " << yw << "\n";
  cout << "Rotação em rad: " << theta << "\n";

  return 0;
}
