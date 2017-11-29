// Did not put declare variables as extern here because this interface is to be
// used exclusively with the <kinematics> library.

#include <pigpio.h>
#include "rotary_encoder.hpp"
#include "rpi_interface.h"

OmniRPiInterface::OmniRPiInterface(int MA, int MB, int EA, int EB)
{
  Mot_A = MA;
  Mot_B = MB;

  Enc_A = EA;
  Enc_B = EB;

  gpioSetMode(Mot_A, PI_OUTPUT);
  gpioSetMode(Mot_B, PI_OUTPUT);
  gpioSetPWMfrequency(Mot_A, 10000);
  gpioSetPWMfrequency(Mot_B, 10000);

  re_decoder dec(Enc_A, Enc_B, dec_callback);
}

void OmniRPiInterface::dec_callback(int way)
{
  t_pos = gpioTick();
  pos += way; // para odometria

  // atualiza os valores de velocidade
  rps[4]=rps[3];
  rps[3]=rps[2];
  rps[2]=rps[1];
  rps[1]=rps[0];
  rps[0] = way*2932/double(t_pos-t_pos_old);

  //getAngSpd??
  pos_old = pos;
  t_pos_old = t_pos;
}

void OmniRPiInterface::resetPos()            // reinicia os contadores de posição
{
  pos = 0;
  pos_old = 0;
}

double OmniRPiInterface::getWheelPos()       // returns pos (mas se é public, pra q?)
{
  return(pos);
}

void OmniRPiInterface::setSetpoint(double sp)// define velocidade desejada
{
  spd_error = sp - getAngSpd();
  control_new = control_old + Kp*spd_error;   //q q eu faço com os integral e derivativo?

  if(control_new > 0)
  {
    gpioPWM(Mot_A, 0);
    gpioPWM(Mot_B, int(control_new));
  }
  if(control_new < 0)
  {
    gpioPWM(Mot_A, -int(control_new));
    gpioPWM(Mot_B, 0);
  }

  control_old = control_new;
}

double OmniRPiInterface::getspd_error(double sp) // returns current controller error signal
{
  spd_error = sp - getAngSpd();
  return(spd_error);
}

double OmniRPiInterface::getAngSpd(int n) // returns current wheel speed (average of n last readings)
{
  double rps_avg=0;
  for(int i = 0; i<n; i++)
  {
    rps_avg = rps_avg + rps[i];
  }
  rps_avg = rps_avg/double(n);
}
