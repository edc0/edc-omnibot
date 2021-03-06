// Did not put declare variables as extern here because this interface is to be
// used exclusively with the <kinematics> library.

#include <pigpio.h>
#include <iostream>
#include "rotary_encoder.hpp"
#include "rpi_interface.h"

OmniRPiInterface::OmniRPiInterface(int MA, int MB, int EA, int EB)
{
  Mot_A = MA;
  Mot_B = MB;

  Enc_A = EA;
  Enc_B = EB;

  Kp = 15;
  zm = 115;
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
  spd_error = sp - getWhlSpd();
  control = control_old + Kp*spd_error;   //q q eu faço com os integral e derivativo?

  if(control+zm>255)
    control = 255-zm;
  if(control-zm<-255)
    control = -255+zm;

  if(control > 0)
  {
    gpioPWM(Mot_A, 0);
    gpioPWM(Mot_B, int(control+zm));
  }
  if(control < 0)
  {
    gpioPWM(Mot_A, -int(control-zm));
    gpioPWM(Mot_B, 0);
  }

  control_old = control;
}

double OmniRPiInterface::getError(double sp) // returns current controller error signal
{
  spd_error = sp - getWhlSpd();
  return(spd_error);
}

double OmniRPiInterface::getWhlSpd(int n) // returns current wheel speed (average of n last readings), in m/s
{
  double rps_avg=0;
  for(int i = 0; i<n; i++)
  {
    rps_avg = rps_avg + rps[i];
  }
  rps_avg = rps_avg/double(n);
  return(rps_avg);
}

void OmniRPiInterface::stop()
{
  gpioWrite(Mot_A, 0);
  gpioWrite(Mot_B, 0);
}

void OmniRPiInterface::start()
{
  gpioSetMode(Mot_A, PI_OUTPUT);
  gpioSetMode(Mot_B, PI_OUTPUT);
  gpioSetPWMfrequency(Mot_A, 10000);
  gpioSetPWMfrequency(Mot_B, 10000);
}
