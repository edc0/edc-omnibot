// Did not put declare variables as extern here because this interface is to be
// used exclusively with the <kinematics> library.

#include <pigpio>
#include "rotary_encoder.hpp"
#include "rpi_interface.h"

void OmniRPiInterface::dec_callback(int way)
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
  erro = sp - getAngSpd();
  control_new = control_old + Kp*erro1;   //q q eu faço com os integral e derivativo?

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
}

double OmniRPiInterface::getError(double sp) // returns current controller error signal
{
  error = sp - getAngSpd();
  return(error);
}

double getAngSpd(int n = 5) // returns current wheel speed (average of n last readings)
{
  double spd_avg=0;
  for(int i = 0; i<n; i++)
  {
    spd_avg = spd_avg + spd[i];
  }
  spd_avg = spd_avg/double(n);
}
