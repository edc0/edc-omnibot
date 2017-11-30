#ifndef RPI_INTERFACE_H
#define RPI_INTERFACE_H


// Did not put declare variables as extern here because this interface is to be
// used exclusively with the <kinematics> library.

class OmniRPiInterface
{
public:
  int Mot_A; // terminais do motor
  int Mot_B;
  int Enc_A; // terminais do encoder
  int Enc_B;

  int pos;     // giro atualizado do motor (somatorio dos encoders)
  int t_pos;   // tempo da atualização
  int pos_old;
  int t_pos_old;

  double rps[5];  // vetor para armazenar 5 velocidades

  double set_point;  // velocidade desejada
  double spd_error;  // diferença entre velocidade desejada e velocidade atual
  double control;    // sinal de controle
  double control_old;// sinal de controle do ciclo passado

  // parâmetros do controlador
  double Kp;
  double Ki;
  double Kd;

public:
  void resetPos();            // reinicia os contadores de posição
  void dec_callback(int way); // callback para atualização do encoder
  double getWheelPos();       // returns pos (mas se é public, pra q?)
  void setSetpoint(double sp);// define velocidade desejada
  double getError(double sp); // returns current controller error signal
  double getAngSpd(int n = 5);// returns current wheel speed (average of n last readings)
  void stop();
  OmniRPiInterface(int MA, int MB, int EA, int EB); // class constructor
};

#endif
