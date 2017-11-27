#include <iostream>

#include "kinematics.h"

double teste;

using namespace std;

int main(void)
{
  teste = getRandom(0.5);
  inverseKinematicsMobile();
  cout << teste;
  cout << "\n";
  cout << omegapL;
  cout << "\n";
  omegapL = 85;
  cout << omegapL;
  cout << "\n";

  return 0;
}
