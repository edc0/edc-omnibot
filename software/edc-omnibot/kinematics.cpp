#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "kinematics.h"

#define L 0.125       // distance between body center and wheel center
#define r 0.029       // wheel radius
#define ppr 341.2     // pulses per encoder revolution
#define uss 1000000   // microseconds per second
#define rev 6.28314
#define PMS 534.18

#define pi30 0.1047197551196597705355242034774843062905347323976457118988037109375 // long double thirty = 30; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / thirty);
#define pi12 1.5707963267948965579989817342720925807952880859375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / two);
#define pi 3.141592653589793115997963468544185161590576171875 // long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne));
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));
#define sqrt3 1.732050807568877193176604123436845839023590087890625 // long double three = 3; printf("%1.70Lf\n", (long double) sqrt(three));
#define sqrt32 0.8660254037844385965883020617184229195117950439453125 // long double two = 2; long double three = 3; printf("%1.70Lf\n", (sqrt(three) / two));
#define Vmax 1.0 // set by experiments (m/s)
#define omegamax 50.0 // set by experiments (rad/s)
#define P 5.0 // PID
#define I 0.0005 // PID
#define stopDistance 0.001
#define stopAngle pi30

double Vx;
double Vy;
double Vxm;
double Vym;
double Vxw;
double Vyw;
double omegap;
double Vback;
double Vleft;
double Vright;
double VbackTarget;
double VleftTarget;
double VrightTarget;
double omegapL;
double omegapLVxm2;
double sqrtVym2;
double Vxm2;
double Vback3;
double Vleft3;
double Vright3;
double x = 0;
double y = 0;
double xm = 0;
double ym = 0;
double xOffset = 0;
double yOffset = 0;
double xw = 0;
double yw = 0;
double theta = 0;
double xTarget;
double yTarget;
double thetaTarget;
double xError;
double yError;
double xErrorI;
double yErrorI;
double thetaError;
double thetaErrorI;
double thetaOffset = 0;;

void Point::offset(double x, double y) {
  this->x += x;
  this->y += y;
}

void Point::rotate(double theta) {
  w = (std::cos(theta) * this->y) + (std::sin(theta) * this->x);
  this->x = (std::cos(theta) * this->x) - (std::sin(theta) * this->y);
  this->y = w;
}

Point::Point(const Point& p) {
  x = p.x;
  y = p.y;
}

Point::Point(double x, double y) {
  this->x = x;
  this->y = y;
}

Point::Point(Point p, double xOffset, double yOffset) {
  x = p.x + xOffset;
  y = p.y + yOffset;
}

std::vector<Point> points;
std::vector<double> angles;

Movements movement = MOVEMENT_NONE;

long long binomialCoefficient_c;
long long binomialCoefficient_i;

/**
 * source:
 * https://en.wikipedia.org/wiki/Binomial_coefficient
 * @param n the number that goes above
 * @param k the number that goes below
 * @return
 */
long long binomialCoefficient(long long n, long long k) {
    if ((k < 0) || (k > n))
        return 0;
    if ((k == 0) || (k == n))
        return 1;
    k = std::min(k, n - k);// take advantage of symmetry
    binomialCoefficient_c = 1;
    for (long binomialCoefficient_i = 0;
     binomialCoefficient_i < k; binomialCoefficient_i++)
        binomialCoefficient_c = binomialCoefficient_c
         * (n - binomialCoefficient_i) / (binomialCoefficient_i + 1);
    return binomialCoefficient_c;
}

double Bezier_t;
double Bezier_step;
long long Bezier_nT;
long long Bezier_nR;
long long Bezier_i;
double Bezier_B;
double Bezier_pX;
double Bezier_pY;
double Bezier_pTheta;

double Bezier_B_(double Bezier_n) {
    return ((double) binomialCoefficient(Bezier_n, Bezier_i))
         * std::pow((double) Bezier_t, (double) Bezier_i)
         * std::pow(1.0 - Bezier_t, (double) (Bezier_n - Bezier_i));
}

void Bezier() {
    Bezier_pX = 0;
    Bezier_pY = 0;
    for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
        Bezier_B = Bezier_B_(Bezier_nT);
        Bezier_pX += points[Bezier_i].x * Bezier_B;
        Bezier_pY += points[Bezier_i].y * Bezier_B;
    }
    Bezier_pTheta = 0;
    for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
        Bezier_pTheta += angles[Bezier_i] * Bezier_B_(Bezier_nR);
    }
}

/**
 * Resets the error values used by the PID controller.
 *
 * Should be used after a movement command if another one is fired
 * before or immediately after the previous one
 * and a continuous movement is not desired.
 */
void resetErrors() {
    xErrorI = 0;
    yErrorI = 0;
    thetaErrorI = 0;
}

/**
 * Calculates the mobile platform's velocities
 * relative to the mobile platform's frame
 * from the wheels velocities.
 */
void forwardKinematicsMobile() {
    Vleft3 = Vleft / 3.0;
    Vback3 = Vback / 3.0;
    Vright3 = Vright / 3.0;
    Vxm = (2 * Vback3) - Vleft3 - Vright3;
    Vym = (sqrt3 * Vright3) - (sqrt3 * Vleft3);
    omegap = (Vleft3 + Vback3 + Vright3) / L;
}

/**
 * Calculates the mobile platform's velocities
 * relative to the world's frame
 * from the wheels velocities.
 *
 * Also runs forwardKinematicsMobile().
 */
void forwardKinematicsWorld() {
    forwardKinematicsMobile();
    Vxw = (std::cos(theta) * Vxm) - (std::sin(theta) * Vym);
    Vyw = (std::cos(theta) * Vym) + (std::sin(theta) * Vxm);
}

/**
 * Generates a pseudo random number between 0 and maximumValue, inclusive.
 * @param maximumValue
 * @return
 */
double getRandom(double maximumValue) {
    return (((double) std::rand()) / ((double) RAND_MAX)) * maximumValue;
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 */
void inverseKinematicsMobile() {
    omegapL = omegap * L;
    sqrtVym2 = sqrt32 * Vym;
    Vxm2 = Vxm / 2.0;
    omegapLVxm2 = omegapL - Vxm2;
    Vleft  = omegapLVxm2 - sqrtVym2;
    Vback  = omegapL + Vxm;
    Vright = omegapLVxm2 + sqrtVym2;
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 *
 * Also runs inverseKinematicsMobile().
 */
void inverseKinematicsWorld() {
    Vxm = (std::cos(theta) * Vxw) + (std::sin(theta) * Vyw);
    Vym = (std::cos(theta) * Vyw) - (std::sin(theta) * Vxw);
    inverseKinematicsMobile();
}

bool isStopPose() {
    return (std::abs(xError) < stopDistance)
     && (std::abs(yError) < stopDistance)
     && (std::abs(thetaError) < stopAngle);
}

double normalizeRadian(double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

// C++ bug: when things are declared as 'static' or 'const',
// they lose their references, and it's only possible to read from them
// to restore their references, they must be decladed again like this:
double Point::w;
