#ifndef KINEMATICS_H
#define KINEMATICS_H

extern double Vx;
extern double Vy;
extern double Vxm;
extern double Vym;
extern double Vxw;
extern double Vyw;
extern double omegap;
extern double Vback;
extern double Vleft;
extern double Vright;
extern double VbackTarget;
extern double VleftTarget;
extern double VrightTarget;
extern double omegapL;
extern double omegapLVxm2;
extern double sqrtVym2;
extern double Vxm2;
extern double Vback3;
extern double Vleft3;
extern double Vright3;
extern double x;
extern double y;
extern double xTarget;
extern double yTarget;
extern double thetaTarget;
extern double xm;
extern double ym;
extern double xOffset;
extern double yOffset;
extern double xw;
extern double yw;
extern double xError;
extern double yError;
extern double xErrorI;
extern double yErrorI;
extern double theta;
extern double thetaError;
extern double thetaErrorI;
extern double thetaOffset;

class Point {

    public: double x;
    public: double y;
    private: static double w;
    public: void offset(double x, double y);
    public: void rotate(double theta);

    public: Point();
    public: Point(const Point& p);
    public: Point(double x, double y);
    public: Point(Point p, double xOffset, double yOffset);
};

enum Movements {
    MOVEMENT_DIRECT,
    MOVEMENT_DIRECT_M,
    MOVEMENT_DIRECT_W,
    MOVEMENT_NONE,
    MOVEMENT_ABSOLUTE_M,
    MOVEMENT_ABSOLUTE_W,
    MOVEMENT_BEZIER_W
};

long long binomialCoefficient(long long n, long long k);
double Bezier_B_(double Bezier_n);
void Bezier();
void resetErrors();
void forwardKinematicsMobile();
void forwardKinematicsWorld();
double getRandom(double maximumValue);
void inverseKinematicsMobile();
void inverseKinematicsWorld();
bool isStopPose();
double normalizeRadian(double radian);

#endif
