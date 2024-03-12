#ifndef HANDCONTROL_H
#define HANDCONTROL_H

#define PI 3.1415926535897932384626433832795

class HandControl
{
public:

    HandControl();
    void jc(double *q, double *q_dot, double *kp, double *kd, double *q_d, double* u);
    void grasp1(double gf, double *q, double* u);
    void grasp2(double gf, double *q, double* u);
    void grasp3(double gf, double *q, double* u);
    void u2duty(double *u, double *duty);
    void SetCoordinate(double* q);
    void DetailedControl(double* q, double* q_dot, double* kp, double* kd, bool* mode, int mul, double* data, double* u);
};

#endif // HANDCONTROL_H
