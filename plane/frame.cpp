#include "frame.h"

namespace planesimulator {
Frame::Frame(QObject *parent) : QObject(parent)
{

}

Frame::Frame(QObject *parent, PlaneModel *p): QObject(parent){
    plane = p;
    memset(Rbw,0,sizeof(float)*9);
}

void Frame::Wind2Body(double *wind, double *body){

    double tmp;
    for(int i=0; i<3; i++){
        tmp = 0;
        for(int j=0; j<3; j++){
            tmp +=  Rbw[j][i] * wind[j];
        }
        body[i] = tmp;
    }
}

void Frame::Body2Frame(double *body, double *frame){
    int i,j;
    double tempR;

    for(i=0;i<3;i++){
        tempR = 0;
        for(j=0;j<3;j++){
            tempR += Rbe[j][i]*body[j];
        }
        frame[i] = tempR;
    }
}
void Frame::Frame2Body(double *frame, double *body){
    int i,j;
    double tempR;

    for(i=0;i<3;i++){
        tempR = 0;
        for(j=0;j<3;j++){
            tempR += Rbe[i][j]*frame[j];
        }
        body[i] = tempR;
    }
}

void Frame::RPY2Quaternion(double *rpy, double *q)
{
    double phi, theta, psi;
    double cphi, sphi, ctheta, stheta, cpsi, spsi;

    phi    = DEG2RAD(rpy[0] / 2);
    theta  = DEG2RAD(rpy[1] / 2);
    psi    = DEG2RAD(rpy[2] / 2);

    cphi   = cosf(phi);
    sphi   = sinf(phi);
    ctheta = cosf(theta);
    stheta = sinf(theta);
    cpsi   = cosf(psi);
    spsi   = sinf(psi);

    q[0]   = cphi * ctheta * cpsi + sphi * stheta * spsi;
    q[1]   = sphi * ctheta * cpsi - cphi * stheta * spsi;
    q[2]   = cphi * stheta * cpsi + sphi * ctheta * spsi;
    q[3]   = cphi * ctheta * spsi - sphi * stheta * cpsi;

    if (q[0] < 0) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

void Frame::Quaternion2R(double *q){
    const double q0s = q[0] * q[0], q1s = q[1] * q[1], q2s = q[2] * q[2], q3s = q[3] * q[3];

    Rbe[0][0] = q0s + q1s - q2s - q3s;
    Rbe[0][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
    Rbe[0][2] = 2 * (q[1] * q[3] - q[0] * q[2]);
    Rbe[1][0] = 2 * (q[1] * q[2] - q[0] * q[3]);
    Rbe[1][1] = q0s - q1s + q2s - q3s;
    Rbe[1][2] = 2 * (q[2] * q[3] + q[0] * q[1]);
    Rbe[2][0] = 2 * (q[1] * q[3] + q[0] * q[2]);
    Rbe[2][1] = 2 * (q[2] * q[3] - q[0] * q[1]);
    Rbe[2][2] = q0s - q1s - q2s + q3s;
}

void Frame::UpdateRbe(){
    double q_l[4];

    attitude[0] = plane->phi;
    attitude[1] = plane->theta;
    attitude[2] = plane->psi;
    RPY2Quaternion(attitude, q_l);
    q[0] = q_l[0];
    q[1] = q_l[1];
    q[2] = q_l[2];
    q[3] = q_l[3];
    Quaternion2R(q_l);
}

void Frame::UpdateRbw(){
    double a = plane->alpha;
    double b = plane->beta;

    double sa = sin(a);
    double ca = cos(a);
    double sb = sin(b);
    double cb = cos(b);

    Rbw[0][0] = ca*cb;
    Rbw[1][0] = -ca*sb;
    Rbw[2][0] = -sa;

    Rbw[0][1] = sb;
    Rbw[1][1] = cb;
    Rbw[2][1] = 0;

    Rbw[0][2] = sa*cb;
    Rbw[1][2] = -sa*sb;
    Rbw[2][2] = ca;
}

}
