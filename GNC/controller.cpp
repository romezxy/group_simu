#include "controller.h"

namespace GNC{

Controller::Controller(QObject *parent) : QObject(parent)
{

}

Controller::Controller(QObject *parent, planesimulator::PlaneModel *p) : QObject(parent)
{
    dm = DataManager::instance();
    plane = p;

    phiCmd = 0;
    thetaCmd = 10*M_PI/180;
    psiCmd = 0;

    pidp = new PID();
    pidq = new PID();
    pidr = new PID();
    pidroll = new PID();
    pidtheta = new PID();
    pidpsi = new PID();

}

void Controller::InitialCtrller(){

    //#define PID_DEFAULTS {1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f,-1.0f}
    pidp ->Kp = 1.0;
    pidp ->Ki = 0.0;
    pidp ->Kd = 0.0;
    pidp ->Kr = 1.0;
    pidp ->c1 = 0.0;
    pidp ->c2 = 0.0;
    pidp ->d2 = 0.0;
    pidp ->d3 = 0.0;
    pidp ->i10 = 0.0;
    pidp ->i14 = 0.0;
    pidp ->Umax = 1.0;
    pidp ->Umin = -1.0;

    memcpy(pidq, pidp, sizeof(PID));
    memcpy(pidr, pidp, sizeof(PID));
    memcpy(pidroll, pidp, sizeof(PID));
    memcpy(pidtheta, pidp, sizeof(PID));
    memcpy(pidpsi, pidp, sizeof(PID));
/*
    pidq ->Kp = 1.0;
    pidq ->Ki = 0.0;
    pidq ->Kd = 0.0;
    pidq ->Kr = 1.0;
    pidq ->c1 = 0.0;
    pidq ->c2 = 0.0;
    pidq ->d2 = 0.0;
    pidq ->d3 = 0.0;
    pidq ->i10 = 0.0;
    pidq ->i14 = 0.0;
    pidq ->Umax = 1.0;
    pidq ->Umin = -1.0;
*/
}

void Controller::GetAttitudeCtrller(){

    double phiFb = plane->phi;
    double thetaFb = plane->theta;
    double psiFb = plane->psi;

    double errorphi = phiCmd - phiFb;
    double errortheta = thetaCmd - thetaFb;
    double errorpsi = psiCmd - psiFb;

    double kp = 1;

    out[0] = -kp * errorphi;
    out[1] = -kp * errortheta;
    out[2] = -kp * errorpsi;

}

void Controller::CalcPids(PID *pid, float r, floay y, float i,float *out){
    float v5, v8, v9,v1, v4;

    //p
    v5 = pid->Kr*r - y;

    //i
    v8 = pid->i10 + pid->i14*Kp*Ki*(r-y);

    pid->i10 = v8;
    pid->i14 = 1;//

    //d
    v1 = pid->Kd * pid->c1 * y;

    v4 = v1 - pid->d2 - pid->d3;

    pid->d2 = v1;
    pid->d3 = pid->c2*v4;

    v9= v8 + pid->Kp*(v4 + v5);

    *out = v9>pid->Umax?(pid->Umax):(v9<pid->Umin?(pid->Umin):(v9));




}

void Controller::RunController(){
    GetAttitudeCtrller();

    plane->SetCtrller(&out[0]);
    emit ControllerDone();
}

}
