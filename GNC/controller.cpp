#include "controller.h"

namespace GNC{

Controller::Controller(QObject *parent) : QObject(parent)
{

}

Controller::Controller(QObject *parent, planesimulator::PlaneModel *p, guidance *g) : QObject(parent)
{
    dm = DataManager::instance();
    plane = p;
    gui = g;

    phiCmd = 0;
    thetaCmd = 0*M_PI/180;
    psiCmd = 45*M_PI/180;
    VCmd = 30;

    pidp = new pid2();
    pidq = new pid2();
    pidr = new pid2();
    pidroll = new pid2();
    pidtheta = new pid2();
    pidpsi = new pid2();
    pidthr = new pid2();

    pidpPara = new pid2Parameter();
    pidqPara = new pid2Parameter();
    pidrPara = new pid2Parameter();
    pidrollPara = new pid2Parameter();
    pidthetaPara = new pid2Parameter();
    pidpsiPara = new pid2Parameter();
    pidthrPara = new pid2Parameter();

}

void Controller::InitialCtrller(){

/*#define PID_DEFAULTS {1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f,-1.0f}
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
    gui->InitialGuidance();

    pidpPara->p = 0.5;
    pidpPara->i = 0.1;
    pidpPara->d = 0.001;
    pidpPara->iLim = 5*M_PI/180;

    pidqPara->p = 0.5;
    pidqPara->i = 0.1;
    pidqPara->d = 0.001;
    pidqPara->iLim = 10*M_PI/180;

    pidrPara->p = 0.5;
    pidrPara->i = 0.1;
    pidrPara->d = 0;
    pidrPara->iLim = 5*M_PI/180;

    pidrollPara->p = 1;
    pidrollPara->i = 2;
    pidrollPara->d = 0.01;
    pidrollPara->iLim = 10*M_PI/180;

    pidthetaPara->p = 5;
    pidthetaPara->i = 5;
    pidthetaPara->d = 0.5;
    pidthetaPara->iLim = 50*M_PI/180;

/*    pidpsiPara->p = 5;
    pidpsiPara->i = 1;
    pidpsiPara->d = 1;
    pidpsiPara->iLim = 50*M_PI/180;
*/
    pidpsiPara->p = 1;
    pidpsiPara->i = 0.5;
    pidpsiPara->d = 0;
    pidpsiPara->iLim = 50*M_PI/180;


    pidthrPara->p = 0.1;
    pidthrPara->i = 0.5;
    pidthrPara->d = 0;
    pidthrPara->iLim = 0.5;

    PIDUpdateParameters(pidp, pidpPara, 0.95, 0.02);
    PIDUpdateParameters(pidq, pidqPara, 0.95, 0.02);
    PIDUpdateParameters(pidr, pidrPara, 0.95, 0.02);
    PIDUpdateParameters(pidroll, pidrollPara, 0.95, 0.02);
    PIDUpdateParameters(pidtheta, pidthetaPara, 0.95, 0.02);
    PIDUpdateParameters(pidpsi, pidpsiPara, 0.95, 0.02);
    PIDUpdateParameters(pidthr, pidthrPara, 0.95, 0.02);
}

float Controller::calYaw(float yawComm, float yaw){

    float yawCommend, tempYaw;
    yawCommend = yawComm;
    tempYaw = yawCommend-yaw;
    if(tempYaw>=0){
        if(tempYaw>M_PI)
        {
            yawCommend = tempYaw-M_PI*2;
        }else{
            yawCommend = tempYaw;
        }
    }else{
        if(tempYaw<-M_PI)
        {
            yawCommend = M_PI*2 + tempYaw;
        }else{
            yawCommend = tempYaw;
        }
    }
    //yawCommend= yawCommend>M_PI_4?(M_PI_4):(yawCommend<-M_PI_4?(-M_PI_4):(yawCommend));

    return yawCommend;
}

void Controller::pidCtrller(double *out){
    double phiFb = plane->phi;
    double thetaFb = plane->theta;
    double psiFb = plane->psi;
    double pFb = plane->p;
    double qFb = plane->q;
    double rFb = plane->r;
    double VFb = plane->V;

    float thrCmd,psiCmd, thetaCmd;

    gui->GetGuidance(&psiCmd, &thetaCmd, &thrCmd);

    if(plane->simuTime>20){
//        rCmd = 0*M_PI/180;
        thetaCmd = 0*M_PI/180;
        psiCmd = 45*M_PI/180;
    }else{
//        rCmd = 0;
        thetaCmd = 0;
        psiCmd = 0*M_PI/180;
    }

    //double tmp1 = calYaw(psiCmd, psiFb);
    //psiCmd = tmp1+psiFb;

    double drCmd = CalcPids(pidpsi, psiCmd, psiFb, -20*M_PI/180, 20*M_PI/180);
    //double drCmd = CalcPids(pidr, rCmd, rFb, -20*M_PI/180, 20*M_PI/180);

    //phiCmd = 0*M_PI/180;
    double daCmd = CalcPids(pidroll, drCmd, phiFb, -20*M_PI/180, 20*M_PI/180);
    //double daCmd = CalcPids(pidp, pCmd, pFb, -20*M_PI/180, 20*M_PI/180);

    //thetaCmd = 0*M_PI/180;
    double deCmd = CalcPids(pidtheta, thetaCmd, thetaFb, -20*M_PI/180, 20*M_PI/180);
    //double deCmd = CalcPids(pidq, qCmd, qFb, -20*M_PI/180, 20*M_PI/180);
    //if(fabs(phiFb)>20*M_PI/180){
    //    deCmd -= phiFb*0.5;
    //}

    thrCmd = CalcPids(pidthr, VCmd, VFb, 0.0, 1.0);

    out[0] = daCmd;
    out[1] = deCmd;
    out[2] = drCmd;
    out[3] = thrCmd;

}

void Controller::FeedbackLinearCtrller(double *out){

    float thrCmd,psiCmd, thetaCmd;

    double phiFb = plane->phi;
    double thetaFb = plane->theta;
    double psiFb = plane->psi;
    double pFb = plane->p;
    double qFb = plane->q;
    double rFb = plane->r;
    double VFb = plane->V;
    double alpha = plane->alpha;
    double pdyn = plane->pdyn;

    //gui->GetGuidance(&psiCmd, &thetaCmd, &thrCmd);

    thetaCmd = 0;
    static float errorLast = 0;
    static float itheta = 0;
    float error = thetaCmd - thetaFb;
    itheta += (error+errorLast)*0.02*0.5;
    errorLast = error;
    float qcom = error*0.5;// + itheta*0.01;

    static float errorLastq = 0;
    static float iq = 0;
    float fiq = 0.25, kbq =12, fc = 1;
    float qc1 = 0;
    float errorq1, errorq2;
    float qdot;

    errorq1 = (qcom - qFb);
    iq += (errorq1 + errorLastq)*0.02*0.5;
    errorLastq = errorq1;
    qc1 = iq *fiq*kbq + fc*qcom;

    errorq2 = qc1 - qFb;
    qdot = errorq2*kbq;

    //now feedbacklinear
    float Iyy = 1.135,  Ixx = 0.8244, Izz = 1.759, Ixz = 0.1204;
    float S = 0.55, MAC = 0.1889941;
    float qsc = pdyn * S * MAC, cm1 = 0.135, cma = -2.7397, cmq = -38.2067, cme = -0.9918;
    float deCmd = 0, cmade = 0, Mb = 0;
    Mb = qdot * Iyy + (Ixx-Izz)*pFb*rFb - Ixz *(pFb*pFb - rFb*rFb);
    cmade = Mb/(qsc)-cm1 - cma*alpha - (cmq*MAC/(VFb*2))*qFb;
    deCmd = cmade/cme;

    out[1] = deCmd;

}

void Controller::GetAttitudeCtrller(){



    pidCtrller(&out[0]);

    FeedbackLinearCtrller(&out[0]);

/*    out[0] = daCmd;
    out[1] = deCmd;
    out[2] = drCmd;
    out[3] = thrCmd;*/

}

void Controller::PIDUpdateParameters(pid2 *pid, pid2Parameter *para, float beta, float dT)
{
    float Ti = para->p / para->i;
    float Td = para->d / para->p;
    float Tt = (Ti + Td) / 2.0f;
    float kt = 1.0f / Tt;
    float u0 = 0.0f;
    float N  = 10.0f;
    float Tf = Td / N;

    if (para->i < 1e-6f) {
        // Avoid Ti being infinite
        Ti = 1e6f;
        // Tt antiwindup time constant - we don't need antiwindup with no I term
        Tt = 1e6f;
        kt = 0.0f;
    }

    if (para->d < 1e-6f) {
        // PI Controller
        Tf = Ti / N;
    }

    if (beta > 1.0f) {
        beta = 1.0f;
    } else if (beta < 0.4f) {
        beta = 0.4f;
    }

        pid->reconfigure = true;
        pid->iLup = para->iLim;
        pid->iLdown = -para->iLim;
    Pid2Configure(pid, para->p, para->i, para->d, Tf, kt, dT, beta, u0, 0.0f, 1.0f);
}

/**
 * Configure the settings for a pid2 structure
 * @param[out] pid The PID2 structure to configure
 * @param[in] kp proportional gain
 * @param[in] ki integral gain.  Time constant Ti = kp/ki
 * @param[in] kd derivative gain. Time constant Td = kd/kp
 * @param[in] Tf filtering time = (kd/k)/N, N is in the range of 2 to 20
 * @param[in] kt tracking gain for anti-windup. Tt = √TiTd and Tt = (Ti + Td)/2
 * @param[in] dt delta time increment
 * @param[in] beta setpoint weight on setpoint in P component.  beta=1 error feedback. beta=0 smoothes out response to changes in setpoint
 * @param[in] u0 initial output for r=y at activation to achieve bumpless transfer
 * @param[in] va constant for compute of actuator output for check against limits for antiwindup
 * @param[in] vb multiplier for compute of actuator output for check against limits for anti-windup
 */
void Controller::Pid2Configure(pid2 *pid,
                    float kp, float ki, float kd,
                    float Tf, float kt, float dT,
                    float beta, float u0, float va, float vb){

    pid->reconfigure = true;
    pid->u0   = u0;
    pid->va   = va;
    pid->vb   = vb;
    pid->kp   = kp;
    pid->beta = beta; // setpoint weight on proportional term

    pid->bi   = ki * dT;
    pid->br   = kt * dT / vb;

    pid->ad   = Tf / (Tf + dT);
    pid->bd   = kd / (Tf + dT);
}

/**
 * Achieve a bumpless transfer and trigger initialisation of I term
 * @param[out] pid The PID structure to configure
 * @param[in] u0 initial output for r=y at activation to achieve bumpless transfer
 */
void Controller::Pid2Transfer(pid2 *pid, float u0){
    pid->reconfigure = true;
    pid->u0 = u0;
}

/**
 * pid controller with setpoint weighting, anti-windup, with a low-pass filtered derivative on the process variable
 * See "Feedback Systems" for an explanation
 * @param[out] pid The PID structure to configure
 * @param[in] r setpoint
 * @param[in] y process variable
 * @param[in] ulow lower limit on actuator
 * @param[in] uhigh upper limit on actuator
 */
float Controller::CalcPids(
    pid2 *pid,
    const float r,
    const float y,
    const float ulow,
    const float uhigh)
{
    // on reconfigure ensure bumpless transfer
    // http://www.controlguru.com/2008/021008.html

    float v;
    float u;
    if (pid->reconfigure) {
        pid->reconfigure = false;

        // initialise derivative terms
        pid->yold = y;
        pid->D    = 0.0f;

        // t=0, u=u0, y=y0, v=u
        //pid->I    = (pid->u0 - pid->va) / pid->vb - pid->kp * (pid->beta * r - y);
        pid->I = 0;
    }

    // compute proportional part
    pid->P = pid->kp * (pid->beta * r - y);

    // update derivative part
    pid->D = pid->ad * pid->D - pid->bd * (y - pid->yold);

    // compute temporary output
    v = pid->va + pid->vb * (pid->P + pid->I + pid->D);

    // simulate actuator saturation
    //u = boundf(v, ulow, uhigh);
    u = v>uhigh?uhigh:(v<ulow?ulow:v);

    // update integral
    pid->I    = pid->I + pid->bi * (r - y) + pid->br * (u - v);

    pid->I    = dm->boundData(pid->I, pid->iLdown, pid->iLup);

    // update old process output
    pid->yold = y;

    return u;
}

//void Controller::CalcPids(PID *pid, float r, float y, float i,float *out){
/*    float v5, v8, v9,v1, v4;

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
*/

//}

void Controller::RunController(){
    GetAttitudeCtrller();

    plane->SetCtrller(&out[0]);
    emit ControllerDone();
}

}
