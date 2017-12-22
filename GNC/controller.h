#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include "data/datamanager.h"
#include "plane/planemodel.h"
#include "GNC/guidance.h"

namespace GNC {

class guidance;

// pid2 structure for a PID+setpoint weighting, anti-windup and filtered derivative control
typedef struct {
    float   u0;
    float   va;
    float   vb;
    float   kp;
    float   bi;
    float   ad;
    float   bd;
    float   br;
    float   beta;
    float   yold;
    float   P;
    float   I;
    float   D;
    float iLup;
    float iLdown;
    uint8_t reconfigure;
}pid2;

typedef struct {
    float p;
    float i;
    float d;
    float iLim;
}pid2Parameter;

typedef struct {
    float Kp;    // proportional gain
    float Ki;    // integral gain
    float Kd;    // derivative gain
    float Kr;    // set point weight
    float c1;    // D filter coefficient 1
    float c2;    // D filter coefficient 2
    float d2;    // D filter storage 1
    float d3;    // D filter storage 2
    float i10;    // I storage
    float i14;    // sat storage
    float Umax;    // upper saturation limit
    float Umin;    // lower saturation limit
}PID;

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(QObject *parent = nullptr);
    Controller(QObject *parent = nullptr, planesimulator::PlaneModel *p = nullptr, guidance *gui = nullptr);

    void GetAttitudeCtrller();
    void InitialCtrller();

    double out[4];

signals:
    void ControllerDone();

public slots:
    void RunController();

private:
    DataManager *dm;
    planesimulator::PlaneModel *plane;
    guidance *gui;

    double phiCmd, thetaCmd, psiCmd, VCmd;
    //PID *pidp,*pidq,*pidr,*pidroll,*pidtheta,*pidpsi;
    pid2 *pidp,*pidq,*pidr,*pidroll,*pidtheta,*pidpsi,*pidthr;
    pid2Parameter *pidpPara,*pidqPara,*pidrPara,*pidrollPara,*pidthetaPara,*pidpsiPara,*pidthrPara;

    //void CalcPids(PID *pid, float r, float y, float i,float *out);
    void Pid2Configure(pid2 *pid,
                        float kp, float ki, float kd,
                        float Tf, float kt, float dT,
                        float beta, float u0, float va, float vb);
    void Pid2Transfer(pid2 *pid, float u0);
    float CalcPids(pid2 *pid,
                const float r,
                const float y,
                const float ulow,
                const float uhigh);
    void PIDUpdateParameters(pid2 *pid,
                             pid2Parameter *para,
                             float beta, float dT);

    float calYaw(float yawComm, float yaw);
    void pidCtrller(double *out);
    //void feedbackCtrller(float *out);
    void FeedbackLinearCtrller(double *out);

};

}

#endif // CONTROLLER_H
