#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include "data/datamanager.h"
#include "plane/planemodel.h"

namespace GNC {

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
} PID;

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(QObject *parent = nullptr);
    Controller(QObject *parent = nullptr, planesimulator::PlaneModel *p = nullptr);

    void GetAttitudeCtrller();
    void InitialCtrller();

    double out[3];

signals:
    void ControllerDone();

public slots:
    void RunController();

private:
    DataManager *dm;
    planesimulator::PlaneModel *plane;

    double phiCmd, thetaCmd, psiCmd;
    PID *pidp,*pidq,*pidr,*pidroll,*pidtheta,*pidpsi;

    void CalcPids();
};

}

#endif // CONTROLLER_H
