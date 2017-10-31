#ifndef FRAME_H
#define FRAME_H

#include <QObject>
#include <math.h>
#include "plane/planemodel.h"

namespace planesimulator {

class PlaneModel;

class Frame : public QObject
{
    Q_OBJECT
public:
    explicit Frame(QObject *parent = nullptr);
    Frame(QObject *parent = nullptr, PlaneModel *p = nullptr);
    void UpdateRbe();
    void UpdateRbw();

    void Wind2Body(double *wind, double *body);
    void Body2Frame(double *body, double *frame);
    void Frame2Body(double *frame, double *body);

signals:

public slots:


private:
    PlaneModel *plane;

    double Rbe[3][3];
    double q[4];
    double attitude[3];
    double Rbw[3][3];

    void Quaternion2R(double *q);
    void RPY2Quaternion(double *rpy, double *q);

};

}
#endif // FRAME_H
