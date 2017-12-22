#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <QObject>
#include "plane/planemodel.h"
#include "gsl/gsl_math.h"

namespace GNC {

typedef struct{
    double lat;
    double lon;
    float h;
    float north;
    float east;
    unsigned int index;//from 0;
}waypoint;

class guidance : public QObject
{
    Q_OBJECT
public:
    explicit guidance(QObject *parent = nullptr);
    explicit guidance(QObject *parent = nullptr, planesimulator::PlaneModel *p = nullptr);
    void GetGuidance(float *yaw, float *theta, float *thr);
    void InitialGuidance();

signals:

public slots:

private:

    planesimulator::PlaneModel *plane;
    QList<waypoint> *wpList;
    DataManager *dm;

    waypoint origin;

};

}
#endif // GUIDANCE_H
