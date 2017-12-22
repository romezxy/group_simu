#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <QObject>

#include "plane/planemodel.h"

namespace GNC{

class navigation : public QObject
{
    Q_OBJECT
public:
    explicit navigation(QObject *parent = nullptr, planesimulator::PlaneModel *p = nullptr);

signals:

public slots:

private:
    planesimulator::PlaneModel *plane;

    void makeAcc();
    void makeGyro();

    float ax,ay,az;
    float wx,wy,wz;
    int lat,lon;
};

}

#endif // NAVIGATION_H
