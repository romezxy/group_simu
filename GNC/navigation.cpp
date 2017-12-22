#include "navigation.h"

namespace GNC{

navigation::navigation(QObject *parent, planesimulator::PlaneModel *p) : QObject(parent)
{
    plane = p;
}

void navigation::makeAcc(){

    float tmpAcc[3];



    ax = 0;
    ay = 0;
    az = 0;

}
void navigation::makeGyro(){

}

}
