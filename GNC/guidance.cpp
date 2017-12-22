#include "guidance.h"

namespace GNC {

guidance::guidance(QObject *parent) : QObject(parent)
{

}

guidance::guidance(QObject *parent, planesimulator::PlaneModel *p) : QObject(parent)
{

    plane = p;
    wpList = new QList<waypoint>;
    dm = DataManager::instance();

}

void guidance::InitialGuidance(){

    //origin.lat = plane->latitude+1000.0/6378137.0;
    origin.lat = plane->latitude;
    origin.lon = plane->longtitude;

    waypoint wp0;
    wp0.index = 0;
    wp0.h = 1000;
    //wp0.lat = 1000/6378137 + origin.lat;
    //wp0.lon = 1000/6378137 + origin.lon;
    wp0.north = -1000;
    wp0.east = 1000;

    wpList->append(wp0);
}

void guidance::GetGuidance(float *yaw, float *theta, float *thr){

    double hP = plane->h;
    float northP = (plane->latitude-origin.lat)*6378137;
    float eastP = (plane->longtitude-origin.lon)*6378137*cos(origin.lat);
    float tPoint[3], pPoint[3],lPoint[3];
    float navLine[3], planeLine[3];
    float Dis, len, len1, k1, k2, a1;
    static bool isArravirl = false;

    northP = plane->posn;
    eastP = plane->pose;

    waypoint wptmp;
    wptmp = wpList->at(0);

    tPoint[0] = wptmp.north;
    tPoint[1] = wptmp.east;
    tPoint[2] = 1;
    lPoint[0] = 0;
    lPoint[1] = 0;
    lPoint[2] = 1;
    pPoint[0] = northP;
    pPoint[1] = eastP;
    pPoint[2] = 1;

    dm->CrossProduct(tPoint, pPoint, planeLine);
    dm->CrossProduct(tPoint, lPoint, navLine);
    k1 = atan2((tPoint[1] - lPoint[1]),(tPoint[0] - lPoint[0]));
    k2 = atan2((tPoint[1] - pPoint[1]),(tPoint[0] - pPoint[0]));
    //k2 = atan2(planeLine[0], planeLine[1]);

    float norm = sqrt((gsl_pow_2(planeLine[0])+gsl_pow_2(planeLine[1]))*(gsl_pow_2(navLine[0])+gsl_pow_2(navLine[1])));

    a1 = acos((planeLine[0]*navLine[0]+planeLine[1]*navLine[1])/norm);
    float sign = k2>k1?(-1):1;
    a1 *= sign;

    len = sqrt(gsl_pow_2(tPoint[0]-pPoint[0])+gsl_pow_2(tPoint[1]-pPoint[1]));

    float cylx = 500+500/sqrt(2)/sqrt(2);
    float cyly = 1000-500/sqrt(2)/sqrt(2);
    float len2 = sqrt(gsl_pow_2(cylx-pPoint[0])+gsl_pow_2(cyly-pPoint[1]));

    /*if((!isArravirl)){
        isArravirl = len2>1?false:true;
        Dis = -(len)*sin(a1);
    }else{

        float cx = 500;
        float cy = 1000;
        float len = sqrt(gsl_pow_2(cx-pPoint[0])+gsl_pow_2(cy-pPoint[1]));
*/
        Dis = (len - 1000*sqrt(2));

        if(k2<0){
            if(k2>-M_PI_2)
                k2 = M_PI_2-k2;
            else
                k2 = M_PI_2+M_PI+k2;
        }else{
            if(k2>M_PI_2)
                k2 -= M_PI_2;
            else
                k2 = M_PI_2+k2;
        }
    //}
    double rr = Dis;
    Dis = Dis>10?(10):(Dis<-10?(-10):Dis);

    if(pPoint[1]>300)
        *thr = 0.7;

    *yaw = k2 + Dis*0.01;
    //*yaw = 0;

    len1 = sqrt(gsl_pow_2(tPoint[0]-lPoint[0])+gsl_pow_2(tPoint[1]-lPoint[1]));

    *theta = atan2((wptmp.h - hP), len1);
    //*theta = 0;
    dm->SetPlotData((*theta)*180/M_PI);
    //dm->SetPlotData((rr));

    *thr = 0.7;

    dm->SetPlotData2d(sqrt(pPoint[1]*pPoint[1]+pPoint[2]*pPoint[2]), plane->h);

}

}
