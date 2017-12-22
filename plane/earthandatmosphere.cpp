#include "earthandatmosphere.h"
#include "QFile"
#include "QStringList"
#include "QString"
#include "QDir"

namespace planesimulator {

EarthAndAtmosphere::EarthAndAtmosphere(QObject *parent) : QObject(parent)
{

}

EarthAndAtmosphere::EarthAndAtmosphere(QObject *parent, PlaneModel *p, Frame *f) : QObject(parent)
{
    plane = p;
    frame = f;
    log = logSystem::instance();
    dm = DataManager::instance();

}

EarthAndAtmosphere::~EarthAndAtmosphere(){
    gsl_spline_free(spline_cubic1);
    gsl_spline_free(spline_cubic2);
    gsl_spline_free(spline_cubic3);
    gsl_spline_free(spline_cubic4);
    gsl_interp_accel_free(acc1);
    gsl_interp_accel_free(acc2);
    gsl_interp_accel_free(acc3);
    gsl_interp_accel_free(acc4);
}

void EarthAndAtmosphere::initialEAA(){

    double tmp1[360][181];
    QString currentPath, fileName;
    QDir dir;
    currentPath=dir.currentPath();
    fileName = currentPath + "/config/egm96.txt";
    QFile egm96file(fileName);
    int row = 0;

    memset(tmp1,0,sizeof(tmp1));
    if (egm96file.open(QIODevice::ReadOnly | QIODevice::Text)){
        while (!egm96file.atEnd()){
            QByteArray line = egm96file.readLine();
            QStringList strList = QString(line).split('\t');
            for(int i=0; i<181; i++){
                QString tmpStr = strList.at(i);
                bool ok;
                float tmpData = tmpStr.toFloat(&ok);
                if(ok){
                   tmp1[row][i] = tmpData;
                }else{
                    i--;
                    continue;
                }
            }
            row++;
        }
        memcpy(egm96Tabel,tmp1,sizeof(tmp1));
        egm96file.close();
    }else{
    // add error
        log->addError(1, 2, egm96file.errorString());
    }

    for(int i=0; i<181; i++){
        egm96LatTabel[i] = -90 + i;
    }

    for(int i=0; i<360; i++){
        egm96LonTabel[i] = i;
    }

    size_t xsize = 181;
    size_t ysize = 360;

    const gsl_interp2d_type *T = gsl_interp2d_bilinear;
    splineEgm96 = gsl_spline2d_alloc(T,  xsize, ysize);
    xaccEgm96 = gsl_interp_accel_alloc();
    yaccEgm96 = gsl_interp_accel_alloc();

    gsl_spline2d_init(splineEgm96, &egm96LatTabel[0], &egm96LonTabel[0], &egm96Tabel[0][0], xsize, ysize);

    const size_t N = 861;
    for(size_t i=0; i<N; i++){
        atmosphereAltTabel[i] = i * 100;
    }
    fileName = currentPath + "/config/atmosphere.txt";
    QFile atmospherefile(fileName);
    double tmp2[861][4];
    memset(tmp2,0,sizeof(tmp2));
    row = 0;
    if (atmospherefile.open(QIODevice::ReadOnly | QIODevice::Text)){
        while (!atmospherefile.atEnd()){
            QByteArray line = atmospherefile.readLine();
            QStringList strList = QString(line).split('\t');
            /*for(int i=0; i<4; i++){
                QString tmpStr = strList.at(i);
                bool ok;
                float tmpData = tmpStr.toFloat(&ok);
                if(ok){
                   tmp2[row][i] = tmpData;
                }else{
                    i--;
                    continue;
                }
            }*/

            QString tmpStr = strList.at(0);
            bool ok;
            float tmpData = tmpStr.toFloat(&ok);
            if(ok){
               tempTabel[row] = tmpData;
            }else{
                continue;
            }

            tmpStr = strList.at(1);
            tmpData = tmpStr.toFloat(&ok);
            if(ok){
                pressureTabel[row] = tmpData;
            }else{
                continue;
            }

            tmpStr = strList.at(2);
            tmpData = tmpStr.toFloat(&ok);
            if(ok){
                densityTabel[row] = tmpData;
            }else{
                continue;
            }

            tmpStr = strList.at(3);
            tmpData = tmpStr.toFloat(&ok);
            if(ok){
                vsTabel[row] = tmpData;
            }else{
                continue;
            }

            row++;
        }
        //memcpy(atmosphereTabel,tmp2,sizeof(tmp2));
        atmospherefile.close();
    }else{
    // add error
        log->addError(4, 2, atmospherefile.errorString());
    }

    acc1 = gsl_interp_accel_alloc();
    spline_cubic1 = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic1, &atmosphereAltTabel[0], &tempTabel[0], N);

    acc2 = gsl_interp_accel_alloc();
    spline_cubic2 = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic2, &atmosphereAltTabel[0], &pressureTabel[0], N);

    acc3 = gsl_interp_accel_alloc();
    spline_cubic3 = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic3, &atmosphereAltTabel[0], &densityTabel[0], N);

    acc4 = gsl_interp_accel_alloc();
    spline_cubic4 = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic4, &atmosphereAltTabel[0], &vsTabel[0], N);

    p = 89876.3;
    t = 291.15;
    rho = 1.225;
    soundSpeed = 340.2941;
}

void EarthAndAtmosphere::GetEgm96Alt(double *MSLAlt){
    float height = plane->h;
    double lat = plane->latitude;
    double lon = plane->longtitude;

    lon = lon<0?(lon+M_PI*2):(lon);

    lon *= 180/M_PI;
    lat *= 180/M_PI;

    lat = dm->boundData(lat, -90, 90);
    lon = dm->boundData(lon, 0, 360);

    double tmp = gsl_spline2d_eval(splineEgm96, lat, lon, xaccEgm96, yaccEgm96);

    double h = height + tmp + (-0.53);

    *MSLAlt = h;

}

void EarthAndAtmosphere::GetWGS84(double *Rm, double *Rn, double *G){
    float height = plane->h;
    double lat = plane->latitude;
    int R = 6378137;
    double firstEcce = 0.0818191908426;
    double GConst = 0.00193185138639;
    double GEquator = 9.7803267714;

    double tmp1 = gsl_pow_2(sin(lat));
    double tmp2 = gsl_pow_2(firstEcce);
    double tmp3 = tmp1 * tmp2 + 1;
    double tmp4 = pow(tmp3, 1.5);
    double tmp5 = R * (1 + tmp2) / tmp4;

    *Rm = tmp5;

    tmp4 = sqrt(tmp3);
    tmp5 = R / tmp4;

    *Rn = tmp5;

    tmp2 = tmp1 * GConst + 1;
    tmp5 = GEquator * tmp2 / tmp4;
    *G = tmp5 -(3.0877e-6 - 0.0044e-6 * tmp1)*height + 0.072e-12 * gsl_pow_2(height);

}

void EarthAndAtmosphere::GetWMM(float *mag){
/*    float height = plane->h;
    double lat = plane->latitude;
    double lon = plane->longtitude;

    QString currentPath;
    QDir dir;
    currentPath=dir.currentPath();
    currentPath += "/config/WMM.COF";

    MAGtype_MagneticModel * MagneticModels[1], *TimedMagneticModel;
    int epochs = 1, epoch;
    int NumTerms, Flag = 1, nMax = 0;
    MAGtype_Geoid Geoid;
    MAGtype_Ellipsoid Ellip;
    MAGtype_CoordSpherical CoordSpherical;
    MAGtype_CoordGeodetic CoordGeodetic;
    MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
    MAGtype_Date UserDate;
    double sdate = -1;

    if(!MAG_robustReadMagModels((char *)currentPath.data(), &MagneticModels, epochs)) {
        log->addError(2, 2, "WMM.COF not found.");
    }

    if(nMax < MagneticModels[0]->nMax)
        nMax = MagneticModels[0]->nMax;
    NumTerms = ((nMax + 1) * (nMax + 2) / 2);

    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms);
    if(MagneticModels[0] == NULL || TimedMagneticModel == NULL)
    {
        //MAG_Error(2);
        log->addError(3, 2, "WMM Allocate failed.");
    }

    MAG_SetDefaults(&Ellip, &Geoid);

    //Geoid.GeoidHeightBuffer = geoidHeightBuffer;
    Geoid.Geoid_Initialized = 1;

    CoordGeodetic.lambda = lon;
    CoordGeodetic.phi = lat;
    CoordGeodetic.HeightAboveGeoid = height;

    time_t curtime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    tm t = *localtime(&curtime);
    sdate = t.tm_year;
    UserDate.DecimalYear = sdate;

    MAG_ConvertGeoidToEllipsoidHeight(&CoordGeodetic, &Geoid); /*This converts the height above mean sea level to height above the WGS-84 ellipsoid*/
//    MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical); /*Convert from geodeitic to Spherical Equations: 17-18, WMM Technical report*/
/*    epoch = ((int) UserDate.DecimalYear - 1900) / 5;
    if(epoch >= epochs)
        epoch = epochs - 1;
    if(epoch < 0)
        epoch = 0;
    MAG_TimelyModifyMagneticModel(UserDate, MagneticModels[epoch], TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
//    MAG_Geomag(Ellip, CoordSpherical, CoordGeodetic, TimedMagneticModel, &GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
/*    MAG_CalculateGridVariation(CoordGeodetic, &GeoMagneticElements);

    mag[0] = GeoMagneticElements.X;
    mag[1] = GeoMagneticElements.Y;
    mag[2] = GeoMagneticElements.Z;*/

}

void EarthAndAtmosphere::GetECEF(double *ECEF){
    float height = plane->h;
    double lat = plane->latitude;
    double lon = plane->longtitude;

    int R = 6378137;

    double slat = sin(lat);
    double slon = sin(lon);
    double clat = cos(lat);
    double clon = cos(lon);

    double firstEcce2 = gsl_pow_2(0.0818191908426);

    double slat2 = gsl_pow_2(slat);
    double tmp6 = firstEcce2 * slat2;
    double tmp2 = R * sqrt(1 - tmp6);
    double tmp5 = tmp2 + height;

    double tmp = tmp5 * clat * clon;

    double tmp1 = tmp5 * clat * slon;

    double tmp4 = (1 - firstEcce2) * tmp2;
    double tmp3 = (tmp4 + height) * slat;

    ECEF[0] = tmp;
    ECEF[1] = tmp1;
    ECEF[2] = tmp3;
}

void EarthAndAtmosphere::GetAtmosphere(){

    double MSLAlt;

    GetEgm96Alt(&MSLAlt);

    double temperature = gsl_spline_eval(spline_cubic1, MSLAlt, acc1);
    double pressure = gsl_spline_eval(spline_cubic2, MSLAlt, acc2);
    double density = gsl_spline_eval(spline_cubic3, MSLAlt, acc3);
    double vs = gsl_spline_eval(spline_cubic4, MSLAlt, acc4);

    t = temperature;
    p = pressure;
    rho = density;
    soundSpeed = vs;
}


}
