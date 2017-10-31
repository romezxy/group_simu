#include "propulsion.h"

namespace planesimulator {

Propulsion::Propulsion(QObject *parent) : QObject(parent)
{

}

Propulsion::Propulsion(QObject *parent, PlaneModel *p, EarthAndAtmosphere *e) : QObject(parent)
{
    plane = p;
    eaa = e;
    dm = DataManager::instance();
}

Propulsion::~Propulsion(){

    gsl_spline2d_free(spline);
    gsl_spline2d_free(spline1);
    gsl_interp_accel_free(xacc);
    gsl_interp_accel_free(yacc);
    gsl_interp_accel_free(xacc1);
    gsl_interp_accel_free(yacc1);
    gsl_spline_free(spline_cubic);
    gsl_interp_accel_free(acc2);
    gsl_spline_free(spline_cubic1);
    gsl_interp_accel_free(acc3);

}

void Propulsion::initialPropusion(){

    Omega = 5000*M_PI/30;
    EPS = 1e-8;
    pSL = 102300; // Pa
    TSL = 291.15; //deg K
    Jeng = 0.0001; // kg*m^2
    Rprop = 0.254; // m
    Jprop = 0.002; // kg*m^2
    timeStepLast = chrono::steady_clock::now();

    double temp1[9] = {60,70,80,90,92,94,96,98,100};
    memcpy(MAPTable,temp1,sizeof(temp1));
    MAPmin = MAPTable[0];//MAP(0)

    double temp4[9] = {1500,2100,2800,3500,4500,5100,5500,6000,7000};
    memcpy(RPMTable,temp4,sizeof(temp4));

    double temp2[9][9] = {
        {31,32,46,53,55,57,65,73,82},
        {40,44,54,69,74,80,92,103,111},
        {50,63,69,92,95,98,126,145,153},
        {66,75,87,110,117,127,150,175,190},
        {83,98,115,143,148,162,191,232,246},
        {93,102,130,159,167,182,208,260,310},
        {100,118,137,169,178,190,232,287,313},
        {104,126,151,184,191,206,253,326,337},
        {123,144,174,210,217,244,321,400,408},
    };
    memcpy(FuelFlowTable,temp2,sizeof(temp2));

    double temp3[9][9] = {
        {18.85,47.12,65.97,67.54,69.12,67.54,67.54,69.12,86.39},
        {59.38,98.96,127.55,149.54,151.74,160.54,178.13,200.12,224.31},
        {93.83,149.54,187.66,237.5,249.23,255.1,307.88,366.52,398.77},
        {109.96,161.27,245.57,307.88,326.2,351.86,421.5,491.14,531.45},
        {164.93,245.04,339.29,438.25,447.68,494.8,565.49,673.87,772.83},
        {181.58,245.67,389.87,496.69,528.73,571.46,662.25,822.47,993.37},
        {184.31,293.74,403.17,535.64,570.2,622.04,748.75,956.09,1059.76},
        {163.36,276.46,420.97,565.49,609.47,691.15,860.8,1130.97,1193.81},
        {124.62,249.23,417.83,586.43,645.07,762.36,996.93,1246.17,1429.42}
    };
    memcpy(PowerTable,temp3,sizeof(temp3));

    double temp5[16] = {-1,0,0.1,0.2,0.3,0.35,0.4,0.45,0.5,0.6,0.7,0.8,0.9,1,1.2,2};
    memcpy(JTable,temp5,sizeof(temp5));
    double temp6[16] = {0.0492,0.0286,0.0266,0.0232,0.0343,0.034,0.0372,0.0314,0.0254,0.0117,-0.005,-0.0156,-0.0203,-0.0295,-0.04,-0.1115};
    memcpy(CTTable,temp6,sizeof(temp6));
    double temp7[16] = {0.0199,0.0207,0.0191,0.0169,0.0217,0.0223,0.0254,0.0235,0.0212,0.0146,0.0038,-0.005,-0.0097,-0.018,-0.0273,-0.0737};
    memcpy(CPTable,temp7,sizeof(temp7));

    size_t xsize = 9;
    size_t ysize = 9;

    const gsl_interp2d_type *T = gsl_interp2d_bilinear;
    spline = gsl_spline2d_alloc(T,  xsize, ysize);
    xacc = gsl_interp_accel_alloc();
    yacc = gsl_interp_accel_alloc();

    gsl_spline2d_init(spline, &MAPTable[0], &RPMTable[0], &FuelFlowTable[0][0], xsize, ysize);

    const gsl_interp2d_type *T1 = gsl_interp2d_bilinear;
    spline1 = gsl_spline2d_alloc(T1,  xsize, ysize);
    xacc1 = gsl_interp_accel_alloc();
    yacc1 = gsl_interp_accel_alloc();

    gsl_spline2d_init(spline1, &MAPTable[0], &RPMTable[0], &PowerTable[0][0], xsize, ysize);

    const size_t N = 16;
    acc2 = gsl_interp_accel_alloc();
    spline_cubic = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic, &JTable[0], &CTTable[0], N);

    acc3 = gsl_interp_accel_alloc();
    spline_cubic1 = gsl_spline_alloc(gsl_interp_cspline, N);
    gsl_spline_init(spline_cubic1, JTable, CPTable, N);

    fuelMass = 2;
}

void Propulsion::CalPiston(double P, double T, double Thr, double Mix){

    //float MAP,RPM,power,torque,fuelflow,airflow;
    /*
    get P T from some where;
    get Thr mix from controller
    */

    P /= 1000.0;
    double tmp = Thr * (P - MAPmin);
    MAP = tmp + MAPmin;

    RPM = Omega * 30/M_PI;

    double tmp1 = 1/1000.0/3600.0;
    double tmp2 = gsl_spline2d_eval(spline, MAP, RPM, xacc, yacc);
    fuelflow = tmp1 * tmp2;
    airflow = fuelflow * Mix;

    double tmp3 = gsl_spline2d_eval(spline1, MAP, RPM, xacc1, yacc1);
    double AltCorre = sqrt(TSL/T);
    power = tmp3 * AltCorre;
    torque = power/Omega;

    //dm->SetPlotData(tmp3);

}
void Propulsion::CalPropeller(double airSpeed, double rho){
    double J;//前进比

    J = M_PI * airSpeed / (Omega * Rprop);

    CT = gsl_spline_eval(spline_cubic, J, acc2);

    CP = gsl_spline_eval(spline_cubic1, J, acc3);

    Fprop = 4 * CT * gsl_pow_4(Rprop) * gsl_pow_2(Omega) *  rho / gsl_pow_2(M_PI);
    Mprop = -4 * CP * gsl_pow_5(Rprop) * gsl_pow_2(Omega) *  rho / gsl_pow_3(M_PI);

    //dm->SetPlotData(torque);
}

double Propulsion::CalDotOmega(){
    double dotOmega, M, J;

    M = torque + Mprop;
    J = Jeng + Jprop;
    dotOmega = M / J;

    return dotOmega;
}

double Propulsion::CalDotOmega(double X){

    double dotOmega, M, J;
    double MpropTmp, torqueTmp;
    double rho = eaa->rho;

    MpropTmp = -4 * CP * gsl_pow_5(Rprop) * gsl_pow_2(X) *  rho / gsl_pow_3(M_PI);
    torqueTmp = power/X;

    M = torqueTmp + MpropTmp;
    J = Jeng + Jprop;
    dotOmega = M / J;

    return dotOmega;
}

void Propulsion::CalcOmegaIntegration()
{
    chrono::steady_clock::time_point tNow;
    static double dotOmegaLast = 0;
    double dotOmega = CalDotOmega();

    tNow= chrono::steady_clock::now();

    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(tNow - timeStepLast);

    double h = time_span.count();
    h = 0.02;
    Omega += 0.5 * h * (dotOmega + dotOmegaLast);

    /*double X = Omega, Xlast = X;
    double K1,K2,K3,K4;
    double dT = h;
    double dT2 = dT*0.5;

    K1 = CalDotOmega(X); // k1 = f(x)

    X = Xlast + dT2 * K1;

    K2 = CalDotOmega(X); // k2 = f(x+0.5*dT*k1)

    X = Xlast + dT2 * K2;

    K3 = CalDotOmega(X); // k3 = f(x+0.5*dT*k2)

    X = Xlast + dT * K3;

    K4 = CalDotOmega(X); // k4 = f(x+dT*k3)

    X = Xlast + dT * (K1 + 2.0 * K2 + 2.0 * K3 + K4) / 6.0;

    Omega = X;*/

    dotOmegaLast = dotOmega;
    timeStepLast = tNow;

    /*float T1,T2,S1,S2,C1,C2,R1,R2,h;
    int n=1,k,m=1;
    h= h2- h1;
    T1=0.5*h*(CalDotOmega(h2)+CalDotOmega(h1));
    while(m<=30)
    {
        double sum=0.0;
        for(k=0;k<n;k++)
        {
            double x=h1+(k+0.5)*h;
            sum=sum+CalDotOmega(x);
        }
        n=n*2,h=h*0.5;
        T2=0.5*(T1+2*h*sum),S1=(4*T2-T1)/3;
        if(m==1)
        {
            T1=T2,++m;
            continue;
        }
        S2=(4*T2-T1)/3,C1=(16*S2-S1)/15;
        if(m==2)
        {
            S1=S2,T1=T2,++m;
            continue;
        }
        S2=(4*T2-T1)/3,C2=(16*S2-S1)/15;
        if(m==3)
        {
            R1=(64*C2-C1)/63;
            C1=C2,S1=S2,T1=T2,++m;
            continue;
        }
        if(m>=4)
        {
            R2=(64*C2-C1)/63;
            if(fabs(R2-R1)<EPS)
                break;
            R1=R2,C1=C2,S1=S2,T1=T2,++m;
        }
    }
    return R2;
    */
}

void Propulsion::getProp(double *F, double *M){
    double P, T, Thr,  Mix;
    double airSpeed, rho;

    airSpeed = plane->V;
    rho = eaa->rho;
    P = eaa->p;
    T = eaa->t;

    //P = 89876.3; //1000m
    //T = 291.15; //1000m
    Thr = 0.7;
    Mix = 13;

    CalPiston(P, T, Thr, Mix);

    CalPropeller(airSpeed, rho);

    CalcOmegaIntegration();

    *F = Fprop;
    *M = Mprop;

}

}
