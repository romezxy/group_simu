#include "planemodel.h"

namespace planesimulator {
PlaneModel::PlaneModel(QObject *parent) : QObject(parent)
{

    //timeStepLast = chrono::steady_clock::now();
    timefuelFlowLast = timeStepLast;
    dm = DataManager::instance();
    statesNumber = 14;

    frame = new Frame(this,this);    
    eaa = new EarthAndAtmosphere(this,this,frame);
    prop = new Propulsion(this,this,eaa);

    isRun = true;

}

void PlaneModel::GetParameters(){

    CL = CL0 + CLa*alpha + CLdf*df + CLde*de + (CLalphadot*dotalpha+CLq*q)*MAC/(V*2);

    double AR = gsl_pow_2(b)/S;
    double k = 1/(M_PI*AR*osw);
    CD = CD0 + gsl_pow_2(CL-CLmind)*k + fabs(CDdf*df) + fabs(CDde*de) + fabs(CDda*da) + fabs(CDdr*dr);

    CY = beta*CYbeta + da*CYda + dr*CYdr;

    Cl = beta*Clbeta + da*Clda + dr*Cldr + (p*Clp + r*Clr)*b/(V*2);

    double tmp1,tmp2,tmp3,tmp4;
    tmp1=Cma*alpha;
    tmp2=Cmalphadot*dotalpha;
    tmp3=q*Cmq;
    tmp4=(Cmalphadot*dotalpha+q*Cmq)*MAC/(V*2);
    Cm = Cm0 + Cma*alpha + + Cmdf*df + de*Cmde + (Cmalphadot*dotalpha+q*Cmq)*MAC/(V*2);

    Cn = beta*Cnbeta + da*Cnda + dr*Cndr + (p*Cnp + r*Cnr)*b/(V*2);

//    gsl_spline2d_free(spline);
//    gsl_interp_accel_free(xacc);
//    gsl_interp_accel_free(yacc);

}
void PlaneModel::GetAeroForcesAndMoments(){

    //rho = pow(1.225*(1-2.0323*0.00001*h),4.830);
    eaa->GetAtmosphere();
    eaa->GetWGS84(&Rm, &Rn, &G);
    prop->getProp(&Fprop, &Mprop, thr);

    double rho = eaa->rho;

    pdyn = 0.5 * rho * V * V;

    Faerox = -CD * pdyn * S;
    Faeroy = CY * pdyn * S;
    Faeroz = -CL * pdyn * S;

    double windF[3], bodyF[3];
    windF[0] = Faerox;
    windF[1] = Faeroy;
    windF[2] = Faeroz;
    frame->Wind2Body(&windF[0], &bodyF[0]);
    Faerox = bodyF[0];
    Faeroy = bodyF[1];
    Faeroz = bodyF[2];

    //tmp->append(QString("%1|%2")
    //            .arg(windF[0], 0, 'g', 5)
    //            .arg(bodyF[0], 0, 'g', 5));
    //dm->SetPlotData(windF[0]);

    Maerox = Cl * pdyn * S * b;

    Maeroy = Cm * pdyn * S * MAC;

    Maeroz = Cn * pdyn * S * b;



//    Fx += Fprop;
//    Mx += Mprop;

}

void PlaneModel::GetInertia(){
    static float fuelMass = prop->fuelMass;
    static float fuelFlowLast = -prop->fuelflow;
    double fuelFlow;

    chrono::steady_clock::time_point tNow;
    tNow= chrono::steady_clock::now();

    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(tNow - timefuelFlowLast);

    double h = time_span.count();
    h = 0.02;
    fuelFlow = -prop->fuelflow;
    double fuelCost = 0.5 * h * (fuelFlow + fuelFlowLast);

    fuelFlowLast = fuelFlow;
    timefuelFlowLast = tNow;

    fuelMass += fuelCost;

    Mass = fuelMass + mempty;

    double tmp1 = (CGgross[0] - CGempty[0])/(mgross - mempty);
    CGpos[0] = tmp1 * fuelMass + CGempty[0];
    tmp1 = (CGgross[1] - CGempty[1])/(mgross - mempty);
    CGpos[1] = tmp1 * fuelMass + CGempty[1];
    tmp1 = (CGgross[2] - CGempty[2])/(mgross - mempty);
    CGpos[2] = tmp1 * fuelMass + CGempty[2];

    tmp1 = (Jgross[0] - Jempty[0])/(mgross - mempty);
    J[0] = tmp1 * fuelMass + Jempty[0];
    tmp1 = (Jgross[1] - Jempty[1])/(mgross - mempty);
    J[1] = tmp1 * fuelMass + Jempty[1];
    tmp1 = (Jgross[2] - Jempty[2])/(mgross - mempty);
    J[2] = tmp1 * fuelMass + Jempty[2];
    tmp1 = (Jgross[3] - Jempty[3])/(mgross - mempty);
    J[3] = tmp1 * fuelMass + Jempty[3];

}

void gsl_vector_cross_mul(gsl_vector *a, gsl_vector *b, gsl_vector *c){
    double ax = gsl_vector_get (a, 0);
    double ay = gsl_vector_get (a, 1);
    double az = gsl_vector_get (a, 2);
    double bx = gsl_vector_get (b, 0);
    double by = gsl_vector_get (b, 1);
    double bz = gsl_vector_get (b, 2);

    double tmp = ay*bz - by*az;
    gsl_vector_set(c, 0, tmp);
    tmp = bx*az - ax*bz;
    gsl_vector_set(c, 1, tmp);
    tmp = ax*by - bx*ay;
    gsl_vector_set(c, 2, tmp);
}

void PlaneModel::GetTotalForceAndTotalMoments(){
    //moments
    double raero[3], rprop[3];
    for(int i=0; i<3; i++){
        raero[i] = 0;
        rprop[i] = 0;
    }
    for(int i=0; i<3; i++){
        raero[i] = rAC[i] - CGpos[i];
        rprop[i] = rHub[i] - CGpos[i];
    }

    gsl_vector *a, *b, *c, *d, *e, *f;
    a = gsl_vector_alloc(3);
    b = gsl_vector_alloc(3);
    c = gsl_vector_alloc(3);
    d = gsl_vector_alloc(3);
    e = gsl_vector_alloc(3);
    f = gsl_vector_alloc(3);

/*    gsl_vector_set(a, 0, raero[1]);
    gsl_vector_set(a, 1, raero[2]);
    gsl_vector_set(a, 2, raero[0]);
    gsl_vector_set(b, 0, Fz);
    gsl_vector_set(b, 1, Fx);
    gsl_vector_set(b, 2, Fy);
    gsl_vector_cross_mul(a, b, c);*/
    gsl_vector_set(c, 0, raero[1]*Faeroz);
    gsl_vector_set(c, 1, raero[2]*Faerox);
    gsl_vector_set(c, 2, raero[0]*Faeroy);

/*    gsl_vector_set(a, 0, raero[2]);
    gsl_vector_set(a, 1, raero[0]);
    gsl_vector_set(a, 2, raero[1]);
    gsl_vector_set(b, 0, Fy);
    gsl_vector_set(b, 1, Fz);
    gsl_vector_set(b, 2, Fx);
    gsl_vector_cross_mul(a, b, d);*/
    gsl_vector_set(d, 0, raero[2]*Faeroy);
    gsl_vector_set(d, 1, raero[0]*Faeroz);
    gsl_vector_set(d, 2, raero[1]*Faerox);

    gsl_vector_sub(c, d);

/*    gsl_vector_set(a, 0, rprop[1]);
    gsl_vector_set(a, 1, rprop[2]);
    gsl_vector_set(a, 2, rprop[0]);
    gsl_vector_set(b, 0, 0);
    gsl_vector_set(b, 1, Fprop);
    gsl_vector_set(b, 2, 0);
    gsl_vector_cross_mul(a, b, e);*/
    gsl_vector_set(e, 0, rprop[1]*0);
    gsl_vector_set(e, 1, rprop[2]*Fprop);
    gsl_vector_set(e, 2, rprop[0]*0);

/*    gsl_vector_set(a, 0, rprop[2]);
    gsl_vector_set(a, 1, rprop[0]);
    gsl_vector_set(a, 2, rprop[1]);
    gsl_vector_set(b, 0, 0);
    gsl_vector_set(b, 1, 0);
    gsl_vector_set(b, 2, Fprop);
    gsl_vector_cross_mul(a, b, f);*/
    gsl_vector_set(f, 0, rprop[2]*0);
    gsl_vector_set(f, 1, rprop[0]*0);
    gsl_vector_set(f, 2, rprop[1]*Fprop);

    gsl_vector_sub(e, f);

    double tmp1, tmp2;

    tmp1 = gsl_vector_get(c, 0);
    tmp2 = gsl_vector_get(e, 0);
    Mx = Maerox + Mprop + gsl_vector_get(c, 0) + gsl_vector_get(e, 0);
    //Mx = Maerox + Mprop;

    tmp1 = gsl_vector_get(c, 1);
    tmp2 = gsl_vector_get(e, 1);
    My = Maeroy + gsl_vector_get(c, 1) + gsl_vector_get(e, 1);
    //My = Maeroy;

    tmp1 = gsl_vector_get(c, 2);
    tmp2 = gsl_vector_get(e, 2);
    Mz = Maeroz + gsl_vector_get(c, 2) + gsl_vector_get(e, 2);
    //Mz = Maeroz;

    //forces
    Fx = Faerox + Fprop;
    Fy = Faeroy;
    Fz = Faeroz;

}

void PlaneModel::DifferentialEquation(double *dotX, double *X)
{

    double Ix = J[0], Iy = J[1], Iz = J[2], Ixz = J[3];
    double Txz2 = gsl_pow_2(Ixz);
    double SIGMAI = Ix * Iz - Txz2;
    double c1 = ((Iy-Iz)*Iz-Txz2)/SIGMAI;
    double c2 = (Ix-Iy+Iz)*Ixz/SIGMAI;
    double c3 = Iz/SIGMAI;
    double c4 = Ixz/SIGMAI;
    double c5 = (Iz-Ix)/Iy;
    double c6 = Ixz/Iy;
    double c7 = 1/Iy;
    double c8 = (Ix*(Ix-Iy)+Txz2)/SIGMAI;
    double c9 = Ix/SIGMAI;

    //X = {p,q,r,phi,theta,psi,u,v,w,posx,posy,posz,lat,lon}
    double p = X[0], q = X[1], r = X[2];
    double phi = X[3], theta = X[4], psi = X[5];
    double u = X[6], v = X[7], w = X[8];
    double posx = X[9], posy = X[10], posz = X[11];
    double ubody[3],uframe[3];
    ubody[0] = u; ubody[1] = v; ubody[2] = w;
    uframe[0] = 0.0; uframe[1] = 0.0; uframe[2] = 0.0;

    dotX[0] = (c1*r+c2*p)*q + c3*Mx + c4*Mz;//dotp
    dotX[1] = c5*p*r - c6*(gsl_pow_2(p)-gsl_pow_2(r)) + c7*My;//dotq
    dotX[2] = (c8*p-c2*r)*q + c4*Mx + c9*Mz;//dotr

    double vbody[3],vframe[3];
    vbody[0] = p;
    vbody[1] = q;
    vbody[2] = r;
    frame->Body2Frame(vbody, vframe);

    //dotX[3] = p + (r*cos(phi) + q*sin(phi))*tan(theta);//dotphi
    //dotX[4] = q*cos(phi) - r*sin(phi);//dottheta
    //dotX[5] = (r*cos(psi)+q*cos(psi))/cos(theta);//dotpsi
    dotX[3] = vframe[0];
    dotX[4] = vframe[1];
    dotX[5] = vframe[2];

    double ax = Fx/Mass;
    double ay = Fy/Mass;
    double az = Fz/Mass;

    double Gbody[3],Gframe[3];
    Gframe[0] = Gframe[1] = 0;Gframe[2] = G;
    frame->Frame2Body(Gframe, Gbody);

    //dotX[6] = v*r - w*q +G*sin(theta) + ax;//dotu
    //dotX[7] = -u*r + w*p + G*cos(theta)*sin(psi) + ay;//dotv
    //dotX[8] = u*q - v*p + G*cos(theta)*cos(psi) + az;//dotw
    dotX[6] = v*r - w*q +Gbody[0] + ax;//dotu
    dotX[7] = -u*r + w*p + Gbody[1] + ay;//dotv
    dotX[8] = u*q - v*p + Gbody[2] + az;//dotw

    frame->Body2Frame(ubody, uframe);

    dotX[9] = uframe[0];//dotposx
    dotX[10] = uframe[1];//dotposy
    dotX[11] = uframe[2];//dotposz

    dotX[12] = uframe[0]/(Rm+posz);//dotlat
    dotX[13] = uframe[1]/((Rn+posz)*cos(latitude));//dotlon

}

void PlaneModel::Solver(){

    double dT2, dT;
    double *K1 = new double[statesNumber];
    double *K2 = new double[statesNumber];
    double *K3 = new double[statesNumber];
    double *K4 = new double[statesNumber];
    double *Xlast = new double[statesNumber];
    double *X = new double[statesNumber];
    chrono::steady_clock::time_point tNow;

    static chrono::steady_clock::time_point timeStepLast1 = chrono::steady_clock::now();

    tNow= chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(tNow - timeStepLast1);
    dT = time_span.count();
    dT = 0.02;
    dT2 = dT/2.0;
    timeStepLast1 = tNow;
    simuTime += dT;

    X[0] = p;X[1] = q;X[2] = r;
    X[3] = phi;X[4] = theta;X[5] = psi;
    X[6] = u;X[7] = v;X[8] = w;
    X[9] = posn;X[10] = pose;X[11] = -h;
    X[12] = latitude, X[13] = longtitude;

    for (int i = 0; i < statesNumber; i++) {
        Xlast[i] = X[i];
    }

    DifferentialEquation(K1, X); // k1 = f(x)
    for (int i = 0; i < statesNumber; i++) {
        X[i] = Xlast[i] + dT2 * K1[i];
    }
    DifferentialEquation(K2, X); // k2 = f(x+0.5*dT*k1)
    for (int i = 0; i < statesNumber; i++) {
        X[i] = Xlast[i] + dT2 * K2[i];
    }
    DifferentialEquation(K3, X); // k3 = f(x+0.5*dT*k2)
    for (int i = 0; i < statesNumber; i++) {
        X[i] = Xlast[i] + dT * K3[i];
    }
    DifferentialEquation(K4, X); // k4 = f(x+dT*k3)

    // Xnew  = X + dT*(k1+2*k2+2*k3+k4)/6
    for (int i = 0; i < statesNumber; i++) {
        X[i] = Xlast[i] + dT * (K1[i] + 2.0 * K2[i] + 2.0 * K3[i] + K4[i]) / 6.0;
    }

    p = X[0];q = X[1];r = X[2];
    phi = X[3];theta = X[4];psi = X[5];
    u = X[6];v = X[7];w = X[8];

    double ubody[3],uframe[3];
    ubody[0] = u; ubody[1] = v; ubody[2] = w;
    uframe[0] = 0.0; uframe[1] = 0.0; uframe[2] = 0.0;
    frame->Body2Frame(ubody, uframe);
    vn = uframe[0];
    ve = uframe[1];
    vd = uframe[2];

    posn = X[9];pose = X[10];h = -X[11];
    latitude = X[12], longtitude = X[13];

    static int dd = 0;
    //tmp[dd] = X[6];
    dd++;

    //if(dd==1){
    if(simuTime>120){
        //dd--;
        isRun = false;
    }

    delete K1;
    delete K2;
    delete K3;
    delete K4;
    delete Xlast;
    delete X;
}

void PlaneModel::GetAlphaAndBeta(){
    double norm;
    double Vwindbody[3], Vwindframe[3];
    double aoa, sideslip;
    static double alphaLast=0,betaLast = 0;

    Vwindframe[0] = windx;
    Vwindframe[1] = windy;
    Vwindframe[2] = windz;

    frame->Frame2Body(Vwindframe, Vwindbody);
    u -= Vwindbody[0];
    v -= Vwindbody[1];
    w -= Vwindbody[2];

    norm = sqrt(gsl_pow_2(u) + gsl_pow_2(v) + gsl_pow_2(w));

    if(norm<10e-6)
        sideslip=0;
    else
        sideslip = asin(v/norm);

    if(fabs(u)<10e-6)
        aoa = 0;
    else
        aoa = atan(w/u);

    V = norm;
    beta = sideslip;//-pi/2~pi/2
    alpha = aoa;//-pi/2~pi/2

    dotalpha = alpha-alphaLast;
    dotbeta = beta-betaLast;

    alphaLast = alpha;
    betaLast = beta;

}

void PlaneModel::SetCtrller(double *ctrller){


    da = -ctrller[0]>(20*M_PI/180)?(20*M_PI/180):(-ctrller[0]<(-20*M_PI/180)?(-20*M_PI/180):-ctrller[0]);
    de = -ctrller[1]>(20*M_PI/180)?(20*M_PI/180):(-ctrller[1]<(-20*M_PI/180)?(-20*M_PI/180):-ctrller[1]);
    dr = -ctrller[2]>(20*M_PI/180)?(20*M_PI/180):(-ctrller[2]<(-20*M_PI/180)?(-20*M_PI/180):-ctrller[2]);
    thr = ctrller[3];
    de = ctrller[1];

    //dm->SetPlotData(psi*180/M_PI);

}

void PlaneModel::InitialPlane(){

    phi = theta = 0;
    psi =0*M_PI/180;
    p = q = r = 0;
    alpha = beta = 0;
    S = 0.55;
    b = 2.8956;
    MAC = 0.1889941;
    da = dr = 0; de = -0.15;
    V = u = 23;
    v = w = 0;
    vn = ve = vd = 0;
    mempty = 8.5; // kg
    mgross = 13.5; // kg
    CGempty[0]=0.156; CGempty[1]=0; CGempty[2]=0.079; // m, [x y z]
    CGgross[0] = 0.159; CGgross[1] = 0; CGgross[2] = 0.090; // m, [x y z]
    Jempty[0] = 0.7795; Jempty[1] = 1.122; Jempty[2] = 1.752; Jempty[3] = 0.1211; // kg*m^2, [Jx Jy Jz Jxz]
    Jgross[0] = 0.8244; Jgross[1] = 1.135; Jgross[2] = 1.759; Jgross[3] = 0.1204; // kg*m^2, [Jx Jy Jz Jxz]
    rHub[0] = rHub[1] = rHub[2] = 0;
    rAC[0] = 0.1425;rAC[1] = rAC[2] = 0;
    G = 9.81;
    Rm=Rn=6378137;
    latitude= 34.5135*M_PI/180;
    longtitude = 113.8582*M_PI/180;

    h = 1000;
    posn = pose = 0;
    windx=windy=windz = 0;

    CL0 = 0.23;
    CLa = 5.6106;
    CLdf = 0.74;
    CLde = 0.13;
    CLalphadot = 1.9724;
    CLq = 7.9543;
    CLmind = 0.23;
    CD0 = 0.0434;
    CDdf = 0.1467;
    CDde = 0.0135;
    CDda = 0.0302;
    CDdr = 0.0303;
    osw = 0.75;
    CYbeta = -0.83;
    CYda = -0.075;
    CYdr = 0.1914;
    Cm0 = 0.135;
    Cma = -2.7397;
    Cmdf = 0.0467;
    Cmde = -0.9918;
    Cmalphadot = -10.3796;
    Cmq = -38.2067;
    Clbeta = -0.13;
    Clda = -0.1695;
    Cldr = 0.0024;
    Clp = -0.5051;
    Clr = 0.2519;
    Cnbeta = 0.0726;
    Cnda = 0.0108;
    Cndr = -0.0693;
    Cnp = -0.069;
    Cnr = -0.0946;

    prop->initialPropusion();
    eaa->initialEAA();

    simuTime = 0.0;
    tmp = new QStringList();
}

void PlaneModel::RunModel(){

    if(isRun){
        frame->UpdateRbe();
        frame->UpdateRbw();

        GetAlphaAndBeta();

        GetParameters();

        GetAeroForcesAndMoments();

        GetInertia();

        GetTotalForceAndTotalMoments();

        Solver();//rk4
    }
}

}
