#include "rudpsend.h"
#include "time.h"

RUdpSend::RUdpSend(QObject *parent) : QObject(parent)
{
    sendSocket1 = new QUdpSocket();
    sendSocket2 = new QUdpSocket();
    sendSocket3 = new QUdpSocket();
    sendSocket4 = new QUdpSocket();

    port1 = 5511;
    port2 = 5512;
    port3 = 5513;
    port4 = 5514;

    beat=0;

    address.setAddress("127.0.0.1");

    dm = DataManager::instance();
}

void RUdpSend::writeChannel(){
    plane p[4];
    dm->getPlanesData(p);
    char buff[1192];
    char buff1[408];
    plane p1 = p[1];
    //makePackage(p1,buff);
    packageSendFg(buff1,p1);
    sendSocket1->writeDatagram(buff1, 408,address,port1);

    p1 = p[2];
    if(makePackage(p1,buff,1))
    sendSocket2->writeDatagram(buff, 1192,address,port2);

    p1 = p[3];
    if(makePackage(p1,buff,2))
    sendSocket3->writeDatagram(buff, 1192,address,port2);

    p1 = p[0];
    if(makePackage(p1,buff,3)){
    //address.setAddress("10.60.1.62");
    sendSocket4->writeDatagram(buff, 1192,address,port2);
}
    beat++;
}

#define _EQURAD 6378137.0
#define _FLATTENING 298.257223563

#define _SQUASH    0.9966471893352525192801545
#define _STRETCH   1.0033640898209764189003079
#define _POLRAD    6356752.3142451794975639668

#define E2 fabs(1 - _SQUASH*_SQUASH)
static double a = _EQURAD;
//static double ra2 = 1/(_EQURAD*_EQURAD);
//static double e = sqrt(E2);
static double e2 = E2;
//static double e4 = E2*E2;

const double R2D = 57.29577951308232087680;
const double D2R =  0.01745329251994329577;

void SGGeodToCart(double * lla, double * cart)
{
  //      double m_pi = 3.1415926;
  // according to
  // H. Vermeille,
  // Direct transformation from geocentric to geodetic ccordinates,
  // Journal of Geodesy (2002) 76:451-454
  double lambda = lla[0]*D2R;
  double phi = lla[1]*D2R;
  double h = lla[2];
  double sphi = sin(phi);
  double n = a/sqrt(1-e2*sphi*sphi);
  double cphi = cos(phi);
  double slambda = sin(lambda);
  double clambda = cos(lambda);
  cart[0] = (h+n)*cphi*clambda;
  cart[1] = (h+n)*cphi*slambda;
  cart[2] = (h+n-e2*n)*sphi;
}

void fromLonLatRad( float lon, float lat, float * q){

    float zd2 = (0.5)*lon*D2R;
    float yd2 = (-0.25)*M_PI - (0.5)*lat*D2R;
   // float yd2 = (0.5)*lat*D2R;
    float Szd2 = sin(zd2);
    float Syd2 = sin(yd2);
    float Czd2 = cos(zd2);
    float Cyd2 = cos(yd2);
    q[0] = Czd2*Cyd2;
    q[1] = -Szd2*Syd2;
    q[2] = Czd2*Syd2;
    q[3] = Szd2*Cyd2;

    float nrm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    q[0] /= nrm;
    q[1] /= nrm;
    q[2] /= nrm;
    q[3] /= nrm;
};

void fromYawPitchRoll(float z, float y, float x, float *q1){

    float zd2 = float(0.5)*z; float yd2 = float(0.5)*y; float xd2 = float(0.5)*x;
    float Szd2 = sin(zd2); float Syd2 = sin(yd2); float Sxd2 = sin(xd2);
    float Czd2 = cos(zd2); float Cyd2 = cos(yd2); float Cxd2 = cos(xd2);
    float Cxd2Czd2 = Cxd2*Czd2; float Cxd2Szd2 = Cxd2*Szd2;
    float Sxd2Szd2 = Sxd2*Szd2; float Sxd2Czd2 = Sxd2*Czd2;
    q1[0] = Cxd2Czd2*Cyd2 + Sxd2Szd2*Syd2;
    q1[1] = Sxd2Czd2*Cyd2 - Cxd2Szd2*Syd2;
    q1[2] = Cxd2Czd2*Syd2 + Sxd2Szd2*Cyd2;
    q1[3] = Cxd2Szd2*Cyd2 - Sxd2Czd2*Syd2;

    float nrm = sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]+q1[3]*q1[3]);
    q1[0] /= nrm;
    q1[1] /= nrm;
    q1[2] /= nrm;
    q1[3] /= nrm;
};

void quatMul(float *q, float *q1){
  float tempq[4];
  tempq[0] = q[0];tempq[1] = q[1];tempq[2] = q[2];tempq[3] = q[3];

  q[1] = tempq[0]*q1[1] + tempq[1]*q1[0] + tempq[2]*q1[3] - tempq[3]*q1[2];
  q[2] = tempq[0]*q1[2] - tempq[1]*q1[3] + tempq[2]*q1[0] + tempq[3]*q1[1];
  q[3] = tempq[0]*q1[3] + tempq[1]*q1[2] - tempq[2]*q1[1] + tempq[3]*q1[0];
  q[0] = tempq[0]*q1[0] - tempq[1]*q1[1] - tempq[2]*q1[2] - tempq[3]*q1[3];

  float nrm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] /= nrm;
  q[1] /= nrm;
  q[2] /= nrm;
  q[3] /= nrm;
};

void Quaternion2RPY(float *q, float *rpy)
{
    float R13, R11, R12, R23, R33;
    float q0s = q[0] * q[0];
    float q1s = q[1] * q[1];
    float q2s = q[2] * q[2];
    float q3s = q[3] * q[3];

    R13    = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R11    = q0s + q1s - q2s - q3s;
    R12    = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R23    = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    R33    = q0s - q1s - q2s + q3s;

    rpy[1] = R2D*asinf(-R13); // pitch always between -pi/2 to pi/2
    rpy[2] = R2D*atan2f(R12, R11);
    rpy[0] = R2D*atan2f(R23, R33);

    // TODO: consider the cases where |R13| ~= 1, |pitch| ~= pi/2
};

  void getAngleAxis(float * q, float * axis)
  {

          float tempq[4] ,angle;
        tempq[0] = q[0];tempq[1] = q[1];tempq[2] = q[2];tempq[3] = q[3];

    float nrm = sqrt(tempq[0]*tempq[0]+tempq[1]*tempq[1]+tempq[2]*tempq[2]+tempq[3]*tempq[3]);

    if (nrm <= 0) {
      angle = 0;
      //axis = SGVec3<T>(0, 0, 0);
      axis[0] = axis[2] = axis[1] = 0;
    } else {
      float rNrm = 1/nrm;
      float s = rNrm*tempq[0];
      s = s>1?1:(s<(-1)?(-1):s);
      angle = acos(s);
      float sAng = sin(angle);
      if (fabs(sAng) <= 0){
        //axis = SGVec3<T>(1, 0, 0);
          axis[0] = 1;
          axis[1] = 0;
          axis[2] = 0;
      }
      else{
        //axis = (rNrm/sAng)*imag(*this);
          axis[0] = (rNrm/sAng)*tempq[1];
          axis[1] = (rNrm/sAng)*tempq[2];
          axis[2] = (rNrm/sAng)*tempq[3];
      }
      angle *= 2;
    }

    axis[0] *= angle;
    axis[1] *= angle;
    axis[2] *= angle;

  };

void reverseInt(unsigned int * in){

        union tmpf{
        unsigned int x;
                unsigned char byte[4];
    } ;

        tmpf tmp; tmp.x= *in;
        tmpf tmpR;

        for(int i=0;i<4;i++)
                tmpR.byte[3-i] = tmp.byte[i];

        *in = tmpR.x;

}

void reverseFloat(float * in){

        union tmpf{
        float x;
                unsigned char byte[4];
    } ;

        tmpf tmp; tmp.x = *in;
        tmpf tmpR;

        for(int i=0;i<4;i++)
                tmpR.byte[3-i] = tmp.byte[i];

        *in= tmpR.x;

}

void reverseDouble(double * in){

        union tmpf{
        double x;
                unsigned char byte[8];
    } ;

        tmpf tmp; tmp.x = *in;
        tmpf tmpR;

        for(int i=0;i<8;i++)
                tmpR.byte[7-i] = tmp.byte[i];

        *in = tmpR.x;

}

bool RUdpSend::makePackage(plane p, char *buffer, int index){

    T_Msg pack;

    union tmpf{
        unsigned int x;
        float f;
        unsigned char byte[4];
    } ;

    union tmpb{
        unsigned long long x;
        double f;
        unsigned char byte[8];
    } ;

    tmpf magic;
    magic.byte[0]= 0x46;
    magic.byte[1]= 0x47;
    magic.byte[2]= 0x46;
    magic.byte[3]= 0x53;
    pack.msg.header.Magic = magic.x;

    tmpf version;
    version.byte[0]=0x00;
    version.byte[1]=0x01;
    version.byte[2]=0x00;
    version.byte[3]=0x01;
    pack.msg.header.Version = version.x;

    tmpf MsgId;
    MsgId.x=0;
    MsgId.byte[3] = 0x07;
    pack.msg.header.MsgId = MsgId.x;

    pack.msg.header.MsgLen = 1192;
    reverseInt(&(pack.msg.header.MsgLen));

    pack.msg.header.ReplyAddress = pack.msg.header.ReplyPort = 0;

    pack.msg.header.Callsign[0] = 0x74;
    pack.msg.header.Callsign[1] = 0x65;
    pack.msg.header.Callsign[2] = 0x73;
    pack.msg.header.Callsign[3] = 0x74;
    switch(index){
    case 1:
        pack.msg.header.Callsign[4] = 0x32;

        break;
    case 2:
        pack.msg.header.Callsign[4] = 0x33;

        break;
    case 3:
        pack.msg.header.Callsign[4] = 0x34;

        break;
    default:
        pack.msg.header.Callsign[4] = 0x00;
        break;
    }
    //pack.msg.header.Callsign[4] = ;
            pack.msg.header.Callsign[5] = pack.msg.header.Callsign[6] = pack.msg.header.Callsign[7] = 0;

    pack.msg.pos.Model[0] = 0x41;
    pack.msg.pos.Model[1] = 0x69;
    pack.msg.pos.Model[2] = 0x72;
    pack.msg.pos.Model[3] = 0x63;
    pack.msg.pos.Model[4] = 0x72;
    pack.msg.pos.Model[5] = 0x61;
    pack.msg.pos.Model[6] = 0x66;
    pack.msg.pos.Model[7] = 0x74;
    pack.msg.pos.Model[8] = 0x2f;
    pack.msg.pos.Model[9] = 0x63;
    pack.msg.pos.Model[10] = 0x31;
    pack.msg.pos.Model[11] = 0x37;
    pack.msg.pos.Model[12] = 0x32;
    pack.msg.pos.Model[13] = 0x70;
    pack.msg.pos.Model[14] = 0x2f;
    pack.msg.pos.Model[15] = 0x4d;
    pack.msg.pos.Model[16] = 0x6f;
    pack.msg.pos.Model[17] = 0x64;
    pack.msg.pos.Model[18] = 0x65;
    pack.msg.pos.Model[19] = 0x6c;
    pack.msg.pos.Model[20] = 0x73;
    pack.msg.pos.Model[21] = 0x2f;
    pack.msg.pos.Model[22] = 0x63;
    pack.msg.pos.Model[23] = 0x31;
    pack.msg.pos.Model[24] = 0x37;
    pack.msg.pos.Model[25] = 0x32;
    pack.msg.pos.Model[26] = 0x70;
    pack.msg.pos.Model[27] = 0x2e;
    pack.msg.pos.Model[28] = 0x78;
    pack.msg.pos.Model[29] = 0x6d;
    pack.msg.pos.Model[30] = 0x6c;

    for(int i=31; i<96;i++)
        pack.msg.pos.Model[i] = 0x00;

    //static int beat=0;

    tmpb tmp;
    if(beat>30)
        tmp.f = (double)(beat*0.1);
    else if(beat==0){
           tmp.f = (double)(0.0f);
    }
    else if(beat==10){
           tmp.f = (double)(1.0f);
    }
    else if(beat==20){
           tmp.f = (double)(2.0f);
    }
    else if(beat==30){
           tmp.f = (double)(3.0f);
    }else{
        return false;
    }

    reverseDouble(&(tmp.f));
    pack.msg.pos.time = tmp.x;

    tmp.f = (double)(0.1);
    reverseDouble(&(tmp.f));
    pack.msg.pos.lag = tmp.x;

    double lla[3], cart[3];

    //[0] = pDlg->m_model.sif.longitude*R2D;
    //lla[1] = pDlg->m_model.sif.latitude*R2D;
    //lla[2] = pDlg->m_model.sif.altitude;
    lla[0] = p.lon;
    lla[1] = p.lat;
    lla[2] = p.alt;

    //lla[0] = -122.125;
    //lla[1] = 37.5625;
    //lla[2] = 100;

    SGGeodToCart(&lla[0], &cart[0]);

        //cart[0] = -2707406.8213;KSFO
        //cart[1] = -4273244.2458;
        //cart[2] = 3871554.9481;
    //-2618739.13222247 -4346528.25697968 3852688.70485603
    //cart[0] = -2128068.44453905;//zhcc-2131102.19981803
    //cart[1] = 4811732.02593855;//4809507.51028363
    //cart[2] = 3593612.16200090+500; //3594868.03289749

    tmpb pos;
    pos.f = cart[0];
    reverseDouble(&(pos.f));
    pack.msg.pos.position[0] = pos.x;
    pos.f = cart[1];
    reverseDouble(&(pos.f));
    pack.msg.pos.position[1] = pos.x;
    pos.f = cart[2];
    reverseDouble(&(pos.f));
    pack.msg.pos.position[2] = pos.x;

    float rpy[3], q[4],q1[4],angleAxis[3];
    angleAxis[0] = angleAxis[1] =angleAxis[2] =0;

    //rpy[0] = pDlg->m_model.sif.phi*D2R;
    //rpy[1] = -pDlg->m_model.sif.theta*D2R;
    //rpy[2] = pDlg->m_model.sif.psi*D2R;
    rpy[0] = p.roll*D2R;
    rpy[1] = p.pitch*D2R;
    //rpy[2] = p.yaw*D2R>M_PI?(p.yaw*D2R-2*M_PI):p.yaw*D2R;
    rpy[2] = p.yaw*D2R;

    fromLonLatRad(lla[0], lla[1], &q[0]);
    fromYawPitchRoll(rpy[2], rpy[1], rpy[0], &q1[0]);
    quatMul(&q[0], &q1[0]);

    getAngleAxis(&q[0], &angleAxis[0]);
    //Quaternion2RPY(&q[0], &rpy[0]);

    //rpy[0] = 1.39971220;
    //rpy[1] = 2.41185451;
    //rpy[2] = 1.37070906;

    tmpb ati;
    ati.f = angleAxis[0];
    reverseDouble(&(ati.f));
    pack.msg.pos.orientation[0] = ati.x;

    ati.f = angleAxis[1];
    reverseDouble(&(ati.f));
    pack.msg.pos.orientation[1] = ati.x;
    ati.f = angleAxis[2];
    reverseDouble(&(ati.f));
    pack.msg.pos.orientation[2] = ati.x;

    pack.msg.pos.linearAccel[0] = 0;
    pack.msg.pos.linearAccel[1] = 0;
    pack.msg.pos.linearAccel[2] = 0;
    pack.msg.pos.angularAccel[0] = 0;
    pack.msg.pos.angularAccel[1] = 0;
    pack.msg.pos.angularAccel[2] = 0;
    pack.msg.pos.linearVel[0] = 0;
    pack.msg.pos.linearVel[1] = 0;
    pack.msg.pos.linearVel[2] = 0;
    pack.msg.pos.angularVel[0] = 0;
    pack.msg.pos.angularVel[1] = 0;
    pack.msg.pos.angularVel[2] = 0;
    pack.msg.pos.pad=0;

    pack.msg.tail.p1101Index = 1101;
    reverseInt(&(pack.msg.tail.p1101Index));
    for(int i=0; i<17*4;i++)
        pack.msg.tail.p1101[i] = 0;
    //00 00 00 10 00 00 00 34 00 00 00 58 00 00 00 2D 00 00 00 43 00 00 00 48 00 00 00 56 00 00 00 5F 00 00 00 48 00 00 00 44 00 00 00 5F 00 00 00 6C 00 00 00 69 00 00 00 76 00 00 00 65 00 00 00 72 00 00 00 79
    pack.msg.tail.p1101[3] = 0x10;
    pack.msg.tail.p1101[3+4] = 0x34;
    pack.msg.tail.p1101[3+4*2] = 0x58;
    pack.msg.tail.p1101[3+4*3] = 0x2d;
    pack.msg.tail.p1101[3+4*4] = 0x43;
    pack.msg.tail.p1101[3+4*5] = 0x48;
    pack.msg.tail.p1101[3+4*6] = 0x56;
    pack.msg.tail.p1101[3+4*7] = 0x5f;
    pack.msg.tail.p1101[3+4*8] = 0x48;
    pack.msg.tail.p1101[3+4*9] = 0x44;
    pack.msg.tail.p1101[3+4*10] = 0x5f;
    pack.msg.tail.p1101[3+4*11] = 0x6c;
    pack.msg.tail.p1101[3+4*12] = 0x69;
    pack.msg.tail.p1101[3+4*13] = 0x76;
    pack.msg.tail.p1101[3+4*14] = 0x65;
    pack.msg.tail.p1101[3+4*15] = 0x72;
    pack.msg.tail.p1101[3+4*16] = 0x79;

    pack.msg.tail.p10001Index = 10001;
    reverseInt(&(pack.msg.tail.p10001Index));
    for(int i=0; i<17*4;i++)
        pack.msg.tail.p10001[i] = 0;
    //00 00 00 09 00 00 00 31 00 00 00 31 00 00 00 38 00 00 00 35 00 00 00 30 00 00 00 30 00 00 00 30 00 00 00 30 00 00 00 30 00 00 00 00 00 00 00 00 00 00 00 00
    pack.msg.tail.p10001[3] = 0x09;
    pack.msg.tail.p10001[3+4] = 0x31;
    pack.msg.tail.p10001[3+4*2] = 0x31;
    pack.msg.tail.p10001[3+4*3] = 0x38;
    pack.msg.tail.p10001[3+4*4] = 0x35;
    pack.msg.tail.p10001[3+4*5] = 0x30;
    pack.msg.tail.p10001[3+4*6] = 0x30;
    pack.msg.tail.p10001[3+4*7] = 0x30;
    pack.msg.tail.p10001[3+4*8] = 0x30;
    pack.msg.tail.p10001[3+4*9] = 0x30;

    pack.msg.tail.p10002Index = 10002;
    reverseInt(&(pack.msg.tail.p10002Index));
    for(int i=0; i<9*4;i++)
        pack.msg.tail.p10002[i] = 0;
    //00 00 00 05 00 00 00 48 00 00 00 65 00 00 00 6C 00 00 00 6C 00 00 00 6F 00 00 00 00 00 00 00 00 00 00 00 00
    pack.msg.tail.p10002[3] = 0x05;
    pack.msg.tail.p10002[3+4] = 0x48;
    pack.msg.tail.p10002[3+4*2] = 0x65;
    pack.msg.tail.p10002[3+4*3] = 0x6c;
    pack.msg.tail.p10002[3+4*4] = 0x6c;
    pack.msg.tail.p10002[3+4*5] = 0x6f;

    pack.msg.tail.p100.index = 100;
    reverseInt(&(pack.msg.tail.p100.index));
    pack.msg.tail.p100.value.f = 0.0269949;
    reverseFloat(&(pack.msg.tail.p100.value.f));


    pack.msg.tail.p101.index = 101;
    reverseInt(&(pack.msg.tail.p101.index));
    pack.msg.tail.p101.value.f =  0.0269949;
    reverseFloat(&(pack.msg.tail.p101.value.f));

    pack.msg.tail.p102.index = 102;
    reverseInt(&(pack.msg.tail.p102.index));
    pack.msg.tail.p102.value.x = 0;
    pack.msg.tail.p103.index = 103;
    reverseInt(&(pack.msg.tail.p103.index));
    pack.msg.tail.p103.value.x = 0;
    pack.msg.tail.p104.index = 104;
    reverseInt(&(pack.msg.tail.p104.index));
    pack.msg.tail.p104.value.x = 0;
    pack.msg.tail.p105.index = 105;
    reverseInt(&(pack.msg.tail.p105.index));
    pack.msg.tail.p105.value.x = 0;
    pack.msg.tail.p106.index = 106;
    reverseInt(&(pack.msg.tail.p106.index));
    pack.msg.tail.p106.value.x = 0;
    //pack.msg.tail.p107.index = 107;
    //reverseInt(&(pack.msg.tail.p107.index));
    //pack.msg.tail.p107.value.x = 0;
    pack.msg.tail.p108.index = 108;
    reverseInt(&(pack.msg.tail.p108.index));
    pack.msg.tail.p108.value.x = 0;
    //pack.msg.tail.p109.index = 109;
    //reverseInt(&(pack.msg.tail.p109.index));
    //pack.msg.tail.p109.value.x = 0;
    //pack.msg.tail.p110.index = 110;
    //reverseInt(&(pack.msg.tail.p110.index));
    //pack.msg.tail.p110.value.x = 0;
    //pack.msg.tail.p111.index = 111;
    //reverseInt(&(pack.msg.tail.p111.index));
    //pack.msg.tail.p111.value.x = 0;
    pack.msg.tail.p112.index = 112;
    reverseInt(&(pack.msg.tail.p112.index));
    pack.msg.tail.p112.value.x = 0;

    pack.msg.tail.p200.index = 200;
    reverseInt(&(pack.msg.tail.p200.index));
    pack.msg.tail.p200.value.f = 0.231995;
    reverseFloat(&(pack.msg.tail.p200.value.f));

    pack.msg.tail.p201.index = 201;
    reverseInt(&(pack.msg.tail.p201.index));
    pack.msg.tail.p201.value.f = 1.0f;
    reverseFloat(&(pack.msg.tail.p201.value.f));

    pack.msg.tail.p210.index = 210;
    reverseInt(&(pack.msg.tail.p210.index));
    pack.msg.tail.p210.value.f = 0.143935;
    reverseFloat(&(pack.msg.tail.p210.value.f));

    pack.msg.tail.p211.index = 211;
    reverseInt(&(pack.msg.tail.p211.index));
    pack.msg.tail.p211.value.f = 1.0f;
    reverseFloat(&(pack.msg.tail.p211.value.f));
    pack.msg.tail.p220.index = 220;
    reverseInt(&(pack.msg.tail.p220.index));
    pack.msg.tail.p220.value.f = 0.128129;
    reverseFloat(&(pack.msg.tail.p220.value.f));
    pack.msg.tail.p221.index = 221;
    reverseInt(&(pack.msg.tail.p221.index));
    pack.msg.tail.p221.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p221.value.f));
    pack.msg.tail.p230.index = 230;
    reverseInt(&(pack.msg.tail.p230.index));
    pack.msg.tail.p230.value.f = 0.0;
    pack.msg.tail.p231.index = 231;
    reverseInt(&(pack.msg.tail.p231.index));
    pack.msg.tail.p231.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p231.value.f));

    pack.msg.tail.p240.index = 240;
    reverseInt(&(pack.msg.tail.p240.index));
    pack.msg.tail.p240.value.f = 0.0;
    pack.msg.tail.p241.index = 241;
    reverseInt(&(pack.msg.tail.p241.index));
    pack.msg.tail.p241.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p241.value.f));

    pack.msg.tail.p302.index = 302;
    reverseInt(&(pack.msg.tail.p302.index));
    pack.msg.tail.p302.value.f = 0.0;
    pack.msg.tail.p312.index = 312;
    reverseInt(&(pack.msg.tail.p312.index));
    pack.msg.tail.p312.value.f = 0.0;
    pack.msg.tail.p322.index = 322;
    reverseInt(&(pack.msg.tail.p322.index));
    pack.msg.tail.p322.value.f = 0.0;
    pack.msg.tail.p330.index = 330;
    reverseInt(&(pack.msg.tail.p330.index));
    pack.msg.tail.p330.value.f = 0.0;
    pack.msg.tail.p331.index = 331;
    reverseInt(&(pack.msg.tail.p331.index));
    pack.msg.tail.p331.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p331.value.f));
    pack.msg.tail.p332.index = 332;
    reverseInt(&(pack.msg.tail.p332.index));
    pack.msg.tail.p332.value.f = 0.0;
    pack.msg.tail.p340.index = 340;
    reverseInt(&(pack.msg.tail.p340.index));
    pack.msg.tail.p340.value.f = 0.0;
    pack.msg.tail.p341.index = 341;
    reverseInt(&(pack.msg.tail.p341.index));
    pack.msg.tail.p341.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p341.value.f));
    pack.msg.tail.p342.index = 342;
    reverseInt(&(pack.msg.tail.p342.index));
    pack.msg.tail.p342.value.f = 0.0;
    pack.msg.tail.p350.index = 350;
    reverseInt(&(pack.msg.tail.p350.index));
    pack.msg.tail.p350.value.f = 0.0;
    pack.msg.tail.p351.index = 351;
    reverseInt(&(pack.msg.tail.p351.index));
    pack.msg.tail.p351.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p351.value.f));
    pack.msg.tail.p352.index = 352;
    reverseInt(&(pack.msg.tail.p352.index));
    pack.msg.tail.p352.value.f = 0.0;
    pack.msg.tail.p360.index = 360;
    reverseInt(&(pack.msg.tail.p360.index));
    pack.msg.tail.p360.value.f = 0.0;
    pack.msg.tail.p361.index = 361;
    reverseInt(&(pack.msg.tail.p361.index));
    pack.msg.tail.p361.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p361.value.f));
    pack.msg.tail.p362.index = 362;
    reverseInt(&(pack.msg.tail.p362.index));
    pack.msg.tail.p362.value.f = 0.0;

    pack.msg.tail.p370.index = 370;
    reverseInt(&(pack.msg.tail.p370.index));
    pack.msg.tail.p370.value.f = 0.0;
    pack.msg.tail.p371.index = 371;
    reverseInt(&(pack.msg.tail.p371.index));
    pack.msg.tail.p371.value.f = 0.0;
    pack.msg.tail.p372.index = 372;
    reverseInt(&(pack.msg.tail.p372.index));
    pack.msg.tail.p372.value.f = 0.0;

    pack.msg.tail.p380.index = 380;
    reverseInt(&(pack.msg.tail.p380.index));
    pack.msg.tail.p380.value.f = 0.0;
    pack.msg.tail.p381.index = 381;
    reverseInt(&(pack.msg.tail.p381.index));
    pack.msg.tail.p381.value.f = 0.0;
    pack.msg.tail.p382.index = 382;
    reverseInt(&(pack.msg.tail.p382.index));
    pack.msg.tail.p382.value.f = 0.0;
        pack.msg.tail.p390.index = 390;
        reverseInt(&(pack.msg.tail.p390.index));
    pack.msg.tail.p390.value.f = 0.0;
    pack.msg.tail.p391.index = 391;
    reverseInt(&(pack.msg.tail.p391.index));
    pack.msg.tail.p391.value.f = 0.0;
    pack.msg.tail.p392.index = 392;
    reverseInt(&(pack.msg.tail.p392.index));
    pack.msg.tail.p392.value.f = 0.0;

    pack.msg.tail.p800.index = 800;
    reverseInt(&(pack.msg.tail.p800.index));
    pack.msg.tail.p800.value.f = 2.40887;
    reverseFloat(&(pack.msg.tail.p800.value.f));
    pack.msg.tail.p801.index = 801;
    reverseInt(&(pack.msg.tail.p801.index));
    pack.msg.tail.p801.value.f = 1.03802;
    reverseFloat(&(pack.msg.tail.p801.value.f));
    pack.msg.tail.p810.index = 810;
    reverseInt(&(pack.msg.tail.p810.index));
    pack.msg.tail.p810.value.f = 0;

    pack.msg.tail.p811.index = 811;
    reverseInt(&(pack.msg.tail.p811.index));
    pack.msg.tail.p811.value.f = 0;
    pack.msg.tail.p812.index = 812;
    reverseInt(&(pack.msg.tail.p812.index));
    pack.msg.tail.p812.value.f = 0;

    pack.msg.tail.p813.index = 813;
    reverseInt(&(pack.msg.tail.p813.index));
    pack.msg.tail.p813.value.f = 3.99108;
    reverseFloat(&(pack.msg.tail.p813.value.f));

    pack.msg.tail.p820.index = 820;
    reverseInt(&(pack.msg.tail.p820.index));
    pack.msg.tail.p820.value.f = 355.951;
    reverseFloat(&(pack.msg.tail.p820.value.f));

    pack.msg.tail.p821.index = 821;
    reverseInt(&(pack.msg.tail.p821.index));
    pack.msg.tail.p821.value.f = -40.9398;
    reverseFloat(&(pack.msg.tail.p821.value.f));
    pack.msg.tail.p822.index = 822;
    reverseInt(&(pack.msg.tail.p822.index));
    pack.msg.tail.p822.value.f = 2.38724;
    reverseFloat(&(pack.msg.tail.p822.value.f));
    pack.msg.tail.p823.index = 823;
    reverseInt(&(pack.msg.tail.p823.index));
    pack.msg.tail.p823.value.f = -44.1575;
    reverseFloat(&(pack.msg.tail.p823.value.f));
    pack.msg.tail.p830.index = 830;
    reverseInt(&(pack.msg.tail.p830.index));
    pack.msg.tail.p830.value.f = -178.55;
    reverseFloat(&(pack.msg.tail.p830.value.f));
    pack.msg.tail.p831.index = 831;
    reverseInt(&(pack.msg.tail.p831.index));
    pack.msg.tail.p831.value.f = -40.5728;
    reverseFloat(&(pack.msg.tail.p831.value.f));

    pack.msg.tail.p1001.index = 1001;
    reverseInt(&(pack.msg.tail.p1001.index));
    pack.msg.tail.p1001.value.f = 0;
    pack.msg.tail.p1002.index = 1002;
    reverseInt(&(pack.msg.tail.p1002.index));
    pack.msg.tail.p1002.value.f = 0;
    pack.msg.tail.p1003.index = 1003;
    reverseInt(&(pack.msg.tail.p1003.index));
    pack.msg.tail.p1003.value.f = 0;
    pack.msg.tail.p1004.index = 1004;
    reverseInt(&(pack.msg.tail.p1004.index));
    pack.msg.tail.p1004.value.f = 1.0;
    reverseFloat(&(pack.msg.tail.p1004.value.f));
    pack.msg.tail.p1005.index = 1005;
    reverseInt(&(pack.msg.tail.p1005.index));
    pack.msg.tail.p1005.value.f = 0.0;
    pack.msg.tail.p1006.index = 1006;
    reverseInt(&(pack.msg.tail.p1006.index));
    pack.msg.tail.p1006.value.f = 0.0;

    pack.msg.tail.p1100.index = 1100;
    reverseInt(&(pack.msg.tail.p1100.index));
    pack.msg.tail.p1100.value.x = 0.0;

    pack.msg.tail.p1200.index = 1200;
    reverseInt(&(pack.msg.tail.p1200.index));
    pack.msg.tail.p1200.value.f = 0.0;
    pack.msg.tail.p1201.index = 1201;
    reverseInt(&(pack.msg.tail.p1201.index));
    pack.msg.tail.p1201.value.x = 0.0;

    pack.msg.tail.p1400.index = 1400;
    reverseInt(&(pack.msg.tail.p1400.index));
    pack.msg.tail.p1400.value.f = 0.0;
    pack.msg.tail.p1500.index = 1500;
    reverseInt(&(pack.msg.tail.p1500.index));
    pack.msg.tail.p1500.value.x = 0.0;
    pack.msg.tail.p1501.index = 1501;
    reverseInt(&(pack.msg.tail.p1501.index));
    pack.msg.tail.p1501.value.x = 0.0;
    pack.msg.tail.p1502.index = 1502;
    reverseInt(&(pack.msg.tail.p1502.index));
    pack.msg.tail.p1502.value.x = 0.0;
    pack.msg.tail.p1503.index = 1503;
    reverseInt(&(pack.msg.tail.p1503.index));
    pack.msg.tail.p1503.value.x = 1;
    reverseInt(&(pack.msg.tail.p1503.value.x));

    pack.msg.tail.p10100.index = 10100;
    reverseInt(&(pack.msg.tail.p10100.index));
    pack.msg.tail.p10100.value.x = 0;

    pack.msg.tail.p10200.index = 10200;
    reverseInt(&(pack.msg.tail.p10200.index));
    pack.msg.tail.p10200.value.x = 0;
    pack.msg.tail.p10201.index = 10201;
    reverseInt(&(pack.msg.tail.p10201.index));
    pack.msg.tail.p10201.value.x = 0;
    pack.msg.tail.p10202.index = 10202;
    reverseInt(&(pack.msg.tail.p10202.index));
    pack.msg.tail.p10202.value.x = 0;
    pack.msg.tail.p10203.index = 10203;
    reverseInt(&(pack.msg.tail.p10203.index));
    pack.msg.tail.p10203.value.x = 0;
    pack.msg.tail.p10204.index = 10204;
    reverseInt(&(pack.msg.tail.p10204.index));
    pack.msg.tail.p10204.value.x = 0;
    pack.msg.tail.p10205.index = 10205;
    reverseInt(&(pack.msg.tail.p10205.index));
    pack.msg.tail.p10205.value.x = 0;
    pack.msg.tail.p10206.index = 10206;
    reverseInt(&(pack.msg.tail.p10206.index));
    pack.msg.tail.p10206.value.x = 0;
    pack.msg.tail.p10207.index = 10207;
    reverseInt(&(pack.msg.tail.p10207.index));
    pack.msg.tail.p10207.value.x = 0;
    pack.msg.tail.p10208.index = 10208;
    reverseInt(&(pack.msg.tail.p10208.index));
    pack.msg.tail.p10208.value.x = 0;
    pack.msg.tail.p10209.index = 10209;
    reverseInt(&(pack.msg.tail.p10209.index));
    pack.msg.tail.p10209.value.x = 0;
    pack.msg.tail.p10210.index = 10210;
    reverseInt(&(pack.msg.tail.p10210.index));
    pack.msg.tail.p10210.value.x = 0;
    pack.msg.tail.p10211.index = 10211;
    reverseInt(&(pack.msg.tail.p10211.index));
    pack.msg.tail.p10211.value.x = 0;
    pack.msg.tail.p10212.index = 10212;
    reverseInt(&(pack.msg.tail.p10212.index));
    pack.msg.tail.p10212.value.x = 0;
    pack.msg.tail.p10213.index = 10213;
    reverseInt(&(pack.msg.tail.p10213.index));
    pack.msg.tail.p10213.value.x = 0;
    pack.msg.tail.p10214.index = 10214;
    reverseInt(&(pack.msg.tail.p10214.index));
    pack.msg.tail.p10214.value.x = 0;
    pack.msg.tail.p10215.index = 10215;
    reverseInt(&(pack.msg.tail.p10215.index));
    pack.msg.tail.p10215.value.x = 0;
    pack.msg.tail.p10216.index = 10216;
    reverseInt(&(pack.msg.tail.p10216.index));
    pack.msg.tail.p10216.value.x = 0;
    pack.msg.tail.p10217.index = 10217;
    reverseInt(&(pack.msg.tail.p10217.index));
    pack.msg.tail.p10217.value.x = 0;

    pack.msg.tail.p10303.index = 10303;
    reverseInt(&(pack.msg.tail.p10303.index));
    pack.msg.tail.p10303.value.x = 0;
    pack.msg.tail.p10304.index = 10304;
    reverseInt(&(pack.msg.tail.p10304.index));
    pack.msg.tail.p10304.value.x = 0;
    pack.msg.tail.p10305.index = 10305;
    reverseInt(&(pack.msg.tail.p10305.index));
    pack.msg.tail.p10305.value.x = 0;
    pack.msg.tail.p10306.index = 10306;
    reverseInt(&(pack.msg.tail.p10306.index));
    pack.msg.tail.p10306.value.x = 0;
    pack.msg.tail.p10307.index = 10307;
    reverseInt(&(pack.msg.tail.p10307.index));
    pack.msg.tail.p10307.value.x = 0;
    pack.msg.tail.p10308.index = 10308;
    reverseInt(&(pack.msg.tail.p10308.index));
    pack.msg.tail.p10308.value.x = 0;
    pack.msg.tail.p10309.index = 10309;
    reverseInt(&(pack.msg.tail.p10309.index));
    pack.msg.tail.p10309.value.x = 0;
    pack.msg.tail.p10310.index = 10310;
    reverseInt(&(pack.msg.tail.p10310.index));
    pack.msg.tail.p10310.value.x = 0;
    pack.msg.tail.p10311.index = 10311;
    reverseInt(&(pack.msg.tail.p10311.index));
    pack.msg.tail.p10311.value.x = 0;
    pack.msg.tail.p10312.index = 10312;
    reverseInt(&(pack.msg.tail.p10312.index));
    pack.msg.tail.p10312.value.x = 0;
    pack.msg.tail.p10313.index = 10313;
    reverseInt(&(pack.msg.tail.p10313.index));
    pack.msg.tail.p10313.value.x = 0;

    memcpy(buffer, pack.buff, 1192);
    return true;
};

void RUdpSend::packageSendFg(char *buffer, plane p1){
    PACKSEND pack;

    //static double lon_old = 0;
    //static double lat_old = 0;
    //double p = 0.9;
    //pack.fdm.version = 24;
    pack.fdm.padding = 0;

    pack.fdm.longitude = p1.lon*D2R;
    //pack.fdm.longitude = 1.98720059;
    //113.8582
    pack.fdm.latitude = p1.lat*D2R;
    //pack.fdm.latitude = 0.60237463;
    //34.5135
    pack.fdm.altitude = p1.alt;
    //pack.fdm.altitude = 248.10795613;
    pack.fdm.agl = 0;
    pack.fdm.phi = p1.roll*D2R;
    pack.fdm.theta = p1.pitch*D2R;
    pack.fdm.psi = p1.yaw*D2R;
    pack.fdm.alpha = 0*D2R;
    pack.fdm.beta = 0*D2R;
    pack.fdm.phidot = 0;
    pack.fdm.thetadot = 0;
    pack.fdm.psidot = 0;
    pack.fdm.vcas = 0;
    pack.fdm.climb_rate = 0;
    //pack.fdm.v_north = pDlg->m_model.sif.vnorth*fps2ms;
    //pack.fdm.v_east = pDlg->m_model.sif.veast*fps2ms;
    //pack.fdm.v_down = pDlg->m_model.sif.vdown*fps2ms;
    pack.fdm.v_north = 0;
    pack.fdm.v_east = 0;
    pack.fdm.v_down = 0;
    //pack.fdm.v_wind_body_north = pDlg->m_model.sif.vwind_n*fps2ms;
    //pack.fdm.v_wind_body_east = pDlg->m_model.sif.vwind_e*fps2ms;
    //pack.fdm.v_wind_body_down = pDlg->m_model.sif.vwind_d*fps2ms;
    pack.fdm.v_wind_body_north = 0;
    pack.fdm.v_wind_body_east = 0;
    pack.fdm.v_wind_body_down = 0;
    //pack.fdm.A_X_pilot = pDlg->m_model.sif.accx*fps2ms2;
    //pack.fdm.A_Y_pilot = pDlg->m_model.sif.accy*fps2ms2;
    //pack.fdm.A_Z_pilot = pDlg->m_model.sif.accz*fps2ms2;
    pack.fdm.A_X_pilot = 0;
    pack.fdm.A_Y_pilot = 0;
    pack.fdm.A_Z_pilot = 0;
    pack.fdm.stall_warning = 0;
    pack.fdm.slip_deg = 0;

    pack.fdm.num_engines = 1;
    pack.fdm.eng_state[0] = pack.fdm.eng_state[1] = pack.fdm.eng_state[2] = pack.fdm.eng_state[3] =0;
    pack.fdm.rpm[0] = pack.fdm.rpm[1] = pack.fdm.rpm[2] = pack.fdm.rpm[3] = 8000;   //maybe something could happen
    pack.fdm.fuel_flow[0] = pack.fdm.fuel_flow[1] = pack.fdm.fuel_flow[2] = pack.fdm.fuel_flow[3] = 0;
    pack.fdm.fuel_px[0] = pack.fdm.fuel_px[1] = pack.fdm.fuel_px[2] = pack.fdm.fuel_px[3] = 0;
    pack.fdm.egt[0] = pack.fdm.egt[1] = pack.fdm.egt[2] = pack.fdm.egt[3] = 0;
    pack.fdm.cht[0] = pack.fdm.cht[1] = pack.fdm.cht[2] = pack.fdm.cht[3] = 0;
    pack.fdm.mp_osi[0] = pack.fdm.mp_osi[1] = pack.fdm.mp_osi[2] = pack.fdm.mp_osi[3] = 0;
    pack.fdm.tit[0] = pack.fdm.tit[1] = pack.fdm.tit[2] = pack.fdm.tit[3] = 0;
    pack.fdm.oil_temp[0] = pack.fdm.oil_temp[1] = pack.fdm.oil_temp[2] = pack.fdm.oil_temp[3] = 0;
    pack.fdm.oil_px[0] = pack.fdm.oil_px[1] = pack.fdm.oil_px[2] = pack.fdm.oil_px[3] = 0;
    pack.fdm.num_tanks = 0;
    pack.fdm.fuel_quantity[0] = pack.fdm.fuel_quantity[1] = pack.fdm.fuel_quantity[2] = pack.fdm.fuel_quantity[3] = 0;
    pack.fdm.num_wheels = 0;
    pack.fdm.wow[0] = pack.fdm.wow[1] = pack.fdm.wow[2] = 0;
    pack.fdm.gear_pos[0] = pack.fdm.gear_pos[1] = pack.fdm.gear_pos[2] = 0;
    pack.fdm.gear_steer[0] = pack.fdm.gear_steer[1] = pack.fdm.gear_steer[2] = 0;
    pack.fdm.gear_compression[0] = pack.fdm.gear_compression[1] = pack.fdm.gear_compression[2] = 0;

    time_t tick;
    struct tm tmHere;
    tick = time(NULL);
    tmHere = *localtime(&tick);
    pack.fdm.cur_time = tick;
    pack.fdm.warp = 7380;
    pack.fdm.visibility = 3200;

    //pack.fdm.cur_time = 0;
    //pack.fdm.warp = 0;
    //pack.fdm.visibility = 0;

    pack.fdm.elevator = 0 + 3;
    pack.fdm.elevator_trim_tab = 0;
    pack.fdm.left_flap = pack.fdm.right_flap = 0;
    pack.fdm.left_aileron = 0;
    pack.fdm.right_aileron = 0;
    pack.fdm.rudder = 0;
    pack.fdm.nose_wheel = 0;
    pack.fdm.speedbrake = 0;
    pack.fdm.spoilers = 0;

    //buffer = pack.buff;

//	htond(pack.fdm.longitude);
//    htond(pack.fdm.longitude);
//    htond(pack.fdm.longitude);

    //ntohl(pack.fdm.version);

    //char buffer[408];
    int j=0;
    //char temp[4];
    for(int i=0; i<408; i++){
        if(i%4==0){
            j++;
            if(j==3||j==4){
                if(j==3){
                    buffer[i] = pack.buff[i+7];
                    buffer[i+1] = pack.buff[i+6];
                    buffer[i+2] = pack.buff[i+5];
                    buffer[i+3] = pack.buff[i+4];
                }else{
                    buffer[i] = pack.buff[i+3];
                    buffer[i+1] = pack.buff[i+2];
                    buffer[i+2] = pack.buff[i+1];
                    buffer[i+3] = pack.buff[i+0];
                }

            }else if(j==5||j==6){
                if(j==5){
                    buffer[i] = pack.buff[i+7];
                    buffer[i+1] = pack.buff[i+6];
                    buffer[i+2] = pack.buff[i+5];
                    buffer[i+3] = pack.buff[i+4];
                }else{
                    buffer[i] = pack.buff[i+3];
                    buffer[i+1] = pack.buff[i+2];
                    buffer[i+2] = pack.buff[i+1];
                    buffer[i+3] = pack.buff[i+0];
                }
            }else if(j==7||j==8){
                if(j==7){
                    buffer[i] = pack.buff[i+7];
                    buffer[i+1] = pack.buff[i+6];
                    buffer[i+2] = pack.buff[i+5];
                    buffer[i+3] = pack.buff[i+4];
                }else{
                    buffer[i] = pack.buff[i+3];
                    buffer[i+1] = pack.buff[i+2];
                    buffer[i+2] = pack.buff[i+1];
                    buffer[i+3] = pack.buff[i+0];
                }
            }else{
                buffer[i] = pack.buff[i+3];
                buffer[i+1] = pack.buff[i+2];
                buffer[i+2] = pack.buff[i+1];
                buffer[i+3] = pack.buff[i];
            }

        }
    }

    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 24;

    //memcpy(buff->data(), buffer, 1192);

}
