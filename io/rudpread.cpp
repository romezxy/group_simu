#include "rudpread.h"
#include <QDataStream>

RUdpRead::RUdpRead(QObject *parent) : QObject(parent)
{
    readSocket1 = new QUdpSocket();
    readSocket2 = new QUdpSocket();
    readSocket3 = new QUdpSocket();
    readSocket4 = new QUdpSocket();

    port1 = 6000;//60开头的端口，用于高级集群计算，不是编队
    port2 = 6001;
    port3 = 6002;
    port4 = 6003;

    readSocket1->bind(port1);
    readSocket2->bind(port2);
    readSocket3->bind(port3);
    readSocket4->bind(port4);

    address.setAddress("127.0.0.1");

    dm = DataManager::instance();

    connect(readSocket1, SIGNAL(readyRead()), this, SLOT(readData1()));
    connect(readSocket2, SIGNAL(readyRead()), this, SLOT(readData2()));
    connect(readSocket3, SIGNAL(readyRead()), this, SLOT(readData3()));
    connect(readSocket4, SIGNAL(readyRead()), this, SLOT(readData4()));

}

typedef struct{
    plane d;
    char buffer[36];
}PlaneData;

void RUdpRead::readData1(){
    QByteArray datagram;
    do{
        datagram.resize(readSocket1->pendingDatagramSize());
        readSocket1->readDatagram(datagram.data(),datagram.size());
    }while(readSocket1->hasPendingDatagrams());

    plane p;
    int lat,lon,alt,roll,pitch,yaw;
    QDataStream in(&datagram,QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_5_7);
    //in>>p.lon>>p.lat>>p.alt>>p.roll>>p.pitch>>p.yaw;
    in>>lon>>lat>>alt>>roll>>pitch>>yaw;
    p.lon = lon/10000000.0;
    p.lat = lat/10000000.0;
    p.alt = alt/100.0;
    p.roll = roll/100.0;
    p.pitch = pitch/100.0;
    p.yaw = yaw/100.0;

/*    PlaneData in;

    for(int i=0; i<36; i++){
        in.buffer[i] = datagram.data()[i];
    }
    p = in.d;
    */

    dm->setPlanes(1,p);
}
void RUdpRead::readData2(){

    QByteArray datagram;
    do{
        datagram.resize(readSocket2->pendingDatagramSize());
        readSocket2->readDatagram(datagram.data(),datagram.size());
    }while(readSocket2->hasPendingDatagrams());

    plane p;
    int lat,lon,alt,roll,pitch,yaw;
    QDataStream in(&datagram,QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_4_8);
    //in>>p.lon>>p.lat>>p.alt>>p.roll>>p.pitch>>p.yaw;
    in>>lon>>lat>>alt>>roll>>pitch>>yaw;
    p.lon = lon/10000000.0;
    p.lat = lat/10000000.0;
    p.alt = alt/100.0;
    p.roll = roll/100.0;
    p.pitch = pitch/100.0;
    p.yaw = yaw/100.0;

    dm->setPlanes(2,p);

}
void RUdpRead::readData3(){

    QByteArray datagram;
    do{
        datagram.resize(readSocket3->pendingDatagramSize());
        readSocket3->readDatagram(datagram.data(),datagram.size());
    }while(readSocket3->hasPendingDatagrams());

    plane p;
    int lat,lon,alt,roll,pitch,yaw;
    QDataStream in(&datagram,QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_4_8);
    //in>>p.lon>>p.lat>>p.alt>>p.roll>>p.pitch>>p.yaw;
    in>>lon>>lat>>alt>>roll>>pitch>>yaw;
    p.lon = lon/10000000.0;
    p.lat = lat/10000000.0;
    p.alt = alt/100.0;
    p.roll = roll/100.0;
    p.pitch = pitch/100.0;
    p.yaw = yaw/100.0;

    dm->setPlanes(3,p);

}
void RUdpRead::readData4(){

    QByteArray datagram;
    do{
        datagram.resize(readSocket4->pendingDatagramSize());
        readSocket4->readDatagram(datagram.data(),datagram.size());
    }while(readSocket4->hasPendingDatagrams());

    plane p;
    int lat,lon,alt,roll,pitch,yaw;
    QDataStream in(&datagram,QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_4_8);
    //in>>p.lon>>p.lat>>p.alt>>p.roll>>p.pitch>>p.yaw;
    in>>lon>>lat>>alt>>roll>>pitch>>yaw;
    p.lon = lon/10000000.0;
    p.lat = lat/10000000.0;
    p.alt = alt/100.0;
    p.roll = roll/100.0;
    p.pitch = pitch/100.0;
    p.yaw = yaw/100.0;

    dm->setPlanes(4,p);

}
