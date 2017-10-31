#ifndef RUDPREAD_H
#define RUDPREAD_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>

#include "data/datamanager.h"

class RUdpRead : public QObject
{
    Q_OBJECT
public:
    explicit RUdpRead(QObject *parent = 0);

signals:

public slots:
    void readData1();
    void readData2();
    void readData3();
    void readData4();

private:
    QUdpSocket *readSocket1;
    QUdpSocket *readSocket2;
    QUdpSocket *readSocket3;
    QUdpSocket *readSocket4;

    int port1;
    int port2;
    int port3;
    int port4;

    QHostAddress address;

    DataManager *dm;
};

#endif // RUDPREAD_H
