#ifndef RUDPSEND_H
#define RUDPSEND_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>

#include "data/datamanager.h"
#include "io/net_fdm.h"

class RUdpSend : public QObject
{
    Q_OBJECT
public:
    explicit RUdpSend(QObject *parent = 0);

signals:

public slots:
    void writeChannel();//other thread;

private:
    QUdpSocket *sendSocket1;
    QUdpSocket *sendSocket2;
    QUdpSocket *sendSocket3;
    QUdpSocket *sendSocket4;

    int port1;
    int port2;
    int port3;
    int port4;

    int beat;

    QHostAddress address;

    DataManager *dm;

    bool makePackage(plane p, char *buffer,int index);
    void packageSendFg(char *buffer, plane p);
};

#endif // RUDPSEND_H
