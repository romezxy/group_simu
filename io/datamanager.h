#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QObject>

//#pragma pack(2)
typedef struct{
   double lat;
   double lon;
   double alt;
   double roll;
   double pitch;
   double yaw;
}plane;
//}__attribute__((packed)) plane;

class DataManager : public QObject
{
    Q_OBJECT
public:
    explicit DataManager(QObject *parent = 0);
    static DataManager *instance();

    void getPlanesData(plane p[4]);
    void setPlanes(int index, plane p);

signals:

public slots:
private:
    static DataManager *m_instance;
    plane p1;
    plane p2;
    plane p3;
    plane p4;
};

#endif // DATAMANAGER_H
