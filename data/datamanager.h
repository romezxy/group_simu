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

    void SetPlotData(double data);
    void GetPlotData(double *data);
    void SetPlotData2d(double xdata, double ydata);
    void GetPlotData2d(double *xdata, double *ydata);

    void CrossProduct(float v1[3], float v2[3], float result[3]);
    double boundData(double data, double min, double max);
    float boundData(float data, float min, float max);

signals:

public slots:
private:
    static DataManager *m_instance;
    plane p1;
    plane p2;
    plane p3;
    plane p4;

    double plotdata;
    double plotxdata, plotydata;
};

#endif // DATAMANAGER_H
