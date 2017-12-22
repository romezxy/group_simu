#include "datamanager.h"

DataManager *DataManager::m_instance = 0;

DataManager::DataManager(QObject *parent) : QObject(parent)
{
    m_instance = this;
    p1.lat = 34.7568711;
    p1.lon = 113.663221;
    p2.lat = 34.7568711;
    p2.lon = 113.663221;
    p3.lat = 34.7568711;
    p3.lon = 113.663221;
    p4.lat = 34.7568711;
    p4.lon = 113.663221;
}

DataManager *DataManager::instance(){
    return m_instance;
}

void DataManager::getPlanesData(plane p[4]){
    p[0] = p1;
    p[1] = p2;
    p[2] = p3;
    p[3] = p4;
}

void DataManager::setPlanes(int index, plane p){
    switch(index){
    case 1:
        p1 = p;
        break;
    case 2:
        p2 = p;
        break;
    case 3:
        p3 = p;
        break;
    case 4:
        p4 = p;
        break;
    }
}

void DataManager::SetPlotData(double data){
    plotdata = data;
}
void DataManager::GetPlotData(double *data){
    *data = plotdata;
}

void DataManager::SetPlotData2d(double xdata, double ydata){
    plotxdata = xdata;
    plotydata = ydata;
}
void DataManager::GetPlotData2d(double *xdata, double *ydata){
    *xdata = plotxdata;
    *ydata = plotydata;
}

void DataManager::CrossProduct(float v1[3], float v2[3], float result[3]){
    result[0] = v1[1] * v2[2] - v2[1] * v1[2];
    result[1] = v2[0] * v1[2] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v2[0] * v1[1];
}

double DataManager::boundData(double data, double min, double max){
    return data>max?(max):(data<min?(min):(data));
}
float DataManager::boundData(float data, float min, float max){
    return data>max?(max):(data<min?(min):(data));
}
