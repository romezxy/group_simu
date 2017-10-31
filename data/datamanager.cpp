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
