#include "logSystem.h"

logSystem *logSystem::m_instance = 0;

logSystem::logSystem(QObject *parent) : QObject(parent)
{

}

logSystem *logSystem::instance(){
    return m_instance;
}

void logSystem::addError(int errorIndex, int errorClass, QString details){

}

void logSystem::addLog(QString details){

}
