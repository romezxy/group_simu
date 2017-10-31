#include "controller.h"

Controller::Controller(QObject *parent) : QObject(parent)
{
    dm = DataManager::instance();
}
