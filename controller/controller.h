#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include "data/datamanager.h"

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(QObject *parent = nullptr);

signals:

public slots:

private:
    DataManager *dm;
};

#endif // CONTROLLER_H
