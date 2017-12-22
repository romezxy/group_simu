#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>

#include "data/datamanager.h"
#include "io/rudpread.h"
#include "io/rudpsend.h"
#include "plane/planemodel.h"
#include "GNC/controller.h"
#include "GNC/guidance.h"
#include "config/config.h"
#include "GUI/plotdialog.h"
#include <QVBoxLayout>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void time();

private:
    Ui::MainWindow *ui;

    RUdpRead *read;
    RUdpSend *send;

    QThread *thread1;
    QThread *thread2;

    DataManager *dm;
    QTimer *timer;
    planesimulator::PlaneModel *plane;
    GNC::guidance *gui;
    GNC::Controller *ctrller;
    PlotDialog *scope;

};

#endif // MAINWINDOW_H
