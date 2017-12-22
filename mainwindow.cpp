#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QVBoxLayout *layout = new QVBoxLayout();

    dm = new DataManager(this);
    read = new RUdpRead(this);
    send = new RUdpSend(this);
    timer = new QTimer(this);
    plane = new planesimulator::PlaneModel(this);
    gui = new GNC::guidance(this,plane);
    ctrller = new GNC::Controller(this,plane,gui);
    scope = new PlotDialog();

    layout->addWidget(scope);
    ui->centralWidget->setLayout(layout);

    thread1 = new QThread();
    thread2 = new QThread();
    read->moveToThread(thread1);
    send->moveToThread(thread2);
    thread1->start();
    thread2->start();

    connect(timer, SIGNAL(timeout()), send, SLOT(writeChannel()));    
    connect(timer, SIGNAL(timeout()), this, SLOT(time()));
    connect(timer, SIGNAL(timeout()), ctrller, SLOT(RunController()));
    connect(ctrller,SIGNAL(ControllerDone()),plane,SLOT(RunModel()));
    connect(timer, SIGNAL(timeout()), scope, SLOT(AddPlotData()));

    plane->InitialPlane();
    ctrller->InitialCtrller();

    timer->setInterval(0.02*1000);
    timer->start();

    //plane = new PlaneModel();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::time(){
    if(!plane->isRun)
        timer->stop();
}
