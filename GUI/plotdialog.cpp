#include "plotdialog.h"
#include "ui_plotdialog.h"
#include "qwt/qwt_plot_curve.h"
#include "qwt/qwt_legend.h"
#include "qwt/qwt_legend_label.h"
#include "qwt/qwt_legend_data.h"
#include "qwt/qwt_plot_grid.h"
#include "qwt/qwt_symbol.h"
#include "qwt/qwt_plot_zoomer.h"
#include "qwt/qwt_plot_panner.h"
#include "qwt/qwt_plot_renderer.h"
#include "qwt/qwt_plot_item.h"
#include "qwt/qwt_plot_marker.h"

PlotDialog::PlotDialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlotDialog)
{
    ui->setupUi(this);

    QVBoxLayout *layout = new QVBoxLayout(this);
    plot = new ScopeLocal();
    //plot->preparePlot(true);
    layout->addWidget(plot);

    this->setLayout(layout);

    dm = DataManager::instance();
}

PlotDialog::~PlotDialog()
{
    delete ui;
}

void PlotDialog::AddPlotData(){
    double data;

    dm->GetPlotData(&data);

    plot->PlotData(data);
}


ScopeLocal::ScopeLocal(QwtPlot *parent): QwtPlot(parent)
{
    m_xData = new QVector<double>();
    m_yData = new QVector<double>();
    antialiased = true;

    timeScale = true;
    m_xWindowSize = 5000;

    m_data = new QVector<double>();
    m_time = new QVector<double>();

    preparePlot(true);
}

void ScopeLocal::clearCurvePlots()
{
    foreach(QwtPlotCurve * curve, m_curvesData.values()) {
        curve->detach();

        delete curve;
    }

    m_curvesData.clear();
}

void ScopeLocal::deleteLegend()
{
    if (!legend()) {
        return;
    }

    disconnect(this, SIGNAL(legendChecked(QwtPlotItem *, bool)), this, 0);

    insertLegend(NULL, QwtPlot::TopLegend);
// insertLegend(NULL, QwtPlot::ExternalLegend);
}

void ScopeLocal::addLegend()
{
    if (legend()) {
        return;
    }

    QwtLegend *legend = new QwtLegend();//曲线描述
    legend->setDefaultItemMode( QwtLegendData::Checkable );//设置描述是QCheckBox类型
    legend->setFrameStyle(QFrame::Box | QFrame::Sunken);
    legend->setToolTip(tr("Click legend to show/hide scope trace"));

    QPalette pal = legend->palette();
    pal.setColor(legend->backgroundRole(), QColor(100, 100, 100)); // background colour
    pal.setColor(QPalette::Text, QColor(0, 0, 0)); // text colour
    legend->setPalette(pal);

    insertLegend(legend, QwtPlot::LeftLegend);

    connect( legend, SIGNAL( checked( const QVariant &, bool, int ) ),
        SLOT( legendChecked( const QVariant &, bool ) ) );
}

void ScopeLocal::legendChecked( const QVariant &itemInfo, bool on )
{
    QwtPlotItem *plotItem = infoToItem( itemInfo );
    if ( plotItem )
        showCurve( plotItem, on );
}

void ScopeLocal::showCurve(QwtPlotItem *item, bool on)
{

    item->setVisible( on );

    QwtLegend *lgd = qobject_cast<QwtLegend *>( legend() );

    QList<QWidget *> legendWidgets =
        lgd->legendWidgets( itemToInfo( item ) );

    if ( legendWidgets.size() == 1 )
    {
        QwtLegendLabel *legendLabel =
            qobject_cast<QwtLegendLabel *>( legendWidgets[0] );

        if ( legendLabel )
            legendLabel->setChecked( on );
    }

    replot();
}

void ScopeLocal::preparePlot(bool timeScale)
{
    clearCurvePlots();

    setMinimumSize(64, 64);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    setCanvasBackground(QColor(10, 10, 10));

    QwtPlotGrid *grid = new QwtPlotGrid;
    grid->setMajorPen(QPen(Qt::gray, 0, Qt::DashLine));
    grid->setMinorPen(QPen(Qt::lightGray, 0, Qt::DotLine));
    grid->setPen(QPen(Qt::darkGray, 1, Qt::DotLine));
    grid->attach(this);

    addLegend();

    if(timeScale){
        setAxisScaleDraw(QwtPlot::xBottom, new TimeScaleDraw());
    }else{
        setAxisScaleDraw(QwtPlot::xBottom, new QwtScaleDraw());
    }

//    setAxisScale(QwtPlot::xBottom, 0, m_xWindowSize);
//    uint NOW = QDateTime::currentDateTime().toTime_t();
//    setAxisScale(QwtPlot::xBottom, NOW - m_xWindowSize, NOW);
    setAxisLabelRotation(QwtPlot::xBottom, 0.0);
    setAxisLabelAlignment(QwtPlot::xBottom, Qt::AlignLeft | Qt::AlignBottom);

    QwtScaleWidget *scaleWidget = axisWidget(QwtPlot::xBottom);

    scaleWidget->setMargin(0);

    QFont fnt(axisFont(QwtPlot::xBottom));
    fnt.setPointSize(7);
    setAxisFont(QwtPlot::xBottom, fnt);
    setAxisFont(QwtPlot::yLeft, fnt);

    int r=255,g=255,b=255;
    QPen *pen = new QPen(QBrush(QColor(r,g,b), Qt::SolidPattern),
                               (qreal)1,
                               Qt::SolidLine,
                               Qt::SquareCap,
                               Qt::BevelJoin);

    addCurvePlot("yaw",m_time, m_data, *pen);

}

void ScopeLocal::addCurvePlot(QString curveNameScaled, QVector<double> *xData, QVector<double> *yData,QPen pen)
{


 //   setAxisScale(QwtPlot::xBottom, minX*0.9, maxX*1.1);
 //   setAxisScale(QwtPlot::yLeft, minY*0.9, maxY*1.1);
    QwtPlotCurve *plotCurve = new QwtPlotCurve(curveNameScaled);

    QwtSymbol *symbol=new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::yellow),pen,QSize(5,5));
    plotCurve->setSymbol(symbol);

    if (antialiased) {
        plotCurve->setRenderHint(QwtPlotCurve::RenderAntialiased);
    }
    plotCurve->setPen(pen);
    plotCurve->setSamples(*xData, *yData);
    plotCurve->attach(this);
    plotCurve->setVisible(false);

    QwtPlotZoomer* zoomer = new QwtPlotZoomer(this->canvas());
    zoomer->setRubberBandPen( QColor( Qt::black ) );
    zoomer->setTrackerPen( QColor( Qt::black ) );
    zoomer->setMousePattern(QwtEventPattern::MouseSelect2,Qt::RightButton, Qt::ControlModifier );
    zoomer->setMousePattern(QwtEventPattern::MouseSelect3,Qt::RightButton );

    m_curvesData.insert(curveNameScaled, plotCurve);
    m_localData.insert(curveNameScaled, yData);

    mutex.lock();
    replot();
    mutex.unlock();
}

void ScopeLocal::savePic(){
    QwtPlotRenderer renderer;
    renderer.exportTo(this, "ls.jpg", QSizeF(500,500),85);
}

void ScopeLocal::doRePlot(){
//    plotCurve->setSamples(*m_xData, *m_yData);

    mutex.lock();
    replot();
    mutex.unlock();
}

void ScopeLocal::PlotData(double data){

    QDateTime NOW = QDateTime::currentDateTime();
    double valueX = NOW.toTime_t() + NOW.time().msec() / 1000.0;

    //add data here
    m_data->append(data);
    m_time->append(valueX);

    removeStaleData(m_data);
    removeStaleData(m_time);

    foreach(QString  name1, m_curvesData.keys()) {
        foreach(QString  name2, m_localData.keys()){
            if(name1==name2){
                QwtPlotCurve * curve = m_curvesData.value(name1);
                QVector<double> *data = m_localData.value(name2);
                curve->setSamples(*m_time,*data);
                continue;
            }
        }
    }

    replot();
}

void ScopeLocal::removeStaleData(QVector<double> *xData)
{
    while (1) {
        if (xData->size() == 0) {
            break;
        }
        if (xData->count() > m_xWindowSize){
            xData->pop_front();
        }else{
            break;
        }
    }
}
