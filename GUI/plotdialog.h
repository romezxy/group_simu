#ifndef PLOTDIALOG_H
#define PLOTDIALOG_H

#include <QWidget>
#include <QDateTime>
#include <QMutex>
#include <QVBoxLayout>
#include "qwt/qwt.h"
#include "qwt/qwt_plot.h"
#include "qwt/qwt_plot_curve.h"
#include "qwt/qwt_scale_draw.h"
#include "qwt/qwt_scale_widget.h"
#include "data/datamanager.h"

namespace Ui {
class PlotDialog;
}

class TimeScaleDraw : public QwtScaleDraw {
public:
    TimeScaleDraw() {}
    virtual QwtText label(double v) const
    {
        uint seconds     = (uint)(v);
        QDateTime upTime = QDateTime::fromTime_t(seconds);
        QTime timePart   = upTime.time().addMSecs((v - seconds) * 1000);

        upTime.setTime(timePart);
        return upTime.toLocalTime().toString("hh:mm:ss");
    }
};

class ScopeLocal : public QwtPlot
{
    Q_OBJECT

public:
    explicit ScopeLocal(QwtPlot *parent = 0);
//    scope();
    int m_xWindowSize;
    void addCurvePlot(QString curveNameScaled, QVector<double> *xData, QVector<double> *yData,QPen pen);
    void preparePlot(bool timeScale);
    void PlotData(double data);
    void PlotData2d(double xdata, double ydata);

private slots:
    void showCurve(QwtPlotItem *item, bool on);
    void savePic();
    void legendChecked( const QVariant &itemInfo, bool on );
    void doRePlot();

private:
    void deleteLegend();
    void addLegend();
    void replotNewData();
    void clearCurvePlots();
    void removeStaleData(QByteArray *xData, QByteArray *yData);
    void removeStaleData(QVector<double> *xData);

    QMutex mutex;
    bool antialiased;
    QMap<QString, QwtPlotCurve *> m_curvesData;
    QMap<QString, QVector<double> *> m_localData;
    bool timeScale;
    QVector<double> *m_xData;
    QVector<double> *m_yData;

    QVector<double> *m_data, *m_time;

};

class PlotDialog : public QWidget
{
    Q_OBJECT

public:
    explicit PlotDialog(QWidget *parent = 0);
    ~PlotDialog();    

public slots:
    void AddPlotData();

private:
    Ui::PlotDialog *ui;
    ScopeLocal *plot;
    DataManager *dm;
};

#endif // PLOTDIALOG_H
