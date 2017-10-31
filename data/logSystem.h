#ifndef LOGSYSTEM_H
#define LOGSYSTEM_H

#include <QObject>

class logSystem: public QObject
{
    Q_OBJECT
public:
    explicit logSystem(QObject *parent = 0);
    static logSystem *instance();

//public slots:


    //errorIndex: 错误序列号，在错误类中的唯一标识，不同类中序列号可重复
    //errorClass: 错误类，表征错误的级别，目前分1，2，3级。
    //            1级最重要，影响系统运行，直接崩溃的，能记录就记录，实在不能记录也没办法
    //            2级一般重要，系统能勉强运行，如果不手工清除一直提醒
    //            3级不重要，不影响系统运行，影响界面美观之类的，提醒一次即可
    //details：错误描述，可空。
    //2.1 egm96 error
    //2.2 wmm file cant find
    //2.3 wmm Allocate failed
    //2.4 atmosphere file cant find
    void addError(int errorIndex, int errorClass, QString details="");

    //details：添加想添加的log，不可为空，要不你要说啥。
    void addLog(QString details);

private:
    static logSystem *m_instance;

};

#endif // ERRORSYSTEM_H
