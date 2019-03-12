#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "AddonShieldCAN.h"
#include <QTimer>

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
    void on_cbCh2_stateChanged(int arg1);

    void on_cbCh3_stateChanged(int arg1);

    void on_cbCh4_stateChanged(int arg1);

    void on_pwm2_freq_valueChanged(int value);

    void on_pwm2_duty_valueChanged(int value);

    void on_cb12V_stateChanged(int arg1);

    void on_cb19V_stateChanged(int arg1);

    void on_pwm3_freq_valueChanged(int value);

    void on_pwm3_duty_valueChanged(int value);

    void on_pwm4_freq_valueChanged(int value);

    void on_pwm4_duty_valueChanged(int value);

    void on_timer_update();

private:
    Ui::MainWindow *ui;

    AddonShieldCAN* _shield;
    SocketCAN*      _can;
    QTimer*         _timer;

    float _freq2;
    float _duty2;
    int   _duty3;
    int   _duty4;
};

#endif // MAINWINDOW_H
