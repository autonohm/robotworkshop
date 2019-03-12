#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDesktopWidget>
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->move(QApplication::desktop()->screen()->rect().center()-this->rect().center());

    _can = new SocketCAN(std::string("slcan0"));
    _can->startListener();
    _shield = new AddonShieldCAN(_can);
    _shield->disable(2);
    _shield->waitForSync(100);
    _shield->disable(3);
    _shield->waitForSync(100);
    _shield->disable(4);
    _shield->waitForSync(100);

    on_pwm2_freq_valueChanged(50);
    _shield->waitForSync(100);
    _shield->setPWMFrequency(3, 50);
    _shield->waitForSync(100);
    _shield->setPWMFrequency(4, 50);

    _freq2 = 50;
    _duty2 = 0;
    _duty3 = 0;
    _duty4 = 0;

    _shield->setPulseWidth(3, _duty3);
    _shield->waitForSync(100);
    _shield->setPulseWidth(4, _duty4);
    _shield->waitForSync(100);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(on_timer_update()));
    _timer->start(250);
}

MainWindow::~MainWindow()
{
    _shield->disable12V();
    _shield->disable19V();
    _shield->disable(2);
    _shield->disable(3);
    _shield->disable(4);

    delete _timer;
    delete _shield;
    _can->stopListener();
    delete _can;
    delete ui;
}

void MainWindow::on_cbCh2_stateChanged(int arg1)
{
    if(arg1)
        _shield->enable(2);
    else
        _shield->disable(2);
}

void MainWindow::on_cbCh3_stateChanged(int arg1)
{
    if(arg1)
        _shield->enable(3);
    else
        _shield->disable(3);
}

void MainWindow::on_cbCh4_stateChanged(int arg1)
{
    if(arg1)
        _shield->enable(4);
    else
        _shield->disable(4);
}

int duty2PWM(float duty, float freq)
{
    float period = 1.f/((float)freq); // period in sec.
    return (int)(duty / period * 100.f);
}

float pwm2Duty(int pwm, float freq)
{
    float period = 1.f/((float)freq); // period in sec.
    return ((float)pwm) / 100.f * period;
}

void MainWindow::on_pwm2_freq_valueChanged(int value)
{
    _shield->setPWMFrequency(2, value);
    _freq2  = value;
    ui->lcdPWM2->display(value);
}

void MainWindow::on_pwm2_duty_valueChanged(int value)
{
    float dutyMin = 750.f/1000000.f;  // min duty of servo HD 1810MG
    float dutyMax = 2250.f/1000000.f; // max duty of servo HD 1810MG
    float pwmMin  = duty2PWM(dutyMin, _freq2);
    float pwmMax  = duty2PWM(dutyMax, _freq2);
    if(value<pwmMin || value>pwmMax)
        return;
    _duty2 = pwm2Duty(value, _freq2);
    _shield->setPulseWidth(2, value);
    ui->lcdDuty2->display(_duty2*1000.f);
}

void MainWindow::on_cb12V_stateChanged(int arg1)
{
    if(arg1)
        _shield->enable12V();
    else
        _shield->disable12V();
}

void MainWindow::on_cb19V_stateChanged(int arg1)
{
    if(arg1)
        _shield->enable19V();
    else
        _shield->disable19V();
}

void MainWindow::on_pwm3_freq_valueChanged(int value)
{
    _shield->setPWMFrequency(3, value);
    _shield->setPulseWidth(3, _duty3);
}

void MainWindow::on_pwm3_duty_valueChanged(int value)
{
    _duty3 = value;
    _shield->setPulseWidth(3, value);
}

void MainWindow::on_pwm4_freq_valueChanged(int value)
{
    _shield->setPWMFrequency(4, value);
    _shield->setPulseWidth(4, _duty4);
}

void MainWindow::on_pwm4_duty_valueChanged(int value)
{
    _duty4 = value;
    _shield->setPulseWidth(4, value);
}

void MainWindow::on_timer_update()
{
    ui->lcdVoltage->display(_shield->getVoltage());
}
