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

    _freq1 = 0;
    _duty1 = 0;
    _freq2 = 0;
    _duty2 = 0;

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
    {
        _shield->enable(3);
        ui->cb12V->setEnabled(false);
    }
    else
    {
        _shield->disable(3);
        ui->cb12V->setEnabled(true);
    }
}

void MainWindow::on_cbCh4_stateChanged(int arg1)
{
    if(arg1)
    {
        _shield->enable(4);
        ui->cb19V->setEnabled(false);
    }
    else
    {
        _shield->disable(4);
        ui->cb19V->setEnabled(true);
    }
}

void MainWindow::on_pwm1_freq_valueChanged(int value)
{
    _shield->setPWMFrequency(1, value);
    _freq1  = value;
    ui->lcdPWM1->display(value);
}

void MainWindow::on_pwm1_duty_valueChanged(int value)
{
    _shield->setPulseWidth(1, value);
    ui->lcdDuty1->display(value);
}

void MainWindow::on_pwm2_freq_valueChanged(int value)
{
    _shield->setPWMFrequency(2, value);
    _freq2  = value;
    ui->lcdPWM2->display(value);
}

void MainWindow::on_pwm2_duty_valueChanged(int value)
{
    _shield->setPulseWidth(2, value);
    ui->lcdDuty2->display(value);
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

void MainWindow::on_timer_update()
{
    ui->lcdVoltage->display(_shield->getVoltage());
}

void MainWindow::on_cbCh2_clicked()
{

}
