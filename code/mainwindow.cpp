#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //滑动条文本框初始值设置
    ui->label_redMinValue->setNum(armourDetector.getThreshod(0, 0));
    ui->label_greenMinValue->setNum(armourDetector.getThreshod(1, 0));
    ui->label_blueMinValue->setNum(armourDetector.getThreshod(2, 0));
    ui->label_redMaxValue->setNum(armourDetector.getThreshod(0, 1));
    ui->label_greenMaxValue->setNum(armourDetector.getThreshod(1, 1));
    ui->label_blueMaxValue->setNum(armourDetector.getThreshod(2, 1));
    //滑动条初始值设置
    ui->horizontalScrollBar_redMin->setValue(armourDetector.getThreshod(0, 0));
    ui->horizontalScrollBar_greenMin->setValue(armourDetector.getThreshod(1, 0));
    ui->horizontalScrollBar_blueMin->setValue(armourDetector.getThreshod(2, 0));
    ui->horizontalScrollBar_redMax->setValue(armourDetector.getThreshod(0, 1));
    ui->horizontalScrollBar_greenMax->setValue(armourDetector.getThreshod(1, 1));
    ui->horizontalScrollBar_blueMax->setValue(armourDetector.getThreshod(2, 1));

    //灯柱块筛选条件初始值设置
    ui->angleRange->setValue(static_cast<double>(armourDetector.params.angleRange));
    ui->minArea->setValue(static_cast<double>(armourDetector.params.minArea));
    ui->maxHeightWidthRat->setValue(static_cast<double>(armourDetector.params.maxHeightWidthRat));
    ui->minHeightWidthRat->setValue(static_cast<double>(armourDetector.params.minHeightWidthRat));
    ui->maxWidthRat->setValue(static_cast<double>(armourDetector.params.maxWidthRat));
    ui->maxHeightRat->setValue(static_cast<double>(armourDetector.params.maxHeightRat));
    ui->maxAngleDiff->setValue(static_cast<double>(armourDetector.params.maxAngleDiff));
    ui->maxHeightGapRat->setValue(static_cast<double>(armourDetector.params.maxHeightGapRat));
    ui->minHeightGapRat->setValue(static_cast<double>(armourDetector.params.minHeightGapRat));
}

MainWindow::~MainWindow()
{
    delete ui;
}

/*三通道阈值过滤滑动条设置*/
void MainWindow::on_horizontalScrollBar_redMin_valueChanged(int value)
{
    value = value < armourDetector.getThreshod(0, 1) ? value : armourDetector.getThreshod(0, 1);
    armourDetector.setThreshod(0, 0, value);
    ui->label_redMinValue->setNum(armourDetector.getThreshod(0, 0));
}

void MainWindow::on_horizontalScrollBar_greenMin_valueChanged(int value)
{
    value = value < armourDetector.getThreshod(1, 1) ? value : armourDetector.getThreshod(1, 1);
    armourDetector.setThreshod(1, 0, value);
    ui->label_greenMinValue->setNum(armourDetector.getThreshod(1, 0));
}

void MainWindow::on_horizontalScrollBar_blueMin_valueChanged(int value)
{
    value = value < armourDetector.getThreshod(2, 1) ? value : armourDetector.getThreshod(2, 1);
    armourDetector.setThreshod(2, 0, value);
    ui->label_blueMinValue->setNum(armourDetector.getThreshod(2, 0));
}

void MainWindow::on_horizontalScrollBar_redMax_valueChanged(int value)
{
    value = value > armourDetector.getThreshod(0, 0) ? value : armourDetector.getThreshod(0, 0);
    armourDetector.setThreshod(0, 1, value);
    ui->label_redMaxValue->setNum(armourDetector.getThreshod(0, 1));
}

void MainWindow::on_horizontalScrollBar_greenMax_valueChanged(int value)
{
    value = value > armourDetector.getThreshod(1, 0) ? value : armourDetector.getThreshod(1, 0);
    armourDetector.setThreshod(1, 1, value);
    ui->label_greenMaxValue->setNum(armourDetector.getThreshod(1, 1));
}

void MainWindow::on_horizontalScrollBar_blueMax_valueChanged(int value)
{
    value = value > armourDetector.getThreshod(2, 0) ? value : armourDetector.getThreshod(2, 0);
    armourDetector.setThreshod(2, 1, value);
    ui->label_blueMaxValue->setNum(armourDetector.getThreshod(2, 1));
}

void MainWindow::on_angleRange_valueChanged(double arg1)
{
    armourDetector.params.angleRange = static_cast<float>(arg1);
}

void MainWindow::on_minArea_valueChanged(double arg1)
{
    armourDetector.params.minArea = static_cast<float>(arg1);
}

void MainWindow::on_maxHeightWidthRat_valueChanged(double arg1)
{
    armourDetector.params.maxHeightWidthRat = static_cast<float>(arg1);
}

void MainWindow::on_minHeightWidthRat_valueChanged(double arg1)
{
    armourDetector.params.minHeightWidthRat = static_cast<float>(arg1);
}

void MainWindow::on_maxWidthRat_valueChanged(double arg1)
{
    armourDetector.params.maxWidthRat = static_cast<float>(arg1);
}

void MainWindow::on_maxHeightRat_valueChanged(double arg1)
{
    armourDetector.params.maxHeightRat = static_cast<float>(arg1);
}

void MainWindow::on_maxAngleDiff_valueChanged(double arg1)
{
    armourDetector.params.maxAngleDiff = static_cast<float>(arg1);
}

void MainWindow::on_maxHeightGapRat_valueChanged(double arg1)
{
    armourDetector.params.maxHeightGapRat = static_cast<float>(arg1);
}

void MainWindow::on_minHeightGapRat_valueChanged(double arg1)
{
    armourDetector.params.minHeightGapRat = static_cast<float>(arg1);
}
