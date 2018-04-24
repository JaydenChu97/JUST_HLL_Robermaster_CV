/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                                *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     serial.h                                                        *
*  @brief    Serial communication library                                    *
*  Details.                                                                  *
*                                                                            *
*  @author   HLL                                                             *
*  @email    sin1997@gmail.com                                               *
*  @version  1.0.0.0                                                         *
*  @date     2017.11.10                                                      *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2017/1/20 | 1.0.0.0   | Zhu Min            | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

/*Qt库*/
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>

/*opencv库*/
#include "opencv2/opencv.hpp"

/*自定义库*/
#include "tool.h"

namespace HCVC {
//! @addtogroup control
//! @{

/**
 * @brief 串口通信模块组
 * @details 提供和电控部分串口通信功能，发送相对坐标，接收反馈信息
 */
class Serial: public QSerialPort
{    
    Q_OBJECT

public:
    /**
    * @brief 初始化
    *
    */
    Serial();

    /**
    * @brief 初始化串口，设定串口通信参数
    * @param[in] portName 串口名称
    * @return 串口初始化是否成功
    *         返回true，表示初始化成功；
    *         返回false，表示串口初始化出现错误
    */
    bool init(QString portName);

    /**
    * @brief 向串口写入相对坐标
    * @details 串口写入四字节数据。前两个字节为一个短整型数，表示x轴相对坐标；后两个字节为
    *          也为一个短整型数，表示y轴相对坐标。都为小端模式，即低字节节在前，高字节在
    *          后
    * @param[in] xDiff x轴相对坐标
    * @param[in] yDiff y轴相对坐标
    * @return null
    * @note 计算的坐标原点为图像几何中心，建立笛卡尔坐标系
    */
    void writeBytes(const cv::Rect2d& armourBlock, const cv::Mat& resizeFrame,
                          const bool& findArmourBlock);

private:
    short HEAD = 16191; // equal to 00111111 00111111
    short TAIL = 7967; // equal to 00011111 000011111

    void convertCoord(const cv::Rect2d& armourBlock, const cv::Mat& resizeFrame,
                      short& xDiff, short& yDiff);

private slots:
    /**
    * @brief 回调函数。一旦串口接收到数据，则触发该函数，读取数据，并执行一定操作。
    * @note  具体功能待实现。
    */
    void readBytes();
};
//! @}
}
#endif // SERIAL_H
