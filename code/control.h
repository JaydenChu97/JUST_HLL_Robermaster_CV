/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     control.h                                                  *
*  @brief    Overall operation logic control                                 *
*  Details.                                                                  *
*                                                                            *
*  @author   HLL                                                         *
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
*  2017/11/10 | 1.0.0.0   | Zhu Min        | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

/**
* @defgroup armour_recognition 装甲板识别模块组
*
* @defgroup device 设备驱动模块组
*
* @defgroup control 总控制模块组
*
* @defgroup debug 调试模块组
*/

#ifndef MAIN_H
#define MAIN_H

/*自定义库*/
#include "armour_detector.h"
#include "armour_tracker.h"
#include "serial.h"
#include "camera.h"
#include "video.h"

/**
* @brief HLL Computer Vision Code namepace.
*
*/
namespace HCVC {
//! @addtogroup control
//! @{
/**
 * @brief 系统总体逻辑控制
 * @details 包括装甲板的识别与追踪，大神符检测，串口通信模块的协调控制
 */
class Control
{
public:
    /**
    * @brief 初始化
    *
    */
    Control();

    /**
    * @brief 运行整体系统并显示运行结果
    * @return null
    */
    void run();

protected:
    //! 图像检测器，处理并分析图像找出装甲的初始位置
    ArmourDetector armourDetector;

    //! 运动追踪器，对经过图像检测后找到的灯柱区域跟踪
    ArmourTracker armourTracker;

private:
    //! 装甲板检测状态常量
    enum
    {
        DETECTING, /*!< 检测状态 */
        TRACKING,  /*!< 跟踪状态 */
    };

    //! 当前装甲板检测程序的状态
    int status;

    //! 串口通信类
    Serial serial;

    //! 摄像头
    Camera camera;

    //! 视频
    Video video;
};
//! @}
}
#endif // MAIN_H
