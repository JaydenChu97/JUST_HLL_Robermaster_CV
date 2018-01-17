/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     main_control.h                                                  *
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
* @defgroup control 总控制模块组
*
* @defgroup debug 调试模块组
*/

#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

/*自定义库*/
#include "armour_detector.h"
#include "armour_tracker.h"

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
class MainControl
{
public:
    /**
    * @brief 初始化
    *
    */
    MainControl();

    /**
    * @brief 设置源文件读取路径
    * @details 从指定路径读取视频文件
    * @param[in] path 读取源文件的路径
    * @return 文件是否读取成功。
    *         返回true，表示文件读取成功；
    *         返回false，表示文件读取失败
    */
    bool readSrcFile(const string& path);

    /**
    * @brief 运行整体系统并显示运行结果
    *
    * @param[in] path 读取源文件的路径
    * @return null
    */
    void run(const string &path);

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

    //! 读取文件路径
    string srcFilePath;

    //! 存储读取的文件数据
    VideoCapture srcFile;
};
//! @}
}
#endif // MAIN_CONTROL_H
