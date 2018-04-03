/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     tool.h                                                          *
*  @brief    Provide some debug tools                                        *
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
*  2017/11/10 | 1.0.0.0   | Zhu Min        | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#ifndef TOOL_H
#define TOOL_H

/*opencv库*/
#include "opencv2/opencv.hpp"

/*C++标准库*/
#include "string"
#include "iostream"

using namespace cv;
using namespace std;

namespace HCVC {
//! @addtogroup debug
//! @{
//! 滑动控制条当前位置
extern int g_trackBarLocation;

/**
 * @brief 提供一些调试用工具
 * @details 包括视频播放控制的功能实现
 */
class Tool
{
public:
    /**
    * @brief 初始化
    *
    */
    Tool()
    {
    }

    /**
    * @brief 添加滑动控制条
    * @details 通过滑动控制条可以控制视频播放进度
    * @param[in] windowName 添加滑动控制条的窗口名称
    * @param[in] file 用于被滑动控制条控制的视频文件
    * @return null
    */
    static void addTrackBar(const string& windowName, VideoCapture& file);

    /**
    * @brief 添加滑动控制条跟随视频进度功能
    * @details 使滑动控制条能够跟随视频播放进度同步移动
    * @param[in] windowName 添加滑动控制条的窗口名称
    * @param[in] file 用于被滑动控制条跟随的视频文件
    * @return null
    * @warning 滑动控制条跟随视频播放进度极其耗费时间，影响程序运行效率，需慎重使用
    */
    static void setTrackBarFollow(const string& windowName, const VideoCapture& file);

    //! 视频播放控制状态常量
    enum
    {
      ESC = 27,               /*!< 退出 */
      PAUSE = ' ',            /*!< 暂停 */
      MOVE_BACK = 'j',        /*!< 回放一帧 */
      MOVE_FORWARD = 'k',     /*!< 快进一帧 */
    };

    /**
    * @brief 添加键盘按键控制
    * @details 使能够通过键盘快捷键控制视频的播放，停止，结束
    * @param[in] srcFile 需要控制进度的视频文件
    * @param[in] delay 视频播放的每一帧时间间隔
    * @return null
    */
    static void addKeyboardControl(VideoCapture& srcFile, const int& delay = 1);

    /**
    * @brief 添加运行时间统计
    * @details 对每一次循环的运行时间进行统计，分析程序运行效率
    * @param[in] id 统计运行时间的代码块编号, 范围:[0, 100)
    * @return null
    * @note 实际计算时间是通过两次调用该函数的时间差来实现
    */
    static void getTimeCount(const int& id);

    /**
     * @brief 画图像
     * @param Points 输入的一组点
     */
    static void drawPoints(Mat resizeFrame, vector<Point>& points);
private:
    /**
    * @brief 滑动控制条回调函数
    * @param[in] pos 滑动控制条的当前位置
    * @param[in] data 额外传递的数据
    * @return null
    * @note 回调函数在类中只能设置为静态函数
    */
    static void onTrackBarCallback(int pos, void* data);
};
//! @}
}
#endif // TOOL_H
