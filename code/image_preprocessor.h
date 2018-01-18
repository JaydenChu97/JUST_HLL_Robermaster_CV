/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     image_preprocessor.h                                            *
*  @brief    Preprocess image to detect the armour zone and track it         *
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
*  2017/11/10 | 1.0.0.0   | Zhu Min            | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#ifndef IMAGE_PREPROCESSOR_H
#define IMAGE_PREPROCESSOR_H

/*C++标准库*/
#include "algorithm"
#include "iostream"
#include "string"

/*opencv库*/
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "tracking.hpp"

/*自定义库*/
#include "tool.h"

using namespace std;
using namespace cv;

namespace HCVC
{
//! @addtogroup armour_recognition
//! @{

/**
* @brief 图像预处理模块
* @details 去除噪声，平衡亮度，阈值处理等
*/
class ImagePreprocessor
{
public:
    /**
    * @brief 初始化
    *
    */
    ImagePreprocessor();

    /**
    * @brief 对读取到的图片预处理
    * @details 对图像进行基本的形态学操作和滤波操作
    * @param[in] srcImage 源图像矩阵
    * @return 预处理后的图像
    */
    Mat preprocess(const Mat& srcImage);

    /**
     * @brief 初始化二值化图
     * @param[in] 建立的背景
     */
    void initThreshold(Mat &framethreshold);

    /**
     * @brief 进行灯条的筛选
     * @param[in] framethreshold 储存二值化图像
     * @param[in] Hue hue通道
     * @param[in] Statulation saturation通道
     * @param[in] Value value通道
     */
    void threshProcess(Mat &framethreshold,Mat &Hue,Mat &Staturation,Mat &Value);

    /**
     * @brief 对二值化图进行去噪处理
     * @param[in] framethreshold 二值化图
     * @param[in] hue H通道
     * @param[in] saturation S通道
     */
    void wipePoints(Mat &framethreshold,Mat &hue,Mat &saturation);

    /**
    * @brief 设置预处理图像阈值
    * @param[in] channel 目标通道
    * @param[in] minOrMax 设置最大值还是最小值
    * @param[in] value 设定阈值
    * @return null
    */       
    void setThreshod(int channel, int minOrMax, int value);

    /**
    * @brief 获取指定通道最大值或者最小值
    * @param[in] channel 目标通道
    * @param[in] minOrMax 获取最大值还是最小值
    * @return 指定通道值
    */
    int getThreshod(int channel, int minOrMax) const;

private:
    //! 存储三通道过滤时，rgb的下上限值
    vector<int> thresholds[3];

    /**
    * @brief 用threshold函数实现区间图像提取
    * @details 相当于threshold的升级版，用于提取一定阈值区间的图像
    * @param[in] srcImage 阈值分割的源图像
    * @param[in] channel 目标通道
    * @return 指定通道值
    * @note opencv中的inrange函数可以实现类似的功能，但是最后得到的是二值图像
    */
    Mat rangeThreshold(const Mat& srcImage, const int& channel);
};
//! @}
}

#endif // IMAGE_PREPROCESSOR_H
