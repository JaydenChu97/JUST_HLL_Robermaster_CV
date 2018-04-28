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

#ifndef IMAGE_H
#define IMAGE_H

/*公共头文件*/
#include "common.h"

/*OpenCV追踪算法库*/
#include "opencv2/video/tracking.hpp"
#include "tracking.hpp"

/*自定义库*/
#include "tool.h"

namespace HCVC
{
//! @addtogroup armour_recognition
//! @{

/**
* @brief 图像预处理模块
* @details 去除噪声，平衡亮度，阈值处理等
*/
class Image
{
public:
    /**
    * @brief 初始化
    *
    */
    Image();

    //! 装甲板颜色枚举量
    enum Color
    {
        RED = 0,   /*!< 红色 */
        BLUE = 1,  /*!< 蓝色 */
    };

    /**
    * @brief 设置图像处理参数
    * @param[in] color 当前需要识别的装甲板颜色
    * @return 是否初始化成功
    */
    bool init(Color color);

    /**
    * @brief 对读取到的图片预处理
    * @details 对图像进行基本的形态学操作和滤波操作
    * @param[in] srcImage 源图像矩阵
    * @return 预处理后的图像
    */
    Mat preprocess(const Mat& srcImage);

    /**
     * @brief 进行灯条的筛选，以V通道为主要通道，H与S通道为辅助通道构建二值化图
     * @details 在V通道中寻找团块坐标，将此坐标在H于S通道中处理
     *          根据团块周围是否存在H通道阈值像素与R,B像素比例区分敌我
     *          根据团块周围是否存在S通道阈值像素去除背景
     *          根据轮廓上像素在H通道情况去除高亮部分
     * @param[out] framethreshold 得到的二值化图像
     * @param[in] hue hue通道
     * @param[in] saturation saturation通道
     * @param[in] value value通道
     * @param[in] orgValue 亮度图原始通道
     * @return null
     */
    void redThreshProcess(const Mat& srcImage,
                       Mat& framethreshold,
                       Mat& hue,
                       Mat& saturation,
                       Mat& value,
                       Mat& orgValue);

    /**
     * @brief 进行灯条的筛选，以V通道为主要通道，H与S通道为辅助通道构建二值化图
     * @details 在V通道中寻找团块坐标，将此坐标在H于S通道中处理
     *          根据团块周围是否存在H通道阈值像素与R,B像素比例区分敌我
     *          根据团块周围是否存在S通道阈值像素去除背景
     *          根据轮廓上像素在H通道情况去除高亮部分
     * @param[out] framethreshold 得到的二值化图像
     * @param[in] hue hue通道
     * @param[in] saturation saturation通道
     * @param[in] value value通道
     * @param[in] orgValue 亮度图原始通道
     * @return null
     */
    void blueThreshProcess(const Mat& srcImage,
                       Mat& framethreshold,
                       Mat& hue,
                       Mat& saturation,
                       Mat& value,
                       Mat& orgValue);

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

    Mat detectValue;

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

    //! 颜色(RED, BLUE)
    Color color;
};
//! @}
}

#endif // IMAGE_H
