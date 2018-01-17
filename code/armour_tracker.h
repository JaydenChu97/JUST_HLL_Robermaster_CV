/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     armour_tracker.h                                                *
*  @brief    Track the armour zone                                           *
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

#ifndef ARMOUR_TRACKER_H
#define ARMOUR_TRACKER_H

#include "image_preprocessor.h"

namespace HCVC {
//! @addtogroup armour_recognition
//! @{

/**
 * @brief 装甲板追踪模块
 * @details 在装甲板检测得出最佳区域后，对该区域进行追踪
 */
class ArmourTracker
{
public:
    /**
    * @brief 初始化
    *
    */
    ArmourTracker();

    /**
    * @brief 创建图像追踪器并设定参数
    * @param[in] srcImage 待检测原图像
    * @param[in] armourBlock 需要追踪的矩形区域
    * @return null
    */
    void init(const Mat& srcImage, Rect2d armourBlock);

    /**
    * @brief 追踪目标区域
    * @param[in] srcImage 待检测原图像
    * @return 是否追踪成功，追踪成功返回true，追踪失败返回false
    */
    bool track(Mat srcImage);

private:
    //! kcf匹配算法图像追踪器
    Ptr<TrackerKCF> tracker;

    /**
    * @brief sobel特征提取器
    * @param[in] img 待检测原图像
    * @param[in] roi 需要提取特征的区域
    * @param[out] feat 输出特征
    * @return null
    * @deprecated 效果不好，暂未使用，后期考虑提取其它特征
    */
    static void sobelExtractor(const Mat img, const Rect roi, Mat& feat);
};
//! @}
}
#endif // ARMOUR_TRACKER_H
