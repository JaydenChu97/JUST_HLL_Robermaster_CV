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
     * @brief 根据检测结果进行更新
     * @param[in] srcImage 输入图像
     */
    void update(Mat& srcImage);

    /**
    * @brief 追踪目标区域
    * @param[in] srcImage 待检测原图像
    * @return 是否追踪成功，追踪成功返回true，追踪失败返回false
    */
    bool track(Mat srcImage);

    /**
     * @brief 对更新后的矩形框进行矫正
     * @details 将矩形框进行放大，在放大后矩形框内根据轮廓数量信息分类筛选
     * @param[in] updateRoi 更新后的矩形框
     * @param[in] srcImage 原图像
     * @return 矫正后的矩形框
     */
    Rect2d refineRect(Mat& updateRoi, Mat& srcImage);

    /**
     * @brief 对越界矩形框的进行矫正
     * @param[out] minArmourRect 矫正后的矩形框
     * @param[in] rotatedRect 更新后的矩形内连通域外接矩形
     * @param[in] rotatedSize rotatedRect数组的长度
     * @return null
     */
    void refineOverBorder(RotatedRect& minArmourRect, RotatedRect* rotatedRect, int rotatedSize);

    /**
     * @brief 对未越界但只含一个连通域的矩形框进行矫正
     * @param[out] minArmourRect 矫正后的矩形框
     * @param[in] rotatedRect 更新后的矩形内连通域外接矩形
     * @param[in] rotatedSize rotatedRect数组的长度
     * @return null
     */
    void refineNonOverBorder(RotatedRect& minArmourRect, RotatedRect* rotatedRect, int rotatedSize);

    /**
     * @brief 对更新后矩形框内连通域数量大于2的矩形框进行矫正
     * @details 分为单独更新后矩形内连通域匹配与更新后连通域与放大后其余连通域匹配
     * @details 将扩大后矩形内连通域分为两类，一类与更新后矩形框形同，其余为另一类，分别进行筛选
     * @param[out] minAreaRect 矫正后的矩形框
     * @param[in] rotatedRect 更新后的矩形内连通域外接矩形
     * @param[in] rotatedSize rotatedRect数组的长度
     * @return null
     */
    void searchmatchDomains(RotatedRect& minAreaRect, RotatedRect* rotatedRect, int rotatedSize);

    /**
     * @brief 对分类后单独的更新后矩形内完整矩形矩形筛选匹配
     * @param[out] updateClone 通过单个矩形性质筛选出来的矩形框
     * @param[in] updateBlocks 分类后的更新后矩形框内完整矩形
     * @param[in] rotatedSize updateBlocks数组的长度
     * @param[out] number updateClone数组内实际储存的矩形数量
     * @return 最后筛选出来的包围两连通域的旋转矩形
     */
    vector<RotatedRect> adjustScreen(RotatedRect* updateClone, RotatedRect* updateBlocks,
                                     int rotatedSize, int number);

    /**
     * @brief 对第一类分类后未成功匹配与第二类分类的矩形进行匹配筛选
     * @param[in] updateClone 通过单个矩形性质筛选出来的矩形框
     * @param[in] number updateClone数组内实际储存的矩形数量
     * @param[in] adjustBlocks 分类后除更新后矩形连通域的放大后矩形区域内的连通域外接矩形
     * @param[in] saveRotatedSize adjustBlocks数组的长度
     * @return 最后筛选出来的包围两连通域的旋转矩形
     */
    vector<RotatedRect> updateScreen(RotatedRect* updateClone, int number,
                                     RotatedRect* adjustBlocks, int saveRotatedSize);

    RotatedRect armourConfidence(vector<RotatedRect>& armours);

    /**
     * @brief 获取分类后筛选出来的两矩形框的外接矩形
     * @param[in] matchDomains 筛选出来的两矩形框
     * @param[in] matchSize matchDomains数组的长度
     * @return 量矩形框的外接矩形
     */
    RotatedRect getArmourRotated(RotatedRect* matchDomains, int matchSize);

    Rect2d getArmourBlock() const;

    void fourierTransform(Mat& src);     

private:
    //! kcf匹配算法图像追踪器
    Ptr<TrackerKCF> tracker;

    //! 检测到的装甲板区域
    Rect2d armourBlock;

    Mat feat;
    Mat roi;                    //检测的装甲板区域
    Mat updateRoi;              //更新后的检测框
    Mat adjustRoi;              //调整后检测的装甲板区域
    Mat updateValue;            //更新后的检测框V通道区域
    Mat adjustValue;            //调整后检装甲板区域V通道图

    int size_patch[2];          // hog特征的sizeY，sizeX
    float initArmourLength;     //初始图像装甲板长度
    float gamma;                //初始灯条最小外接矩形的旋转角
};
//! @}
}
#endif // ARMOUR_TRACKER_H
