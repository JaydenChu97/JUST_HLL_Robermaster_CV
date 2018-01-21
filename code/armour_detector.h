/*****************************************************************************
*  HLL Computer Vision Code                                                  *
*  Copyright (C) 2017 HLL  sin1997@gmail.com.                            *
*                                                                            *
*  This file is part of HCVC.                                                *
*                                                                            *
*  @file     armour_detector.h                                               *
*  @brief    Detect the armour zone                                          *
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

#ifndef ARMOUR_DETECTOR_H
#define ARMOUR_DETECTOR_H

#include "image_preprocessor.h"
#include "stack"

namespace HCVC {
//! @addtogroup armour_recognition
//! @{

/**
 * @brief 装甲板检测模块
 * @details 在图像预处理之后，检测图像确定装甲板区域，根据一定评价原则选出最佳目标
 */
class ArmourDetector: public ImagePreprocessor
{
public:
    /**
    * @brief 加载装甲板区域判定参数
    *
    */
    ArmourDetector();

    /**
    * @brief 检测图像中是否存在装甲板区域
    * @details 对装甲板检测流程总体控制，并将检测出来的装甲板区域旋转矩形存储起来
    * @param[in] srcImage 待检测原图像，图像由摄像头直接传回，并未经过处理
    * @return 图像中是否存在装甲板区域。
    *         返回true，表示检测到装甲板区域；
    *         返回false，表示未检测到装甲板区域
    */
    bool detect(const Mat& srcImage);

    /**
    * @brief 获取上一次图像中检测出的最佳装甲板区域
    * @details 对装甲板检测流程总体控制，向外提供运行装甲板检测模块运行接口
    * @return 包围装甲板区域旋转矩形的矩形
    */
    Rect2d getBestArmourBlock() const;

    /**
    * @brief 在原图像上画出旋转矩形，便于调试
    * @param[in] srcImage 待检测原图像
    * @param[in] minRotatedRects 需要画出的全部旋转矩形
    * @param[in] color 线条颜色
    * @return null
    */
    void drawBlocks(Mat srcImage, const vector<RotatedRect>& minRotatedRects, const Scalar& color) const;

    //! 装甲板判定参数
    struct Params
    {
        float angleRange;          /*!< 单团块检测，倾斜角度限制 */
        float minArea;             /*!< 单团块检测，最小面积限制 */
        float maxHeightWidthRat;   /*!< 单团块检测，最大高宽比 */
        float minHeightWidthRat;   /*!< 单团块检测，最小高宽比 */

        float maxWidthRat;         /*!< 团块对匹配，最大宽度差 */
        float maxHeightRat;        /*!< 团块对匹配，最大高度差 */
        float maxAngleDiff;        /*!< 团块对匹配，最大角度差(暂时没有使用，效果不明显) */
        float maxHeightGapRat;     /*!< 团块对匹配，最大高度间距比 */
        float minHeightGapRat;     /*!< 团块对匹配，最小高度间距比 */
    }params;

private:
    /**
     * @brief 最佳装甲板区域结构体
     *
     */
    struct OptimalArmourBlock
    {
        //! 包围装甲板区域的选装矩形
        RotatedRect block;
        //! 对该装甲板区域的评分
        double grade;

        //! 初始化参数
        OptimalArmourBlock(const RotatedRect& _block, const double& _grade):
        block(_block), grade(_grade)
        {}

        //! 重载小于符号，便于直接使用sort函数，按从小到大排序
        bool operator < (const OptimalArmourBlock& other) const
        {
            return grade < other.grade;
        }
    };

    //！ 按顺序存储最优的装甲板区域
    vector<OptimalArmourBlock> optimalArmourBlocks;

    /**
    * @brief 搜寻图中所有独立的团块
    * @param[in] srcImage 待检测原图像
    * @return 所有连通块的点集合
    */
    vector<vector<Point> > searchBlocks(Mat srcImage);

    /**
    * @brief 用广度优先搜索填充每一个连通块
    * @param[in] srcImage 待检测原图像
    * @param[out] blocks 所有连通块的点集合，初始为空
    * @param[in] row 连通块第一个点的纵坐标
    * @param[in] col 连通块第一个点的横坐标
    * @return null
    */
    void fillLampBlock(Mat& srcImage, vector<vector<Point> >& blocks, int row, int col);

    /**
    * @brief 计算每一个团块的信息，并进行初步的筛选即去除掉一些不符合条件的团块
    * @param[in] blocks 团块点的集合
    * @return 包围团块的最小旋转矩形数组
    */
    vector<RotatedRect> calcBlocksInfo(const vector<vector<Point> >& blocks);

    /**
    * @brief 进一步筛选，匹配团块即灯柱对，提取出最优目标
    * @details 通过初步筛选，甲板像素离散检测，框定区域内连通域数量检测确定甲板
    * @param[in] minRotatedRects 包围团块的最小旋转矩形数组
    * @return 包围灯柱对即装甲板区域的最小旋转矩形数组
    */
    vector<RotatedRect> extracArmourBlocks(const vector<RotatedRect>& lampBlocks,const Mat srcImage,const Mat dstImage);

    /**
     * @brief 建立检测所需要的包含两灯柱的掩码
     * @param[in] initPoints 通过初步筛选两灯柱的最小外接矩形的角
     * @return null
     */
    void estableMask(Mat mask,const Mat& dstImage,const vector<Point> initPoints);

    /**
     * @brief 根据灰度图计算甲板的区间范围的值
     * @details 计算甲板像素平均值左右与大于某区间的像素比例
     * @param[in] mask 建立的掩码
     * @param[in] srcImage 原图像
     * @param[in] avg 像素的平均值
     * @param[in] mean 区间范围内像素的平均值
     * @param[in] percent 区间范围外像素的平均值
     * @return null
     */
    void calcDeviation(vector<RotatedRect> initLightBlocks,const Mat& srcImage,const Mat& dstImage, double& avg, double& mean, double&percent);

    /**
     * @brief 连通域数量检测
     * @details 求两灯柱外接矩形，检测矩形内的连通域，若连通域数量为2，则返回该灯柱团块
     * @param[in] allInitLightBlocks 初步筛选一帧总的灯柱矩形
     * @param[in] initLightBlocks 初步筛选两个外接矩形
     * @param[in] finalLightBlocks 连通域数量检测后符合条件的矩形
     * @return null
     */
    vector<RotatedRect> domainCountDetect(const vector<RotatedRect> &allInitLightBlocks,const vector<RotatedRect> &initLightBlocks, vector<RotatedRect> &finalLightBlocks,const Mat& dstImage);

    /**
     * @brief 绘制连通域数量检测的团块
     * @details 根据初步筛选的总团块绘制连通域数量检测的二值化图
     * @param[in] allInitLightBlocks 初步筛选的总团块
     * @return 包围甲板的矩形
     */
    void drawLabelImg(const vector<RotatedRect> &allInitLightBlocks,Mat&labelImg);

    /**
    * @brief 对最后提取出的灯柱区域评分，选出最优区域
    * @param[in] srcImage 待检测原图像
    * @param[in] dstImage 对原图像进行图像预处理后的图像
    * @param[in] armourBlocks 包围装甲板区域的最小旋转矩形数组
    * @return null
    */
    void markArmourBlocks(const Mat& srcImage, const Mat& dstImage, const vector<RotatedRect>& armourBlocks);

    /**
    * @brief 减去旋转矩形的边角，使旋转矩形的左右两条边与竖直方向平行，成为平行四边形
    * @param[in] points 旋转矩形的四个角点
    * @return null
    */
    void cutEdgeOfRect(Point2f* points);
};
//! @}
}
#endif // ARMOUR_DETECTOR_H
