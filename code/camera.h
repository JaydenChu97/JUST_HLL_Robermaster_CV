#ifndef CAMERA_H
#define CAMERA_H

/*公共头文件*/
#include "common.h"

namespace HCVC
{
//! @addtogroup deivce
//! @{

/**
* @brief 摄像头模块
* @details 摄像头的初始化和读取图像
*/
class Camera
{
public:
    Camera();

    /**
    * @brief 设置读取摄像头源
    * @details 从指定摄像头编号读取视频流
    * @param[in] cameraId 读取摄像头编号
    * @return 摄像头是否打开成功。
    *         返回true，表示摄像头打开成功；
    *         返回false，表示摄像头打开失败
    */
    bool init(int cameraId);

    /**
    * @brief 读取视频流下一帧图像
    * @param[out] frame 下一帧图像
    * @return null
    */
    void getNextFrame(Mat& frame);

    /**
    * @brief 读取视频流下一帧图像
    * @param[out] frame 下一帧图像
    * @return 视频流数据结构
    */
    VideoCapture& operator >> (Mat& frame);

    //! 摄像头参数
    struct Params
    {
        double brightness;             /*!< 亮度 */
        double contrast;               /*!< 对比度 */
        double hue;                    /*!< 色调 */
        double saturation;             /*!< 饱和度 */
        double pan;                    /*!< 清晰度 */
        double gamma;                  /*!< 伽马 */
        double white_balance_red_v;    /*!< 白平衡 */
        double backlight;              /*!< 逆光对比 */
        double gain;                   /*!< 增益 */
        double exposure;               /*!< 曝光 */
    }params;

private:
    //! 存储摄像头视频流
    VideoCapture srcFile;

    //! 摄像头编号
    string cameraId;
};
//! @}
}

#endif // CAMERA_H
