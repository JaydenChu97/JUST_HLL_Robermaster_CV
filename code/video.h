#ifndef VIDEO_H
#define VIDEO_H

#include "common.h"

namespace HCVC
{
//! @addtogroup deivce
//! @{

/**
* @brief 视频模块
* @details 视频文件的初始化和读取图像
*/
class Video
{
public:
    Video();

    /**
    * @brief 设置视频文件读取路径
    * @details 从指定路径读取视频文件
    * @param[in] path 读取视频文件的路径
    * @return 视频文件是否读取成功。
    *         返回true，表示文件读取成功；
    *         返回false，表示文件读取失败
    */
    bool init(string path);

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

private:
    //! 存储读取的视频数据
    VideoCapture srcFile;

    //! 读取视频路径
    string srcFilePath;
};
//! @}
}

#endif // VIDEO_H
