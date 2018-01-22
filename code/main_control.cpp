#include "main_control.h"

namespace HCVC
{
MainControl::MainControl()
{
    status = DETECTING;
}

bool MainControl::readSrcFile(string path)
{
    srcFilePath = path;
    srcFile.open(srcFilePath);

    return srcFile.isOpened();
}

bool MainControl::readSrcFile(int cameraId)
{
    srcFilePath = string("cameraId:");
    srcFilePath.push_back('0'+cameraId);
    srcFile.open(cameraId);
    srcFile.set(CAP_PROP_SETTINGS, -1);

    return srcFile.isOpened();
}

void MainControl::run(const string& path)
{
    //读取源文件
    if(path == "camera")
    {
        readSrcFile(0);
    }
    else
    {
        if(!readSrcFile(path))
        {
            //异常处理，防止读取失败
            throw string("Can not read file from ") + path;
        }
    }

    //创建原图像显示窗口
    namedWindow(srcFilePath, WINDOW_FULLSCREEN);

    //添加滑动控制条
    //Tool::addTrackBar(srcFilePath, srcFile);

    //视频图像缓存区域
    Mat frame;
    while(true)
    {
        //添加运行时间统计
        Tool::getTimeCount(0);
        //添加滑动控制条跟随视频进度功能(这个功能极其耗时间，最好不要使用)
        //Tool::setTrackBarFollow(srcFilePath, srcFile);
        //添加键盘控制
        Tool::addKeyboardControl(srcFile);

        //读取一帧图像
        srcFile >> frame;
        //视频播放完毕跳出程序
        if(frame.empty())
        {
            break;
        }

        Mat resizeFrame(Size(1080, 560), CV_8UC1);

        //重调至合适的大小，减小运算量
        resize(frame, resizeFrame, Size(1080, 560));

        //检测图片中的灯柱位置
        if(status == DETECTING && armourDetector.detect(resizeFrame))
        {
            Rect2d armourBlock = armourDetector.getBestArmourBlock();
            armourTracker.init(resizeFrame, armourBlock);
            status = TRACKING;
        }

        // 追踪装甲板区域
        if(status == TRACKING && !armourTracker.track(resizeFrame))
        {
            status = DETECTING;
        }

        //显示原图像(重调大小后)
        imshow(srcFilePath, resizeFrame);
    }

    destroyAllWindows();
}
}

