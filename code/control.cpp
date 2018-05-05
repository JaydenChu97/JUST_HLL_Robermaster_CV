#include "control.h"

namespace HCVC
{
Control::Control()
{
    status = DETECTING;

    //初始化串口，如果初始化不成功
    if(serial.init("COM6"))
    {
        qDebug() << "SerialPort init sucess!" << endl;
    }

    if(video.init("F:\\Robomaster\\视觉素材\\视觉素材\\炮台素材红车旋转-ev-0.MOV"))
    {
        qDebug() << "video init sucess!" << endl;
    }

    if(camera.init(0))
    {
        qDebug() << "camera init sucess!" << endl;
    }

    armourDetector.init(Image::RED);
}

void Control::run()
{
    //创建原图像显示窗口
    namedWindow("srcFile", WINDOW_FULLSCREEN);
    vector<Point> points;  

    //视频图像缓存区域
    Mat frame;
    while(true)
    {   
        //添加运行时间统计
        Tool::setTimeCount(1, Tool::BEGIN, "total time");
        //添加键盘控制
        //Tool::addKeyboardControl(srcFile);
        waitKey(1);

        //读取一帧图像
        camera >> frame;

        //视频播放完毕跳出程序
        if(frame.empty())
        {
            break;
        }

        resize(frame, frame, Size(1080, 560));

        //检测到的装甲区域
        Rect2d armourBlock;
        bool findArmourBlock = false;

        //检测图片中的灯柱位置
        if(status == DETECTING && armourDetector.detect(frame))
        {
            armourBlock = armourDetector.getBestArmourBlock();
            armourTracker.init(frame, armourBlock);
            status = TRACKING;
            findArmourBlock = true;
        }

        // 追踪装甲板区域
        if(status == TRACKING)
        {
            if(armourTracker.track(frame))
            {
                armourBlock = armourTracker.getArmourBlock();
                findArmourBlock = true;
            }
            else
            {
                status = DETECTING;
            }
        }        

        //在输出图像中画出装甲板中心轨迹
        points.push_back(Point(armourBlock.x +armourBlock.width/2,
                               armourBlock.y + armourBlock.height/2));
        //Tool::drawPoints(frame, points);

        //在输出图像中画出坐标系
        Tool::drawCoord(frame);

        //向串口写入相对坐标
        serial.writeBytes(armourBlock, frame, findArmourBlock);

        //显示原图像(重调大小后)
        imshow("srcFile", frame);

        //添加运行时间统计
        Tool::setTimeCount(1, Tool::END, "total time");
    }

    destroyAllWindows();
}
}

