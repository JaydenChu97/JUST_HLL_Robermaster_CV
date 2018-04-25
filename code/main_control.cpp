#include "main_control.h"

namespace HCVC
{
MainControl::MainControl()
{
    status = DETECTING;

    //初始化串口，如果初始化不成功
    if(serial.init("COM3"))
    {
        qDebug() << "SerialPort init sucess!" << endl;
    }
    
    FileStorage fs("statics/params.xml", FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "Open file failed" << endl;
    }

    FileNode node = fs["camera_parameter"];
        
    node["brightness"] >> params.brightness;
    node["contrast"] >> params.contrast;
    node["hue"] >> params.hue;
    node["saturation"] >> params.saturation;
    node["pan"] >> params.pan;
    node["gamma"] >> params.gamma;
    node["white_balance_red_v "] >> params.white_balance_red_v;
    node["backlight"]>> params.backlight;
    node["gain"] >> params.gain;
    node["exposure"] >>params.exposure;
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
    srcFile.set(CAP_PROP_BRIGHTNESS, params.brightness);
    srcFile.set(CAP_PROP_CONTRAST, params.contrast);
    srcFile.set(CAP_PROP_HUE, params.hue);
    srcFile.set(CAP_PROP_SATURATION, params.saturation);
    srcFile.set(CAP_PROP_PAN, params.pan);
    srcFile.set(CAP_PROP_GAMMA, params.gamma);
    srcFile.set(CAP_PROP_WHITE_BALANCE_RED_V, params.white_balance_red_v);
    srcFile.set(CAP_PROP_BACKLIGHT, params.backlight);
    srcFile.set(CAP_PROP_GAIN, params.gain);
    srcFile.set(CAP_PROP_EXPOSURE, params.exposure);

    return srcFile.isOpened();
}

void MainControl::run(const string& path)
{
    clock_t start, end;

    start = clock();

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
    vector<Point> points;

    //添加滑动控制条
    //Tool::addTrackBar(srcFilePath, srcFile);    

    //视频图像缓存区域
    Mat frame;
    int currentFrame = 0;
    while(true)
    {
        clock_t begin, finish;
        begin = clock();
        //添加运行时间统计
        Tool::getTimeCount(1);
        //添加滑动控制条跟随视频进度功能(这个功能极其耗时间，最好不要使用)
        //Tool::setTrackBarFollow(srcFilePath, srcFile);
        //添加键盘控制
        Tool::addKeyboardControl(srcFile);
        finish = clock();

        cout<<"toolTime:"<<(double)(finish - begin)/CLOCKS_PER_SEC<<"s"<<"\t"<<endl;

        begin = clock();
        //读取一帧图像
        srcFile >> frame;
        currentFrame++;
        finish = clock();

        cout<<"readImgTime:"<<(double)(finish - begin)/CLOCKS_PER_SEC<<"s"<<"\t"<<endl;

        //视频播放完毕跳出程序
        if(frame.empty())
        {
            break;
        }

        Mat resizeFrame(Size(1080, 560), CV_8UC1);

        //重调至合适的大小，减小运算量
        resize(frame, resizeFrame, Size(1080, 560));

        //检测到的装甲区域
        Rect2d armourBlock;
        bool findArmourBlock = false;

        begin = clock();
        //检测图片中的灯柱位置
        if(status == DETECTING && armourDetector.detect(resizeFrame))
        {
            armourBlock = armourDetector.getBestArmourBlock();
            armourTracker.init(resizeFrame, armourBlock);
            status = TRACKING;
            findArmourBlock = true;
        }
        finish = clock();
        cout<<"detectTime:"<<(double)(finish - begin)/CLOCKS_PER_SEC<<"s"<<"\t"<<endl;

        // 追踪装甲板区域
        if(status == TRACKING)
        {
            if(armourTracker.track(resizeFrame))
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
        //Tool::drawPoints(resizeFrame, points);

        //在输出图像中画出坐标系
        //Tool::drawCoord(resizeFrame);

        //向串口写入相对坐标
        //serial.writeBytes(armourBlock, resizeFrame, findArmourBlock);

        //在输出图像中画出装甲板中心轨迹
        points.push_back(Point(armourBlock.x +armourBlock.width/2,
                               armourBlock.y + armourBlock.height/2));
        Tool::drawPoints(resizeFrame, points);

        //在输出图像中画出坐标系
        Tool::drawCoord(resizeFrame);

        //显示原图像(重调大小后)
        imshow(srcFilePath, resizeFrame);
    }

    end = clock();
    cout<<"Run time: "<<(double)(end - start) / CLOCKS_PER_SEC<<"S"<<endl;

    destroyAllWindows();

}
}

