#include "tool.h"

namespace HCVC {
int g_trackBarLocation = 0;

void Tool::addTrackBar(const string& windowName, VideoCapture& file)
{
    double count = file.get(CAP_PROP_FRAME_COUNT);
    createTrackbar("trackBar", windowName, &g_trackBarLocation, static_cast<int>(count), onTrackBarCallback, &file);
}

void Tool::onTrackBarCallback(int pos, void* file)
{
    static_cast<VideoCapture*>(file)->set(CAP_PROP_POS_FRAMES, pos);
}

void Tool::setTrackBarFollow(const string& windowName, const VideoCapture& file)
{
    double cur = file.get(CAP_PROP_POS_FRAMES);
    setTrackbarPos("trackBar", windowName, static_cast<int>(cur));
}

void Tool::addKeyboardControl(VideoCapture& srcFile, const int& delay)
{
    //记录当前暂停帧
    static double recordFrame = -1;
    //相当于设定播放帧率为每秒1000/33=30帧
    switch(waitKey(delay))
    {
    //如果读取到esc键终止播放
    case ESC:
        srcFile.release();
        break;

    //如果读取到空格键暂停播放，直到再次按下空格键，期间可以按下esc键退出
    case PAUSE:
        if(recordFrame < 0)
        {
            recordFrame = srcFile.get(CAP_PROP_POS_FRAMES);
        }
        else
        {
            recordFrame = -1;
        }
        break;

    //j键为视频回放
    case MOVE_BACK:
        if(recordFrame < 0)
        {
            double cur = srcFile.get(CAP_PROP_POS_FRAMES);
            srcFile.set(CAP_PROP_POS_FRAMES, cur-1);
        }
        else
        {
            recordFrame--;
            srcFile.set(CAP_PROP_POS_FRAMES, recordFrame);
        }
        break;

    //k键为视频快进
    case MOVE_FORWARD:
        if(recordFrame < 0)
        {
            double cur = srcFile.get(CAP_PROP_POS_FRAMES);
            srcFile.set(CAP_PROP_POS_FRAMES, cur+1);
        }
        else
        {
            recordFrame++;
            srcFile.set(CAP_PROP_POS_FRAMES, recordFrame);
        }
        break;

    default:
        if(recordFrame >= 0)
        {
            srcFile.set(CAP_PROP_POS_FRAMES, recordFrame);
        }
    }
}

void Tool::getTimeCount(const int& id)
{
    static long long startTimes[100] = {0};

    if(startTimes[id] == 0)
    {
        startTimes[id] = getTickCount();
    }
    else
    {
        cout << "id - " << id << " Cost time: "
             << (getTickCount()-startTimes[id]) / getTickFrequency()
             << " s" << endl;

        startTimes[id] = getTickCount();
    }
}

void Tool::drawPoints(Mat resizeFrame, vector<Point>& points)
{
    for(unsigned int i = 0; i < points.size(); i++)
    {
        circle(resizeFrame, points[i], 3, Scalar(0, 0, 255));
    }
}

void Tool::drawCoord(Mat resizeFrame)
{
    line(resizeFrame, Point(0, resizeFrame.rows/2),
         Point(resizeFrame.cols, resizeFrame.rows/2), Scalar(0, 255, 0));
    line(resizeFrame, Point(resizeFrame.cols/2, 0),
         Point(resizeFrame.cols/2, resizeFrame.rows), Scalar(0, 255, 0));
}

void Tool::showPoints(Mat resizeFrame, short coord, int org_x, int org_y)
{
    //短型转整型
    const string text = to_string(coord);

    //坐标在屏幕上显示位置
    Point origin(org_x, org_y);

    //坐标字体
    int font_face = FONT_HERSHEY_COMPLEX;

    //显示坐标
    putText(resizeFrame, text, origin, font_face, 1, Scalar(0, 255, 255), 2);
}
}
