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
}
