#include "image_preprocessor.h"

namespace HCVC {
ImagePreprocessor::ImagePreprocessor()
{
    //初始化阈值列表
    FileStorage fs("statics/params.xml", FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "Open file failed" << endl;
    }
    FileNode node = fs["image_preprocessor_threshod"];
    
    for(unsigned int i = 0; i < 3; i++)
    {
        //cout << int(node[2*i]) << endl;
        thresholds[i].push_back(int(node[2*i]));
        thresholds[i].push_back(int(node[2*i+1]));
    }
}

Mat ImagePreprocessor::preprocess(const Mat& srcImage)
{
    Mat dstImage;
    //均值滤波，去除局部极值
    //blur(srcImage, dstImage, Size(3,3));
    //将bgr格式(opencv默认将彩色图片存储为bgr格式)图像转变为hsv格式
    cvtColor(srcImage, dstImage, CV_BGR2HSV);
    //分离图像三通道
    Mat hsvImages[3];
    split(dstImage, hsvImages);

    hsvImages[0] = rangeThreshold(hsvImages[0], 0);
    hsvImages[1] = rangeThreshold(hsvImages[1], 1);
    hsvImages[2] = rangeThreshold(hsvImages[2], 2);

    //获取自定义核，核边长只能为奇数
    //Mat erodeKernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel_1 = getStructuringElement(MORPH_RECT, Size(2,2));
    Mat kernel_2 = getStructuringElement(MORPH_RECT, Size(8,8));

    //开运算去除小的噪声点，闭运算连接断开部分，对H与S通道进行处理，方便去除背景干扰
    Mat Hue,Saturation,Value;
    hsvImages[2].convertTo(Value,CV_8UC1);
    morphologyEx(hsvImages[0],Hue,MORPH_OPEN,kernel_1);
    morphologyEx(Hue,Hue,MORPH_CLOSE,kernel_2);
    morphologyEx(hsvImages[1],Saturation,MORPH_OPEN,kernel_1);
    morphologyEx(Saturation,Saturation,MORPH_CLOSE,kernel_2);

    //初始化二值化图
    Mat framethreshold = Mat(Value.size(), CV_8UC1,Scalar(0));

    //根据三个通道初步绘制二值化图
    threshProcess(framethreshold,Hue,Saturation,Value);

    //开运算去除二值化图部分噪声点
    Mat kernel = getStructuringElement(MORPH_RECT, Size(2,2));
    morphologyEx(framethreshold, framethreshold, MORPH_OPEN, kernel);

    //根据团块外接轮廓的H与S通道去噪
    wipePoints(framethreshold,Hue,Saturation);
    //中值滤波
    medianBlur(framethreshold,framethreshold,3);

    //显示单通道处理后图像
    imshow("hImage", Hue);
    imshow("SImage", Saturation);
    imshow("VImage", Value);

    //显示预处理后图像
    imshow("result", framethreshold);

    return framethreshold;
}

void ImagePreprocessor::threshProcess(Mat &framethreshold,Mat &Hue,Mat &Staturation,Mat &Value)
{
    vector< vector<Point> > contours;//定义一个返回轮廓的容器
    findContours(Value, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//查找轮廓，只检索最外面的轮廓，将所有的连码点转换成点
    vector<Rect> boundRect(contours.size());//画矩形
    Point*center;//声明点
    center = new Point[contours.size()];//创建点的数组
    Rect box;    

    //框定待检测区域
    for(int a=0;a<contours.size()-1;a++)
    {
        boundRect[a] = boundingRect(contours[a]);
        for (int b = a+1;b<contours.size();b++)
        {
            double redpixel[1] = { 0 };
            double dark[1] = { 0 };
            boundRect[b] = boundingRect(contours[b]);
            double a1 = boundRect[a].x, a2 = boundRect[a].x + boundRect[a].width, a3 = boundRect[a].x, a4 = boundRect[a].x + boundRect[a].width;
            double b1 = boundRect[b].y, b2 = boundRect[b].y + boundRect[b].height, b3 = boundRect[b].y, b4 = boundRect[b].y + boundRect[b].height;
            double left = min(min(min(a1, a2), a3), a4);//左边界
            double right = max(max(max(a1, a2), a3), a4);//右边界
            double top = min(min(min(b1, b2), b3), b4);//上边界
            double bottom = max(max(max(b1, b2), b3), b4);//下边界
            box = Rect(left, top, right - left, bottom - top);//确定矩形大小

            center[a].x = boundRect[a].x + boundRect[a].width / 2;
            center[a].y = boundRect[a].y + boundRect[a].height / 2;
            center[b].x = boundRect[b].x + boundRect[b].width / 2;
            center[b].y = boundRect[b].y + boundRect[b].height / 2;

            //颜色检测与饱和度检测，查找框定矩形中间是否存在所需颜色与饱和度像素
            if (abs(center[b].y - center[a].y) < abs(center[b].x - center[a].x)*0.5)//夹角
            {
                for(int i=top;i<bottom;i++)
                {
                    uchar*tmph1data=Hue.ptr<uchar>(i);
                    uchar*tmph2data=Staturation.ptr<uchar>(i);
                    for(int j=left;j<right;j++)
                    {
                        if(tmph1data[j]==255)
                            redpixel[0]++;
                        if(tmph2data[j]==255)
                            dark[0]++;
                    }
                }
                //进行二值图的绘制，如存在上述像素，则根据亮度图进行绘制
                if(redpixel[0]>0&&dark[0]>0)
                {
                    if(boundRect[a].height>0.5*boundRect[a].width&&boundRect[b].height>0.5*boundRect[b].width)
                    {

                            for (int i = boundRect[a].y;i < boundRect[a].y + boundRect[a].height;i++)
                            {
                                uchar*tmph3data = Value.ptr<uchar>(i);
                                uchar*framethresholdData = framethreshold.ptr<uchar>(i);
                                for (int j = boundRect[a].x;j < boundRect[a].x + boundRect[a].width;j++)
                                {
                                    if (tmph3data[j] = 255)
                                        framethresholdData[j] = 255;
                                }
                            }

                            for (int i = boundRect[b].y;i < boundRect[b].y + boundRect[b].height;i++)
                            {
                                uchar*tmph3data = Value.ptr<uchar>(i);
                                uchar*framethresholdData = framethreshold.ptr<uchar>(i);
                                for (int j = boundRect[b].x;j < boundRect[b].x + boundRect[b].width;j++)
                                {
                                    if (tmph3data[j] = 255)
                                        framethresholdData[j] = 255;
                                }
                            }
                    }
                }
            }
        }
    }
imshow("framethreshold",framethreshold);
}

void ImagePreprocessor::wipePoints(Mat &framethreshold,Mat &hue,Mat &saturation)
{    
    //对已经绘制好的灰度图进行除噪，通过轮廓上点是否存在所需颜色与饱和度像素，轮廓面积进行去噪
    vector< vector<Point> >contours;
    findContours(framethreshold,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    vector<Rect> boundRect(contours.size());
    for(int a=0;a<contours.size();a++)
    {
        double H_out[1]={0};
        double S_out[1]={0};
        //轮廓面积
        double lightArea=contourArea(contours[a],false);
        boundRect[a] = boundingRect(contours[a]);
        for(int b=0;b<contours[a].size();b++)
        {
            Point outPoint;
            outPoint=Point(contours[a][b].x,contours[a][b].y);
            if(hue.at<uchar>(outPoint)==255)
            {
                H_out[0]++;
                if(saturation.at<uchar>(outPoint)==255)
                    S_out[0]++;
            }
        }

        //若不存在所需像素，则将像素变为黑色
        if((H_out[0]==0&&S_out[0]==0)||lightArea<5)
        {
            for (int i = boundRect[a].y;i < boundRect[a].y + boundRect[a].height;i++)
            {
                uchar*framethresholdData = framethreshold.ptr<uchar>(i);
                for (int j = boundRect[a].x;j < boundRect[a].x + boundRect[a].width;j++)
                    framethresholdData[j] = 0;
            }
        }
    }
}

void ImagePreprocessor::setThreshod(int channel, int minOrMax, int value)
{
    thresholds[channel][minOrMax] = value;
}

int ImagePreprocessor::getThreshod(int channel, int minOrMax) const
{
    return thresholds[channel][minOrMax];
}

Mat ImagePreprocessor::rangeThreshold(const Mat& srcImage, const int& channel)
{
    Mat result;
    threshold(srcImage, result, static_cast<double>(thresholds[channel][0]), 255, THRESH_BINARY);
    //threshold(srcImage, result, static_cast<double>(thresholds[channel][1]), 0, THRESH_BINARY);

    return result;
}
}

