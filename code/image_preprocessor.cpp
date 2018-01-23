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

    //开运算去除小的噪声点，闭运算连接断开部分，对H进行处理
    Mat hue, saturation, value;
    hsvImages[2].convertTo(value, CV_8UC1);
    morphologyEx(hsvImages[0], hue, MORPH_OPEN,kernel_1);
    morphologyEx(hue, hue, MORPH_CLOSE, kernel_2);

    //中值滤波，去除S通道噪声点
    medianBlur(hsvImages[1],saturation,3);
    //morphologyEx(hsvImages[1], saturation, MORPH_OPEN, kernel_1);
    //morphologyEx(saturation, saturation, MORPH_CLOSE, kernel_2);

    //初始化二值化图
    Mat framethreshold = Mat(value.size(), CV_8UC1,Scalar(0));

    //根据三个通道初步绘制二值化图
    threshProcess(framethreshold, hue, saturation, value);

    //中值滤波去除噪声点，同时使灯柱边缘润滑
    medianBlur(framethreshold, framethreshold,3);

    //水平方向连接一些断开的团块，防止运动模糊产生重影而加大检测计算量
    Mat kernel_3 = getStructuringElement(MORPH_RECT, Size(2,1));
    morphologyEx(framethreshold, framethreshold, MORPH_CLOSE, kernel_3);

    //根据团块外接轮廓的H与S通道去噪
    //wipePoints(framethreshold, hue, saturation);
    //中值滤波

    //显示单通道处理后图像
    imshow("hImage", hue);
    imshow("SImage", saturation);
    imshow("VImage", value);

    //显示预处理后图像
    imshow("result", framethreshold);

    return framethreshold;
}

void ImagePreprocessor::threshProcess(Mat& framethreshold,
                                      Mat& hue,
                                      Mat& saturation,
                                      Mat& value)
{
    //查找轮廓，只检索最外面的轮廓，将所有的连码点转换成点
    vector<vector<Point> > contours;//定义一个返回轮廓的容器
    findContours(value, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //轮廓的最小外接矩形
    vector<Rect> boundRect(contours.size());

    for(unsigned int a = 0; a < contours.size(); a++)
    {
        boundRect[a] = boundingRect(contours[a]);

        unsigned int huePixel = 0;
        unsigned int saturationPixel = 0;
        float left = boundRect[a].x - boundRect[a].width,
              right = boundRect[a].x + 2*boundRect[a].width,
              top = boundRect[a].y - boundRect[a].height/2,
              bottom = boundRect[a].y + 2*boundRect[a].height/2;

        //检测亮度通道团块在H与S通道周围像素情况，存在所设定阈值像素则判定为灯柱
        if(left > 0 && right < framethreshold.cols
                && top > 0 && bottom < framethreshold.rows)
        {
            for(int i = top; i < bottom; i++)
            {
                uchar* hueData = hue.ptr<uchar>(i);
                uchar* saturationData = saturation.ptr<uchar>(i);
                for(int j = left; j < right; j++)
                {
                    if(hueData[j] == 255)
                        huePixel++;
                    if(saturationData[j] == 255)
                        saturationPixel++;
                }
            }
        }

        //根据亮度图团块进行二值图的绘制
        if(huePixel > 0 && saturationPixel > 0)
        {
            for (unsigned int i = boundRect[a].y;
                 i < boundRect[a].y + boundRect[a].height;
                 i++)
            {
                uchar* valueData = value.ptr<uchar>(i);
                uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                for (unsigned int j = boundRect[a].x;
                     j < boundRect[a].x + boundRect[a].width;
                     j++)
                {
                        if (valueData[j] == 255)
                            framethresholdData[j] = 255;
                }
            }
        }
    }
}

void ImagePreprocessor::wipePoints(Mat& framethreshold, Mat& hue, Mat& saturation)
{    
    //对已经绘制好的灰度图进行除噪，通过轮廓上点是否存在所需颜色与饱和度像素，轮廓面积进行去噪
    vector<vector<Point> >contours;
    findContours(framethreshold, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    vector<Rect> boundRect(contours.size());
    for(unsigned int a=0; a<contours.size(); a++)
    {
        float H_out[1] = {0};
        float S_out[1] = {0};
        //轮廓面积
        float lightArea = contourArea(contours[a], false);
        boundRect[a] = boundingRect(contours[a]);
        for(int b = 0; b < contours[a].size(); b++)
        {
            Point outPoint;
            outPoint = Point(contours[a][b].x, contours[a][b].y);
            if(hue.at<uchar>(outPoint) == 255)
            {
                H_out[0]++;
                if(saturation.at<uchar>(outPoint) == 255)
                    S_out[0]++;
            }
        }

        //若不存在所需像素，则将像素变为黑色
        if((H_out[0] == 0 && S_out[0] == 0) || lightArea < 5)
        {
            for(unsigned int i = boundRect[a].y; i < boundRect[a].y + boundRect[a].height; i++)
            {
                uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                for (unsigned int j = boundRect[a].x; j < boundRect[a].x + boundRect[a].width; j++)
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

