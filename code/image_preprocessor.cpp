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
    FileNode red_node = fs["red_image_preprocessor_threshod"];
    FileNode blue_node = fs["blue_image_preprocessor_threshod"];

    color = 0;

    if(color == 0)
    {
        for(unsigned int i = 0; i < 3; i++)
        {
            //cout << int(red_node[2*i]) << endl;
            thresholds[i].push_back(int(red_node[2*i]));
            thresholds[i].push_back(int(red_node[2*i+1]));
        }
    }
    else
    {
        for(unsigned int i = 0; i < 3; i++)
        {
            //cout << int(red_node[2*i]) << endl;
            thresholds[i].push_back(int(blue_node[2*i]));
            thresholds[i].push_back(int(blue_node[2*i+1]));
        }
    }
}

Mat ImagePreprocessor::preprocess(const Mat& srcImage)
{
    Mat dstImage;   

    clock_t begin, end;
    begin = clock();
    //将bgr格式(opencv默认将彩色图片存储为bgr格式)图像转变为hsv格式
    cvtColor(srcImage, dstImage, CV_BGR2HSV);
    //分离图像三通道
    Mat hsvImages[3];
    split(dstImage, hsvImages);

    hsvImages[0] = rangeThreshold(hsvImages[0], 0);
    hsvImages[1] = rangeThreshold(hsvImages[1], 1);
    hsvImages[2] = rangeThreshold(hsvImages[2], 2);

    Mat hue, saturation, value;    

    //中值滤波，去除S通道噪声点
    medianBlur(hsvImages[0], hue, 3);
    medianBlur(hsvImages[1], saturation,3);
    medianBlur(hsvImages[2], value, 5);

    //闭操作，去除H通道噪声点，以水平方向为主膨胀H,S通道像素
    Mat kernel_1 = getStructuringElement(MORPH_RECT, Size(4,1));
    Mat kernel_2 = getStructuringElement(MORPH_RECT, Size(5,2));
    erode(hue, hue, kernel_1);
    dilate(hue, hue, kernel_2);
    dilate(saturation, saturation, kernel_1);

    //初始化二值化图
    Mat framethreshold = Mat(value.size(), CV_8UC1,Scalar(0));

    //根据三个通道绘制二值化图
    if(color == 0)
        redThreshProcess(srcImage, framethreshold, hue, saturation, value);
    else
        blueThreshProcess(srcImage, framethreshold, hue, saturation, value);

    //中值滤波去除噪声点，同时使灯柱边缘润滑
    medianBlur(framethreshold, framethreshold, 3);

    //水平,竖直方向连接一些连接的团块，防止运动模糊产生重影
    Mat kernel_3 = getStructuringElement(MORPH_RECT, Size(4,1));
    morphologyEx(framethreshold, framethreshold, MORPH_CLOSE, kernel_3);

    end = clock();
    //cout<<"imageProcessTime:"<<double(end - begin)/CLOCKS_PER_SEC<<"s"<<"\t"<<endl;

    //显示单通道处理后图像
    imshow("hImage", hue);
    imshow("SImage", saturation);
    imshow("VImage", value);

    //显示预处理后图像
    imshow("result", framethreshold);

    return framethreshold;
}

void ImagePreprocessor::redThreshProcess(const Mat& srcImage,
                                      Mat& framethreshold,
                                      Mat& hue,
                                      Mat& saturation,
                                      Mat& value)
{
    //查找轮廓，只检索最外面的轮廓，将所有的连码点转换成点
    vector<vector<Point> > contours;//定义一个返回轮廓的容器
    findContours(value, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //轮廓的最小外接矩形
    Rect boundRect[contours.size()];
    RotatedRect rotatedRect[contours.size()];

    if(contours.size() != 0 )
    {
        for(unsigned int a = 0; a < contours.size(); a++)
        {
            rotatedRect[a] = minAreaRect(contours[a]);
            boundRect[a] = rotatedRect[a].boundingRect2f();

            if(boundRect[a].height < 9*boundRect[a].width
                    && boundRect[a].height > 0.8*boundRect[a].width)
            {
                if(abs(rotatedRect[a].angle) < 30 || abs(rotatedRect[a].angle) > 60)
                {                    
                    //计算团块周边红绿特征
                    float hueContourPixels = 0;
                    unsigned int contoursRedSum = 0;
                    unsigned int contoursBlueSum = 0;
                    unsigned int contoursArea = contourArea(contours[a], false);

                    if(1.4*contoursArea > rotatedRect[a].size.area())
                    {
                        for(double b = 0; b < contours[a].size(); b++)
                        {
                            contoursRedSum += srcImage.at<Vec3b>(contours[a][b])[2];
                            contoursBlueSum += srcImage.at<Vec3b>(contours[a][b])[0];
                            if(hue.at<uchar>(contours[a][b]) == 255)
                               hueContourPixels++;
                        }
                        unsigned int redAvg = contoursRedSum/contours[a].size();
                        unsigned int blueAvg = contoursBlueSum/contours[a].size();
                        //cout<<"redAvg:"<<redAvg<<"\t"<<"blueAvg:"<<blueAvg<<endl;
                        //cout<<"hueContourPixels:"<<hueContourPixels/contours[a].size()<<endl;

                        if(redAvg > blueAvg)
                        {
                            unsigned int huePixel = 0;
                            unsigned int saturationPixel = 0;
                            float left = boundRect[a].x - 6,
                                  right = boundRect[a].x + boundRect[a].width + 6,
                                  top = boundRect[a].y,
                                  bottom = boundRect[a].y + boundRect[a].height;

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
                            if(saturationPixel > 0)
                            {
                                if(huePixel > 0
                                   && contoursArea > 15)
                                {
                                    for (int i = boundRect[a].y;
                                         i < boundRect[a].y + boundRect[a].height;
                                         i++)
                                    {
                                        uchar* valueData = value.ptr<uchar>(i);
                                        uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                                        for (int j = boundRect[a].x;
                                             j < boundRect[a].x + boundRect[a].width;
                                             j++)
                                        {
                                                if (valueData[j] == 255)
                                                    framethresholdData[j] = 255;
                                        }
                                    }
                                }
                            }
                            else if(huePixel > 0
                                    && hueContourPixels/contours[a].size()>0.03
                                    && contoursArea > 20
                                    && boundRect[a].height > 1.5*boundRect[a].width)
                            {
                                for (int i = boundRect[a].y;
                                     i < boundRect[a].y + boundRect[a].height;
                                     i++)
                                {
                                    uchar* valueData = value.ptr<uchar>(i);
                                    uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                                    for (int j = boundRect[a].x;
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
                }
            }
        }
    }
}

void ImagePreprocessor::blueThreshProcess(const Mat& srcImage,
                                          Mat& framethreshold,
                                          Mat& hue,
                                          Mat& saturation,
                                          Mat& value)
{
    //查找轮廓，只检索最外面的轮廓，将所有的连码点转换成点
    vector<vector<Point> > contours;//定义一个返回轮廓的容器
    findContours(value, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //轮廓的最小外接矩形
    Rect boundRect[contours.size()];
    RotatedRect rotatedRect[contours.size()];

    if(contours.size() != 0 )
    {
        for(unsigned int a = 0; a < contours.size(); a++)
        {
            rotatedRect[a] = minAreaRect(contours[a]);
            boundRect[a] = rotatedRect[a].boundingRect2f();

            if(boundRect[a].height < 9*boundRect[a].width
                    && boundRect[a].height > 0.8*boundRect[a].width)
            {
                if(abs(rotatedRect[a].angle) < 30 || abs(rotatedRect[a].angle) > 60)
                {

                    //计算团块周边红绿特征
                    float hueContourPixels = 0;
                    unsigned int contoursRedSum = 0;
                    unsigned int contoursBlueSum = 0;
                    unsigned int contoursArea = contourArea(contours[a], false);

                    if(1.4*contoursArea > rotatedRect[a].size.area())
                    {
                        for(double b = 0; b < contours[a].size(); b++)
                        {
                            contoursRedSum += srcImage.at<Vec3b>(contours[a][b])[2];
                            contoursBlueSum += srcImage.at<Vec3b>(contours[a][b])[0];
                            if(hue.at<uchar>(contours[a][b]) == 255)
                               hueContourPixels++;
                        }
                        unsigned int redAvg = contoursRedSum/contours[a].size();
                        unsigned int blueAvg = contoursBlueSum/contours[a].size();
                        //cout<<"redAvg:"<<redAvg<<"\t"<<"blueAvg:"<<blueAvg<<endl;
                        //cout<<"hueContourPixels:"<<hueContourPixels/contours[a].size()<<endl;

                        if(redAvg < blueAvg)
                        {
                            unsigned int huePixel = 0;
                            unsigned int saturationPixel = 0;
                            float left = boundRect[a].x - 6,
                                  right = boundRect[a].x + boundRect[a].width + 6,
                                  top = boundRect[a].y,
                                  bottom = boundRect[a].y + boundRect[a].height;

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
                            if(saturationPixel > 0)
                            {
                                if(huePixel > 0
                                   && contoursArea > 15)
                                {
                                    for (int i = boundRect[a].y;
                                         i < boundRect[a].y + boundRect[a].height;
                                         i++)
                                    {
                                        uchar* valueData = value.ptr<uchar>(i);
                                        uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                                        for (int j = boundRect[a].x;
                                             j < boundRect[a].x + boundRect[a].width;
                                             j++)
                                        {
                                                if (valueData[j] == 255)
                                                    framethresholdData[j] = 255;
                                        }
                                    }
                                }
                            }
                            else if(huePixel > 0
                                    && hueContourPixels/contours[a].size()>0.03
                                    && contoursArea > 20
                                    && boundRect[a].height > 1.5*boundRect[a].width)
                            {
                                for (int i = boundRect[a].y;
                                     i < boundRect[a].y + boundRect[a].height;
                                     i++)
                                {
                                    uchar* valueData = value.ptr<uchar>(i);
                                    uchar* framethresholdData = framethreshold.ptr<uchar>(i);
                                    for (int j = boundRect[a].x;
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
                }
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
    //threshold(srcImage, result, static_cast<double>(thresholds[channel][1]), 255, THRESH_BINARY_IVN);

    return result;
}
}

