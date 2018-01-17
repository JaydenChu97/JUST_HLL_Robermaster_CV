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
        cout << int(node[2*i]) << endl;
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
    Mat dilateKernel = getStructuringElement(MORPH_RECT, Size(15, 15));
    Mat closeKernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    //腐蚀，消除离散点
    //erode(hsvImages[1], hsvImages[1], erodeKernel);
    //膨胀，增大光柱区域
    dilate(hsvImages[0], hsvImages[0], dilateKernel);
    //中值滤波，去除椒盐噪声
    //medianBlur(hsvImages[2], hsvImages[2], 5);
//    //显示装甲板轮廓
//    Laplacian(hsvImages[1], hsvImages[1], CV_8UC1, 3, 1);
    //逻辑与操作，过滤出小车灯柱
    bitwise_and(hsvImages[0], hsvImages[2], dstImage);
    //闭运算，将断裂的灯柱图像连接起来
    morphologyEx(dstImage, dstImage, MORPH_CLOSE, closeKernel);

    //显示单通道处理后图像
    imshow("hImage", hsvImages[0]);
    imshow("SImage", hsvImages[1]);
    imshow("VImage", hsvImages[2]);

    //显示预处理后图像
    imshow("result", dstImage);

    return dstImage;
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
    threshold(srcImage, result, static_cast<double>(thresholds[channel][0]), 0, THRESH_TOZERO);
    threshold(result, result, static_cast<double>(thresholds[channel][1]), 0, THRESH_TOZERO_INV);

    return result;
}
}

