#include "armour_detector.h"

namespace HCVC
{
ArmourDetector::ArmourDetector()
{
    FileStorage fs("statics/params.xml", FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "Open file failed" << endl;
    }

    FileNode node = fs["armour_detector"];

    node["angleRange"] >> params.angleRange;
    node["minArea"] >> params.minArea;
    node["maxHeightWidthRat"] >> params.maxHeightWidthRat;
    node["minHeightWidthRat"] >> params.minHeightWidthRat;

    node["inRangePercent"] >> params.inRangePercent;
    node["outRangePercent"] >> params.outRangePercent;
    node["armourPixelAvg"] >> params.armourPixelAvg;
    fs.release();
}

bool ArmourDetector::detect(const Mat& srcImage)
{
    Mat dstImage = preprocess(srcImage);

    clock_t A,B,C,D,a,b,c,d;

    a = clock();
    //存储初步找到的团块
    vector<vector<Point> > blocks = searchBlocks(dstImage.clone());
    A = clock();

    //检验数，配合数组，检测数组的实际长度
    int lampsNum = 0, armoursNum = 0;;

    b = clock();
    //存储找到的所有灯柱块
    vector<RotatedRect> lampBlocks = calcBlocksInfo(blocks, lampsNum);

    //将vector存储到数组中
    RotatedRect lamps[lampsNum];
    for(unsigned i = 0; i < lampBlocks.size(); i++)
        lamps[i] = lampBlocks[i];
    B = clock();

    //为中间结果显示准备图像
    Mat drawImage = srcImage.clone();

    //查看搜索出的每一个灯柱块
    drawVectorBlocks(drawImage, lampBlocks, Scalar(200, 150, 100));

    c = clock();
    //存储筛选过符合条件的所有对灯柱对最小包围矩形即装甲板区域
    RotatedRect armourBlocks[lampsNum];
    double average[lampsNum], standard[lampsNum];
    extracArmourBlocks(armourBlocks, lamps, srcImage, dstImage,
                       lampsNum, armoursNum, average, standard);
    C = clock();

    //查看搜索出的每一个独立的团块
    drawArrayBlocks(drawImage, armourBlocks, lampsNum, armoursNum, Scalar(100, 150, 200));

    if(armoursNum == 0)
    {
        return false;
    }

    //对每个装甲板区域评分

    d = clock();
    markArmourBlocks(srcImage, dstImage, armourBlocks, lampsNum, armoursNum, average, standard);
    D = clock();

    drawVectorBlocks(drawImage,
               vector<RotatedRect>(1, optimalArmourBlocks.front().block),
               Scalar(180, 200, 220));

//    cout<<"Run time: "<<(double)(A - a) / CLOCKS_PER_SEC<<"S"<<"\t"<<
//                        (double)(B - b) / CLOCKS_PER_SEC<<"S"<<"\t"<<
//                        (double)(C - c) / CLOCKS_PER_SEC<<"S"<<"\t"<<
//                        (double)(D - c) / CLOCKS_PER_SEC<<"S"<<"\t"<< endl;

    return true;
}

Rect2d ArmourDetector::getBestArmourBlock() const
{
    return optimalArmourBlocks.front().block.boundingRect();
}

void ArmourDetector::drawVectorBlocks(Mat srcImage,
                                const vector<RotatedRect>& minRotatedRects,
                                const Scalar& color) const
{
    for(unsigned int i = 0; i < minRotatedRects.size(); i++)
    {
        Point2f points[4];
        minRotatedRects[i].points(points);

        for(unsigned int j = 0; j < 4; j++)
        {
            line(srcImage, points[j], points[(j+1)%4], color, 2);
        }
    }

    imshow("detectBlocks", srcImage);
}

void ArmourDetector::drawArrayBlocks(Mat srcImage,
                     const RotatedRect* minRotatedRects,
                     int lampsNum,
                     int armoursNum,
                     const Scalar& color) const
{
    for(unsigned int i = 0; i < armoursNum; i++)
    {
        Point2f points[4];
        minRotatedRects[i].points(points);

        for(unsigned int j = 0; j < 4; j++)
        {
            line(srcImage, points[j], points[(j+1)%4], color, 2);
        }
    }
}

void ArmourDetector::fillLampBlock(Mat& srcImage,
                                   vector<vector<Point> >& blocks,
                                   int row,
                                   int col)
{
    //如果这个像素越界或者为零则不需要进一步访问它的相邻像素
    if(row < 0 || row >= srcImage.rows
               || col < 0 || col >= srcImage.cols
               || srcImage.at<uchar>(row, col) == 0)
    {
        return ;
    }

    //向当前正在填充的团块中加入数据，
    //注意点的坐标是（x，y），指针访问的坐标顺序是（y，x），x为横轴，y为纵轴

    blocks.back().push_back(Point(col, row));
/*
    qDebug() << blocks.back().size() << endl;
    long long count = 0;
    for(vector<vector<Point> >::iterator it = blocks.begin(); it != blocks.end(); it++)
    {
        count += it->size();
    }
    qDebug() << "total: " << count << endl;
*/

    //避免已访问像素的重复访问，将其置零
    srcImage.at<uchar>(row, col) = 0;

    for(int x = -1; x <= 1; x++)
    {
        for(int y = -1; y <= 1; y++)
        {
            //避免重复访问自身
            if(x == 0 && y == 0)
            {
                continue;
            }

            fillLampBlock(srcImage, blocks, row+x, col+y);
        }
    }
}

vector<vector<Point> > ArmourDetector::searchBlocks(Mat srcImage)
{
    vector<vector<Point> > blocks;

    int rowNum = srcImage.rows;
    int colNum = srcImage.cols;

    //如果图像在内存空间中存储是连续的，则把图像当作一行连续的矩阵来访问，提高访问效率
    if (srcImage.isContinuous())
    {
        rowNum = 1;
        colNum = colNum * srcImage.rows * srcImage.channels();
    }

    for (int row = 0; row < rowNum; row++)
    {
        uchar* srcImagePtr = srcImage.ptr<uchar>(row);

        for (int col = 0; col < colNum; col++)
        {
            //按行优先的顺序访问图像矩阵后，找到的每一个非零数据一定是一个新的独立团块的起点
            if(*srcImagePtr++)
            {
                //根据找到的起点，递归遍历所有它的相邻像素
                 blocks.push_back(vector<Point>());
                //由于存在两种不同的访问方式，需要进行坐标转换
                if(row == 0)
                {
                    fillLampBlock(srcImage, blocks, col/srcImage.cols, col%srcImage.cols);
                }
                else
                {
                    fillLampBlock(srcImage, blocks, row, col);
                }
            }
        }
    }

    return blocks;
}

vector<RotatedRect> ArmourDetector::calcBlocksInfo(const vector<vector<Point> >& blocks,
                                                   int& lampsNum)
{
    vector<RotatedRect> lampBlocks;

    if(blocks.size() != 0)
    {
        for(unsigned int i = 0; i < blocks.size(); i++)
        {
            RotatedRect minRotatedRect = minAreaRect(blocks[i]);

            if(minRotatedRect.size.area() > params.minArea)
            {
                if(minRotatedRect.angle > -45)
                {
                    if(minRotatedRect.size.height >
                            minRotatedRect.size.width*params.minHeightWidthRat)
                    {
                        lampBlocks.push_back(minRotatedRect);
                        lampsNum++;
                    }
                }
                else
                {
                    if(minRotatedRect.size.width >
                            minRotatedRect.size.height*params.minHeightWidthRat)
                    {
                        lampBlocks.push_back(minRotatedRect);
                        lampsNum++;
                    }
                }
            }
        }
    }

    return lampBlocks;
}

void ArmourDetector::extracArmourBlocks(RotatedRect* armourBlocks,
                                        const RotatedRect *lampBlocks,
                                        const Mat srcImage,
                                        const Mat dstImage,
                                        int lampsNum,
                                        int& armoursNum,
                                        double* average,
                                        double* standard)
{
    int initNum = 0;
    RotatedRect initArmourBlock;
    RotatedRect initLightBlocks[2];
    RotatedRect armourReserve[lampsNum];    

    //非空判定，如果为空的话在下面遍历的时候会出现一个bug，i-1溢出成2^32-1，使循环卡死
    if(lampsNum != 0)
    {
        for(unsigned int i = 0; i < lampsNum - 1; i++)
        {
            for(unsigned int j = i + 1; j < lampsNum; j++)
            {
                if(abs(lampBlocks[i].center.y - lampBlocks[j].center.y) <
                        0.4*abs(lampBlocks[i].center.x - lampBlocks[j].center.x))
                {
                    float angleI = min(abs(lampBlocks[i].angle), 90 - abs(lampBlocks[i].angle));
                    float angleJ = min(abs(lampBlocks[j].angle), 90 - abs(lampBlocks[j].angle));
                    if(abs(angleI - angleJ) < params.angleRange)
                    {
                        if(abs(lampBlocks[i].angle - lampBlocks[j].angle) < 45)
                        {
                            if((lampBlocks[i].boundingRect2f().area()>
                                0.2*lampBlocks[j].boundingRect2f().area())
                                    &&(lampBlocks[j].boundingRect2f().area()>
                                       0.2*lampBlocks[i].boundingRect2f().area()))
                            {
                                int labelValue = 0;

                                initLightBlocks[0] = lampBlocks[i];
                                initLightBlocks[1] = lampBlocks[j];

                                //外接正矩形连通域数量检测
                                domainCountDetect(initLightBlocks, dstImage, labelValue, 2);

                                if(labelValue == 2)
                                {
                                    initArmourBlock = getArmourRotated(initLightBlocks, 2);
                                    armourReserve[initNum] = initArmourBlock;
                                    initNum++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if(initNum != 0)
    {
        //计算甲板区间范围内的像素比例

        for(unsigned i = 0; i < initNum; i++)
        {
            double armourPixelAvg, inRangePercent, outRangePercent, armourStandard;

            calcDeviation(armourReserve[i], srcImage, dstImage,
                          armourPixelAvg, inRangePercent, outRangePercent, armourStandard);

            //根据像素的离散程度再次筛选甲板
            if(abs(armourPixelAvg) < params.armourPixelAvg)
            {
                armourBlocks[armoursNum] = armourReserve[i];
                average[armoursNum] = abs(armourPixelAvg);
                standard[armoursNum] = armourStandard;
                armoursNum++;
                cout<<"outRangePercent:"<<outRangePercent<<"\t"
                 <<"armourPixelAvg:"<<armourPixelAvg<<"\t"
                <<"armourStandard:"<<armourStandard<<endl;
            }
        }
    }
}

void ArmourDetector::domainCountDetect(const RotatedRect* initLightBlocks,
                                       const Mat& dstImage,
                                       int& labelValue,
                                       int lightNum)
{
    Mat labelImg = dstImage.clone();       

    int a1 = initLightBlocks[0].boundingRect2f().x,
        a2 = initLightBlocks[0].boundingRect2f().x
           + initLightBlocks[0].boundingRect2f().width,
        a3 = initLightBlocks[1].boundingRect2f().x,
        a4 = initLightBlocks[1].boundingRect2f().x
           + initLightBlocks[1].boundingRect2f().width;
    int b1 = initLightBlocks[0].boundingRect2f().y,
        b2 = initLightBlocks[0].boundingRect2f().y
           + initLightBlocks[0].boundingRect2f().height,
        b3 = initLightBlocks[1].boundingRect2f().y,
        b4 = initLightBlocks[1].boundingRect2f().y
           + initLightBlocks[1].boundingRect2f().height;

    int left = min(min(min(a1, a2), a3), a4);//左边界
    int right = max(max(max(a1, a2), a3), a4);//右边界
    int top = min(min(min(b1, b2), b3), b4);//上边界
    int bottom = max(max(max(b1, b2), b3), b4);//下边界
    int width = right - left, height = bottom - top;

    Point seed, neighbor;
    int rows = bottom;
    int cols = right;
    stack<Point> pointStack; // 堆栈

    //通过压栈计算框定区域内连通域数量    
    //矫正边界
    correctBorder(left, top, width, height, dstImage);

    int trebleHeight = 3*height;
    if(top + trebleHeight >= dstImage.rows){trebleHeight = dstImage.rows - top;}
    /*
    for (unsigned int i = top; i < rows; i++)
    {
        uchar* data = labelImg.ptr<uchar>(i);//获取一行的点
        for (unsigned int j = left; j < cols; j++)
        {
            if (data[j] == 255)
            {
                labelValue++; //不断将标签数加一
                seed = Point(j, i);// Point坐标
                labelImg.at<uchar>(seed) = labelValue;//标签
                pointStack.push(seed);//将像素seed压入栈，增加数据

                while (!pointStack.empty())//死循环，直到堆栈为空
                {
                    neighbor = Point(seed.x - 1, seed.y);//左像素
                    if ((seed.x != 0)
                            && (labelImg.at<uchar>(neighbor) == 255))
                    {
                        labelImg.at<uchar>(neighbor) = labelValue;
                        pointStack.push(neighbor);
                    }

                    neighbor = Point(seed.x + 1, seed.y);//右像素
                    if ((seed.x != (cols - 1))
                            && (labelImg.at<uchar>(neighbor) == 255))
                    {
                        labelImg.at<uchar>(neighbor) = labelValue;
                        pointStack.push(neighbor);
                    }

                    neighbor = Point(seed.x, seed.y - 1);//上像素
                    if ((seed.y != 0)
                            && (labelImg.at<uchar>(neighbor) == 255))
                    {
                        labelImg.at<uchar>(neighbor) = labelValue;
                        pointStack.push(neighbor);
                    }

                    neighbor = Point(seed.x, seed.y + 1);//下像素
                    if ((seed.y != (rows - 1))
                            && (labelImg.at<uchar>(neighbor) == 255))
                    {
                        labelImg.at<uchar>(neighbor) = labelValue;
                        pointStack.push(neighbor);
                    }
                    seed = pointStack.top();

                    //  获取堆栈上的顶部像素并将其标记为相同的标签
                    pointStack.pop();//弹出栈顶像素
                }
            }
        }
    }
    */

    vector<vector<Point> > contours;
    Mat roi = dstImage(Rect(left, top, width, trebleHeight));
    findContours(roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    labelValue = contours.size();
}

RotatedRect ArmourDetector::getArmourRotated(RotatedRect* initLightBlocks, int lightsNum)
{
    RotatedRect initArmourBlock;
    vector<Point> armourPoints;

    //获取两团块的最小外接矩形
    for(unsigned int m = 0; m < lightsNum; m++)
    {
        Point2f lightPoints[4];
        initLightBlocks[m].points(lightPoints);

        for(unsigned int n = 0; n < 4; n++)
            armourPoints.push_back(lightPoints[n]);
    }

    initArmourBlock = minAreaRect(armourPoints);

    return initArmourBlock;
}

void ArmourDetector::calcDeviation(const RotatedRect armourReserve,
                                   const Mat& srcImage,
                                   const Mat& dstImage,
                                   double& armourPixelAvg,
                                   double& inRangePercent,
                                   double& outRangePercent,
                                   double& armourStandard)
{
    Mat gray = Mat (srcImage.rows, srcImage.cols,CV_8UC1);
    cvtColor(srcImage,gray,COLOR_BGR2GRAY);

    Mat framethreshold=dstImage.clone();

    double sum=0;//像素值的总和
    double armourPixelCount = 0;//甲板像素数量
    double armourRangPixel = 0;//所需区间内像素
    double notArmourRangPixel = 0;//远离甲板平均值像素
    armourPixelAvg = 0;//像素的平均值
    inRangePercent = 0;//区间范围内像素所占比例
    outRangePercent = 0;//区间外像素所占比例

    int left = armourReserve.boundingRect2f().x,        
        top = armourReserve.boundingRect2f().y,
        width = armourReserve.boundingRect2f().width,
        height = armourReserve.boundingRect2f().height;

    float maxPixel = 0, minPixel = 255;
//    int total[256];
//    for(unsigned i = 0; i < 256; i++)
//        total[i] = 0;

    //矫正边界
    correctBorder(left, top, width, height, dstImage);

    for(unsigned int i = top; i < top + height; i++)
    {
        uchar* grayData = gray.ptr<uchar>(i);//灰度图像素
        uchar* framethresholdData = framethreshold.ptr<uchar>(i);//二值化图像素
        for (unsigned int j = left; j < left + width; j++)
        {
            if (framethresholdData[j] == 0)//非灯条像素
            {
                sum += grayData[j];
                if(grayData[j] >= maxPixel)
                    maxPixel = grayData[j];
                if(grayData[j] <= minPixel)
                    minPixel = grayData[j];

                armourPixelCount++;
                //total[grayData[j]]++;
            }
        }
    }

//    unsigned int mode = 0;
//    unsigned int number = total[0];
//    for(unsigned int i = 1; i < 256; i++)
//    {
//        if(total[i] > number)
//        {
//            number = total[i];
//            mode = i;
//        }
//    }

     armourPixelAvg = (sum - armourPixelCount*minPixel)/armourPixelCount;

     //求甲板像素给定区间范围内像素值与标准差
     for (unsigned int i = top; i < top + height; i++)
     {
         uchar* grayData = gray.ptr<uchar>(i);
         uchar* framethresholdData = framethreshold.ptr<uchar>(i);//二值化图像素
         for (unsigned int j = left; j < left + width; j++)
         {
             if (framethresholdData[j] == 0)//非灯条像素
             {
                 armourStandard = sqrt(pow(abs((grayData[j] - minPixel)) - armourPixelAvg, 2)
                                 /armourPixelCount);
                 if (grayData[j] - minPixel > 2.5*armourPixelAvg)
                     notArmourRangPixel++;
             }
         }
     }

     //armourPixelAvg = armourPixelAvg/(maxPixel - minPixel)*255;
     //inRangePercent = armourRangPixel / armourPixelCount;
     outRangePercent = (notArmourRangPixel / armourPixelCount)*100;
}

void ArmourDetector::markArmourBlocks(const Mat& srcImage,
                                      const Mat& dstImage,
                                      const RotatedRect* armourBlocks,
                                      int lampsNum,
                                      int armoursNum,
                                      double* average,
                                      double* standard)
{
    //清除之前运算的结果
    optimalArmourBlocks.clear();

    //通过评分选出最优装甲板
    if(armoursNum > 1)
    {
        //根据方差找到最近的两个装甲板区域
        RotatedRect nearRotated[2];
        double frontStd[2], frontAvg[2];
        if(standard[0] > standard[1])
        {
            frontStd[0] = standard[0]; nearRotated[0] = armourBlocks[0];
            frontAvg[0] = abs(average[0]);
            frontStd[1] = standard[1]; nearRotated[1] = armourBlocks[1];
            frontAvg[1] = abs(average[1]);
        }
        else
        {
            frontStd[0] = standard[1]; nearRotated[0] = armourBlocks[1];
            frontAvg[0] = abs(average[1]);
            frontStd[1] = standard[0]; nearRotated[1] = armourBlocks[0];
            frontAvg[1] = abs(average[0]);
        }

        if(armoursNum >= 3)
        {
            for(unsigned int i = 3; i < armoursNum; i++)
            {
                if(standard[i] < frontStd[1])
                {
                    frontStd[0] = frontStd[1]; nearRotated[0] = nearRotated[1];
                    frontAvg[0] = abs(frontAvg[1]);
                    frontStd[1] = standard[i]; nearRotated[1] = armourBlocks[i];
                    frontAvg[1] = abs(average[i]);
                }
                if(standard[i] < frontStd[0] && standard[i] > frontStd[1])
                {
                    frontStd[0] = standard[i]; nearRotated[0] = armourBlocks[i];
                    frontAvg[0] = abs(average[i]);
                }
            }
        }

        for(unsigned int id = 0; id < 2; id++)
        {

            //剪去旋转矩形的多余边角，得到装甲板的平行四边形区域
            //cutEdgeOfRect(fpoints);

            double shortEdge = min(nearRotated[id].size.height, nearRotated[id].size.width);
            double longEdge = max(nearRotated[id].size.height, nearRotated[id].size.width);

            float angle = min(abs(nearRotated[id].angle), 90 - abs(nearRotated[id].angle));

            //长宽比与离散系数乘积去除错误错误匹配并判别多辆车远近
            double grade = (sin(angle) + 1)*frontAvg[id]*standard[id];

            //imshow("mask", mask);

            optimalArmourBlocks.push_back(OptimalArmourBlock(nearRotated[id], grade));
        }

        //将装甲板区域按分从小到大排序，找出最佳区域
        sort(optimalArmourBlocks.begin(), optimalArmourBlocks.end());
    }

    else if(armoursNum == 1)
    {
        optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[0], abs(average[0])));
        sort(optimalArmourBlocks.begin(), optimalArmourBlocks.end());
    }
}

void ArmourDetector::correctBorder(int& left, int& top, int& width, int& height, Mat image)
{
    int leftClone = left, topClone = top;

    if(left < 0){left = 0; width += leftClone;}
    if(left + width > image.cols){width = image.cols - leftClone;}
    if(top < 0){top = 0; height += topClone;}
    if(top + height > image.rows){height = image.rows - topClone;}
}

void ArmourDetector::cutEdgeOfRect(Point2f* points)
{
    //求出四个点的横坐标中点
    float centerx = 0;
    for(unsigned int i = 0; i < 4; i++)
    {
        centerx += points[i].x;
    }
    centerx /= 4;

    //通过横坐标中点将这组点分为左右两对
    vector<Point2f> leftPoints;
    vector<Point2f> rightPoints;
    for(unsigned int i = 0; i < 4; i++)
    {
        if(points[i].x < centerx)
        {
            leftPoints.push_back(points[i]);
        }
        else
        {
            rightPoints.push_back(points[i]);
        }
    }

    //组内分别按高度排序，方便之后处理
    if(leftPoints[0].y < leftPoints[1].y)
    {
        reverse(leftPoints.begin(), leftPoints.end());
    }

    if(rightPoints[0].y < rightPoints[1].y)
    {
        reverse(rightPoints.begin(), rightPoints.end());
    }

    //如果左边这对比右边高，则矩形为倒向左侧状态，否则为倒向右侧状态
    if(leftPoints[1].y > rightPoints[1].y)
    {
        Point2f newPoint;

        //两条直线相交的交点
        newPoint.x = leftPoints[0].x;
        newPoint.y = (leftPoints[1].y-rightPoints[1].y)/(leftPoints[1].x-rightPoints[1].x)
                * (leftPoints[0].x-rightPoints[1].x) + rightPoints[1].y;
        leftPoints[1] = newPoint;

        newPoint.x = rightPoints[1].x;
        newPoint.y = (leftPoints[0].y-rightPoints[0].y)/(leftPoints[0].x-rightPoints[0].x)
                * (rightPoints[1].x-leftPoints[0].x) + leftPoints[0].y;
        rightPoints[0] = newPoint;
    }
    else
    {
        Point2f newPoint;

        //两条直线相交的交点
        newPoint.x = leftPoints[1].x;
        newPoint.y = (leftPoints[0].y-rightPoints[0].y)/(leftPoints[0].x-rightPoints[0].x)
                * (leftPoints[1].x-rightPoints[0].x) + rightPoints[0].y;
        leftPoints[0] = newPoint;

        newPoint.x = rightPoints[0].x;
        newPoint.y = (leftPoints[1].y-rightPoints[1].y)/(leftPoints[1].x-rightPoints[1].x)
                * (rightPoints[0].x-leftPoints[1].x) + leftPoints[1].y;
        rightPoints[1] = newPoint;
    }

    //拼接两对点
    points[0] = leftPoints[0];
    points[1] = leftPoints[1];
    points[2] = rightPoints[1];
    points[3] = rightPoints[0];
}
}
