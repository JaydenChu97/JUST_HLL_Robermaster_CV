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

    //存储初步找到的团块
    vector<vector<Point> > blocks = searchBlocks(dstImage.clone());

    //存储找到的所有灯柱块
    vector<RotatedRect> lampBlocks = calcBlocksInfo(blocks);

    //为中间结果显示准备图像
    Mat drawImage = srcImage.clone();

    //查看搜索出的每一个灯柱块
    drawBlocks(drawImage, lampBlocks, Scalar(200, 150, 100));

    //存储筛选过符合条件的所有对灯柱对最小包围矩形即装甲板区域
    vector<RotatedRect> armourBlocks =
            extracArmourBlocks(lampBlocks, srcImage, dstImage);

    //查看搜索出的每一个独立的团块
    drawBlocks(drawImage, armourBlocks, Scalar(100, 150, 200));

    if(armourBlocks.empty())
    {
        return false;
    }

    //对每个装甲板区域评分

    markArmourBlocks(srcImage, dstImage, armourBlocks);

    drawBlocks(drawImage,
               vector<RotatedRect>(1, optimalArmourBlocks.front().block),
               Scalar(180, 200, 220));

    if(armourBlocks.empty())
    {
        return false;
    }

    return true;
}

Rect2d ArmourDetector::getBestArmourBlock() const
{
    return optimalArmourBlocks.front().block.boundingRect();
}

void ArmourDetector::drawBlocks(Mat srcImage,
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

vector<RotatedRect> ArmourDetector::calcBlocksInfo(const vector<vector<Point> >& blocks)
{
    vector<RotatedRect> lampBlocks;

    for(unsigned int i = 0; i < blocks.size(); i++)
    {
        RotatedRect minRotatedRect = minAreaRect(blocks[i]);

        if(minRotatedRect.size.area() > params.minArea)
        {
            if(minRotatedRect.angle > -45)
            {
                if(minRotatedRect.size.height/minRotatedRect.size.width > params.minHeightWidthRat)
                   lampBlocks.push_back(minRotatedRect);
            }
            else
            {
                if(minRotatedRect.size.width/minRotatedRect.size.height > params.minHeightWidthRat)
                    lampBlocks.push_back(minRotatedRect);
            }
        }
    }

    return lampBlocks;
}

vector<RotatedRect> ArmourDetector::extracArmourBlocks(const vector<RotatedRect>& lampBlocks,
                                                       const Mat srcImage,
                                                       const Mat dstImage)
{
    vector<RotatedRect> armourBlocks;

    //非空判定，如果为空的话在下面遍历的时候会出现一个bug，i-1溢出成2^32-1，使循环卡死
    if(lampBlocks.empty())
    {
        return armourBlocks;
    }

    for(unsigned int i = 0; i < lampBlocks.size() - 1; i++)
    {
        for(unsigned int j = i + 1; j < lampBlocks.size(); j++)
        {
            if(abs(lampBlocks[i].center.y - lampBlocks[j].center.y) <
                    0.58*abs(lampBlocks[i].center.x-lampBlocks[j].center.x))
            {
                float angleI = min(abs(lampBlocks[i].angle), 90 - abs(lampBlocks[i].angle));
                float angleJ = min(abs(lampBlocks[j].angle), 90 - abs(lampBlocks[j].angle));
                if(abs(angleI - angleJ) < params.angleRange)
                {
                    if((lampBlocks[i].boundingRect2f().area()>
                        0.2*lampBlocks[j].boundingRect2f().area())
                            &&(lampBlocks[j].boundingRect2f().area()>
                               0.2*lampBlocks[i].boundingRect2f().area()))
                    {
                        vector<RotatedRect> initLightBlocks;
                        initLightBlocks.push_back(lampBlocks[i]);
                        initLightBlocks.push_back(lampBlocks[j]);

                        //计算甲板区间范围内的像素比例
                        double armourPixelAvg, variance, inRangePercent, outRangePercent;
                        calcDeviation(initLightBlocks,
                                      srcImage,dstImage,
                                      armourPixelAvg,
                                      variance,
                                      inRangePercent,
                                      outRangePercent);

                        //float product = pow(1/(4*outRangePercent)*inRangePercent,2);

                        //根据像素的离散程度再次筛选甲板
                        if(inRangePercent > params.inRangePercent
                                &&outRangePercent < params.outRangePercent
                                &&armourPixelAvg < params.armourPixelAvg)
                        {

//                            cout<<"inRangePercent:"<<inRangePercent<<"\t"
//                              <<"outRangePercent:"<<outRangePercent<<"\t"
//                             <<"armourPixelAvg:"<<armourPixelAvg<<"\t"
//                            <<"variance:"<<variance<<endl;

                            vector<RotatedRect> finalLightBlocks;

                            //外接正矩形连通域数量检测
                            vector<RotatedRect>initArmourBlocks =
                                    domainCountDetect(initLightBlocks, finalLightBlocks, dstImage);

                            if(initArmourBlocks.size() != 0)
                            {
                                for(unsigned int k=0; k < initArmourBlocks.size(); k++)
                                {
                                    armourBlocks.push_back(initArmourBlocks[k]);
                                }
                            }
                       }
                    }
                }
            }
        }
    }
    //cout << "Num of lampBlocksRects: " << armourBlocks.size() << endl;
    return armourBlocks;
}

void ArmourDetector::calcDeviation(vector<RotatedRect> initLightBlocks,
                                   const Mat& srcImage,
                                   const Mat& dstImage,
                                   double& armourPixelAvg,
                                   double& variance,
                                   double& inRangePercent,
                                   double& outRangePercent)
{
    Mat gray = Mat (srcImage.rows, srcImage.cols,CV_8UC1);
    cvtColor(srcImage,gray,COLOR_BGR2GRAY);

    Mat framethreshold=dstImage.clone();

    double sum=0;//像素值的总和
    double armourPixelCount = 0;//甲板像素数量
    double armourRangPixel = 0;//所需区间内像素
    double notArmourRangPixel = 0;//远离甲板平均值像素
    armourPixelAvg = 0;//像素的平均值
    variance = 0;//甲板像素方差
    inRangePercent = 0;//区间范围内像素所占比例
    outRangePercent = 0;//区间外像素所占比例

    for (unsigned int i = 0; i < initLightBlocks.size()-1; i++)
    {
        for (unsigned int j = i+1; j < initLightBlocks.size(); j++)
        {
            int a1=initLightBlocks[i].boundingRect2f().x,
                a2=initLightBlocks[i].boundingRect2f().x+initLightBlocks[i].boundingRect2f().width,
                a3=initLightBlocks[j].boundingRect2f().x,
                a4=initLightBlocks[j].boundingRect2f().x+initLightBlocks[j].boundingRect2f().width;
            int b1=initLightBlocks[i].boundingRect2f().y,
                b2=initLightBlocks[i].boundingRect2f().y+initLightBlocks[i].boundingRect2f().height,
                b3=initLightBlocks[j].boundingRect2f().y,
                b4=initLightBlocks[j].boundingRect2f().y+initLightBlocks[j].boundingRect2f().height;

            int left = min(min(min(a1, a2), a3), a4);//左边界
            int right = max(max(max(a1, a2), a3), a4);//右边界
            int top = min(min(min(b1, b2), b3), b4);//上边界
            int bottom = max(max(max(b1, b2), b3), b4);//下边界

            if(left > 0 && right < framethreshold.cols
                    && top > 0 && bottom < framethreshold.rows)
            {
                for(unsigned int i = top; i < bottom; i++)
                {
                    uchar* grayData = gray.ptr<uchar>(i);//灰度图像素
                    uchar* framethresholdData = framethreshold.ptr<uchar>(i);//二值化图像素
                    for (unsigned int j = left; j < right; j++)
                    {
                        if (framethresholdData[j] == 0)//非灯条像素
                        {
                            sum += grayData[j];
                            armourPixelCount++;
                        }
                    }
                }
                 armourPixelAvg = sum/armourPixelCount;

                 //求甲板像素给定区间范围内与外像素值
                 int range = (floor(armourPixelAvg)+ceil(armourPixelAvg))/2;
                 for (unsigned int i = top; i < bottom; i++)
                 {
                     uchar* grayData = gray.ptr<uchar>(i);
                     uchar* framethresholdData = framethreshold.ptr<uchar>(i);//二值化图像素
                     for (unsigned int j = left; j < right; j++)
                     {
                         if (framethresholdData[j] == 0)//非灯条像素
                         {
                             variance += pow(grayData[j]-armourPixelAvg, 2);
                             if (grayData[j] < range + 10 && grayData[j] > range - 10)
                                 armourRangPixel++;
                             if (grayData[j] > range + 15)
                                 notArmourRangPixel++;
                         }
                     }
                 }
                 variance = variance/armourPixelCount;
                 inRangePercent = armourRangPixel / armourPixelCount;
                 outRangePercent=notArmourRangPixel / armourPixelCount;
            }
        }
    }
}

vector<RotatedRect> ArmourDetector::domainCountDetect(const vector<RotatedRect> &initLightBlocks,
                                                      vector<RotatedRect> &finalLightBlocks,
                                                      const Mat& dstImage)
{

    vector<RotatedRect> initArmourBlock;

    Mat labelImg = dstImage.clone();

    for (unsigned int i = 0; i < initLightBlocks.size()-1; i++)
    {
        for (unsigned int j = i+1; j < initLightBlocks.size(); j++)
        {
            int a1 = initLightBlocks[i].boundingRect2f().x,
                a2 = initLightBlocks[i].boundingRect2f().x
                   + initLightBlocks[i].boundingRect2f().width,
                a3 = initLightBlocks[j].boundingRect2f().x,
                a4 = initLightBlocks[j].boundingRect2f().x
                   + initLightBlocks[j].boundingRect2f().width;
            int b1 = initLightBlocks[i].boundingRect2f().y,
                b2 = initLightBlocks[i].boundingRect2f().y
                   + initLightBlocks[i].boundingRect2f().height,
                b3 = initLightBlocks[j].boundingRect2f().y,
                b4 = initLightBlocks[j].boundingRect2f().y
                   + initLightBlocks[j].boundingRect2f().height;

            int left = min(min(min(a1, a2), a3), a4) - 1;//左边界
            int right = max(max(max(a1, a2), a3), a4) + 1;//右边界
            int top = min(min(min(b1, b2), b3), b4) - 1;//上边界
            int bottom = max(max(max(b1, b2), b3), b4) + 1;//下边界

            int labelvalue[1] = {0};
            Point seed, neighbor;
            int rows = bottom;
            int cols = right;
            stack<Point> pointStack; // 堆栈

            //通过压栈计算框定区域内连通域数量
            if(left > 0 && right < dstImage.cols
                    && top > 0 && bottom < dstImage.rows)
            {
                for (unsigned int i = top; i < rows; i++)
                {
                    uchar* data = labelImg.ptr<uchar>(i);//获取一行的点
                    for (unsigned int j = left; j < cols; j++)
                    {
                        if (data[j] == 255)
                        {
                            labelvalue[0]++; //不断将标签数加一
                            seed = Point(j, i);// Point坐标
                            labelImg.at<uchar>(seed) = labelvalue[0];//标签
                            pointStack.push(seed);//将像素seed压入栈，增加数据

                            while (!pointStack.empty())//死循环，直到堆栈为空
                            {
                                neighbor = Point(seed.x - 1, seed.y);//左像素
                                if ((seed.x != 0)
                                        && (labelImg.at<uchar>(neighbor) == 255))
                                {
                                    labelImg.at<uchar>(neighbor) = labelvalue[0];
                                    pointStack.push(neighbor);
                                }

                                neighbor = Point(seed.x + 1, seed.y);//右像素
                                if ((seed.x != (cols - 1))
                                        && (labelImg.at<uchar>(neighbor) == 255))
                                {
                                    labelImg.at<uchar>(neighbor) = labelvalue[0];
                                    pointStack.push(neighbor);
                                }

                                neighbor = Point(seed.x, seed.y - 1);//上像素
                                if ((seed.y != 0)
                                        && (labelImg.at<uchar>(neighbor) == 255))
                                {
                                    labelImg.at<uchar>(neighbor) = labelvalue[0];
                                    pointStack.push(neighbor);
                                }

                                neighbor = Point(seed.x, seed.y + 1);//下像素
                                if ((seed.y != (rows - 1))
                                        && (labelImg.at<uchar>(neighbor) == 255))
                                {
                                    labelImg.at<uchar>(neighbor) = labelvalue[0];
                                    pointStack.push(neighbor);
                                }
                                seed = pointStack.top();

                                //  获取堆栈上的顶部像素并将其标记为相同的标签
                                pointStack.pop();//弹出栈顶像素
                            }
                        }
                    }
                }

                //imshow("labelImg", labelImg);

                //筛选出框定区域内只喊两个连通域的外接矩形，排除错误匹配
                if(labelvalue[0] == 2)
                {
                    if(abs(initLightBlocks[i].center.y-initLightBlocks[j].center.y)<
                            0.5*abs(initLightBlocks[i].center.x-initLightBlocks[j].center.x))
                    {
                        finalLightBlocks.push_back(initLightBlocks[i]);
                        finalLightBlocks.push_back(initLightBlocks[j]);
                        vector<Point> armourPoints;
                        if(finalLightBlocks.size() != 0)
                        {
                            Point2f lightPoints[finalLightBlocks.size()][4];
                            for(unsigned int m = 0; m < finalLightBlocks.size(); m++)
                            {
                                finalLightBlocks[m].points(lightPoints[m]);
                                for(unsigned int n = 0; n < 4; n++)
                                    armourPoints.push_back(lightPoints[m][n]);
                            }
                        }

                        if(armourPoints.size()>4)
                        {
                            RotatedRect minRotatedRect = minAreaRect(armourPoints);
                            initArmourBlock.push_back(minRotatedRect);
                        }

                        finalLightBlocks.clear();
                        armourPoints.clear();
                    }
                    return initArmourBlock;
                }
            }
        }
    }
}

void ArmourDetector::markArmourBlocks(const Mat& srcImage,
                                      const Mat& dstImage,
                                      const vector<RotatedRect>&armourBlocks)
{
    //清除之前运算的结果
    optimalArmourBlocks.clear();

    //去除灯柱灯光区域影响
    Mat invDstImage;
    threshold(dstImage, invDstImage, 0, 255, THRESH_BINARY_INV);

    for(unsigned int id = 0; id < armourBlocks.size(); id++)
    {
        Point2f fpoints[4];
        armourBlocks[id].points(fpoints);

        //剪去旋转矩形的多余边角，得到装甲板的平行四边形区域
        //cutEdgeOfRect(fpoints);

        //浮点数转换整数
        Point points[4];
        for(unsigned int i = 0; i < 4; i++)
        {
            points[i] = Point(static_cast<int>(fpoints[i].x), static_cast<int>(fpoints[i].y));
        }

        const Point* pts = points;
        const int npts = 4;

        //创建掩码区域为包含装甲板的旋转矩形
        Mat mask(srcImage.size(), CV_8UC1, Scalar(0));
        //多边形填充
        fillConvexPoly(mask, pts, npts, Scalar(255));

        bitwise_and(mask, invDstImage, mask);        

        double shortEdge = min(armourBlocks[id].size.height, armourBlocks[id].size.width);
        double longEdge = max(armourBlocks[id].size.height, armourBlocks[id].size.width);

        Mat gray;
        cvtColor(srcImage, gray, CV_BGR2GRAY);
        Scalar armourBlockMean, armourBlockStdDev;

        //计算平均值与方差
        meanStdDev(gray, armourBlockMean, armourBlockStdDev, mask);

//        cout<<"meanStdDev:"<<armourBlockStdDev[0]/armourBlockMean[0]<<"\t"
//        <<"longEdge/shortEdge"<<longEdge/shortEdge<<endl;

        //长宽比与离散系数乘积去除错误错误匹配并判别多辆车远近
        double grade = (longEdge/shortEdge)/5*sin(90 - abs(armourBlocks[id].angle))
                *(armourBlockStdDev[0]/armourBlockMean[0]);

        //imshow("mask", mask);

        optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[id], grade));
    }

    //将装甲板区域按分从小到大排序，找出最佳区域
    sort(optimalArmourBlocks.begin(), optimalArmourBlocks.end());
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


