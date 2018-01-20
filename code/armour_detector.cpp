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

    node["maxWidthRat"] >> params.maxWidthRat;
    node["maxHeightRat"] >> params.maxHeightRat;
    node["maxAngleDiff"] >> params.maxAngleDiff;
    node["maxHeightGapRat"] >> params.maxHeightGapRat;
    node["minHeightGapRat"] >> params.minHeightGapRat;
    
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
    vector<RotatedRect> armourBlocks = extracArmourBlocks(lampBlocks,srcImage,dstImage);
    Mat a=srcImage.clone();
    drawBlocks(a,armourBlocks,Scalar(0,255,255));
    imshow("a",a);

    //查看搜索出的每一个独立的团块
    drawBlocks(drawImage, armourBlocks, Scalar(100, 150, 200));

    if(armourBlocks.empty())
    {
        return false;
    }

    //对每个装甲板区域评分

    markArmourBlocks(srcImage, dstImage, armourBlocks);

    drawBlocks(drawImage, vector<RotatedRect>(1, optimalArmourBlocks.front().block), Scalar(180, 200, 220));

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

void ArmourDetector::drawBlocks(Mat srcImage, const vector<RotatedRect>& minRotatedRects, const Scalar& color) const
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



void ArmourDetector::fillLampBlock(Mat& srcImage, vector<vector<Point> >& blocks, int row, int col)
{
    //如果这个像素越界或者为零则不需要进一步访问它的相邻像素
    if(row < 0 || row >= srcImage.rows || col < 0 || col >= srcImage.cols || srcImage.at<uchar>(row, col) == 0)
    {
        return ;
    }

    //向当前正在填充的团块中加入数据，注意点的坐标是（x，y），指针访问的坐标顺序是（y，x），x为横轴，y为纵轴
    blocks.back().push_back(Point(col, row));
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
            if(minRotatedRect.size.area() > params.minArea
            &&((minRotatedRect.angle > -params.angleRange
              &&((minRotatedRect.size.height/minRotatedRect.size.width >= params.minHeightWidthRat)
                &&(minRotatedRect.size.height/minRotatedRect.size.width <= params.maxHeightWidthRat)))
             ||(minRotatedRect.angle < params.angleRange-90
               &&((minRotatedRect.size.width/minRotatedRect.size.height >= params.minHeightWidthRat)
                &&(minRotatedRect.size.width/minRotatedRect.size.height <= params.maxHeightWidthRat)))))
            {
                lampBlocks.push_back(minRotatedRect);
            }
    }

    return lampBlocks;
}

vector<RotatedRect> ArmourDetector::extracArmourBlocks(const vector<RotatedRect>& lampBlocks,const Mat srcImage,const Mat dstImage)
{
    vector<RotatedRect> armourBlocks;
    vector<RotatedRect> initArmourBlocks;

    //非空判定，如果为空的话在下面遍历的时候会出现一个bug，i-1溢出成2^32-1，使循环卡死
    if(lampBlocks.empty())
    {
        return armourBlocks;
    }

    for(unsigned int i = 0; i < lampBlocks.size() - 1; i++)
    {
        for(unsigned int j = i + 1; j < lampBlocks.size(); j++)
        {
            if(abs(lampBlocks[i].center.y-lampBlocks[j].center.y)<0.5*abs(lampBlocks[i].center.x-lampBlocks[j].center.x))
            {
                if((lampBlocks[i].boundingRect2f().area()>0.2*lampBlocks[j].boundingRect2f().area())&&(lampBlocks[j].boundingRect2f().area()>0.2*lampBlocks[i].boundingRect2f().area()))
                {
                    //建立初始团块最小外接矩形
                    vector<Point> initPoints;
                    Point2f firstPoints[4], secondPoints[4];
                    lampBlocks[i].points(firstPoints);
                    lampBlocks[j].points(secondPoints);
                    for(unsigned int k = 0; k < 4; k++)
                    {
                        initPoints.push_back(firstPoints[k]);
                        initPoints.push_back(secondPoints[k]);
                    }

                    //创建掩码区域为包含装甲板的旋转矩形
                    Mat mask(srcImage.size(), CV_8UC1, Scalar(0));
                    estableMask(mask,dstImage,initPoints);

                    //计算甲板区间范围内的像素比例
                    double avg,mean;
                    double percent;
                    calcDeviation(mask,srcImage,avg,mean,percent);

                    //根据像素的离散程度再次筛选甲板
                    if(mean>0.85&&percent<0.06&&avg<50)
                    {
                        cout<<mean<<" "<<percent<<" "<<" "<<avg<<endl;

                        initArmourBlocks.push_back(lampBlocks[i]);
                        initArmourBlocks.push_back(lampBlocks[j]);

                        //外接正矩形连通域数量检测
                        int labelvalue[1];
                        //domainCountDetect(labelvalue[1],initArmourBlocks);

                        vector<Point> points;
                        Point2f iPoints[4], jPoints[4];
                        lampBlocks[i].points(iPoints);
                        lampBlocks[j].points(jPoints);
                        for (unsigned int k = 0; k < 4; k++)
                        {
                            points.push_back(iPoints[k]);
                            points.push_back(jPoints[k]);
                        }

                        RotatedRect minRotatedRect = minAreaRect(points);
                        armourBlocks.push_back(minRotatedRect);
                    }
                }
            }
        }
    }
    //cout << "Num of lampBlocksRects: " << armourBlocks.size() << endl;

    return armourBlocks;
}

void ArmourDetector::estableMask(Mat mask,const Mat& dstImage,const vector<Point> initPoints)
{
    //绘制最小外接矩形
    RotatedRect minRect=minAreaRect(initPoints);

    //寻找矩形的四个点
    Point2f fpoints[4];
    minRect.points(fpoints);

    //剪去旋转矩形的多余边角，得到装甲板的平行四边形区域
    cutEdgeOfRect(fpoints);

    //浮点数转换整数
    Point points[4];
    for(unsigned int i = 0; i < 4; i++)
    {
        points[i] = Point(static_cast<int>(fpoints[i].x), static_cast<int>(fpoints[i].y));
    }

    const Point* pts = points;
    const int npts=4;

    //去除灯柱灯光区域影响
    Mat invDstImage;
    threshold(dstImage, invDstImage, 0, 255, THRESH_BINARY_INV);

    fillConvexPoly(mask,pts,npts,Scalar(255));
    bitwise_and(mask,invDstImage,mask);
}

void ArmourDetector::calcDeviation(const Mat& mask,const Mat& srcImage,double& avg,double& mean,double& percent)
{
    Mat gray=Mat (srcImage.rows,srcImage.cols,CV_8UC1);
    cvtColor(srcImage,gray,COLOR_BGR2GRAY);
    int rows=srcImage.rows;
    int cols=srcImage.cols;
    double sum=0;//像素值的总和
    double armourpixelCount[1] = { 0 };//甲板像素数量
    double armourRangPixel[1]={0};//所需区间内像素
    double notArmourRangPixel[1] = { 0 };//远离甲板平均值像素
    avg=0;//像素的平均值
    mean=0;//区间范围内像素所占比例
    percent=0;//区间外像素所占比例

    //计算甲板像素平均值
    for(int i=0;i<rows;i++)
    {
        const uchar* grayData=gray.ptr<uchar>(i);
        const uchar* maskData=mask.ptr<uchar>(i);
        for(int j=0;j<cols;j++)
        {
            if(maskData[j]==255)
            {
                sum+=grayData[j];
                armourpixelCount[0]++;
            }
        }
    }
    avg=sum/armourpixelCount[0];

    //求甲板像素给定区间范围内与外像素值
    int range = (int)avg;
    for (int i = 0;i < rows;i++)
    {
        const uchar* grayData = gray.ptr<uchar>(i);
        const uchar* maskData = mask.ptr<uchar>(i);
        for (int j = 0;j < cols;j++)
        {
            if (maskData[j] == 255)
            {
                if (grayData[j]<range + 10 && grayData[j]>range - 10)
                    armourRangPixel[0]++;
                if (grayData[j] > range + 15)
                    notArmourRangPixel[0]++;
            }
        }
    }
    mean=armourRangPixel[0]/armourpixelCount[0];
    percent=notArmourRangPixel[0]/armourpixelCount[0];
}

/*
void ArmourDetector::domainCountDetect(int& labelvalue[1],const vector<RotatedRect> &initArmourBlocks)
{

}
*/
void ArmourDetector::markArmourBlocks(const Mat& srcImage, const Mat& dstImage, const vector<RotatedRect> &armourBlocks)
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

        Scalar armourBlockMean, armourBlockStdDev;
        meanStdDev(srcImage, armourBlockMean, armourBlockStdDev, mask);
        double grade = sqrt((pow(armourBlockMean[0], 2) + pow(armourBlockMean[1], 2) + pow(armourBlockMean[2], 2))/3.0) + 5 * sqrt((pow(armourBlockStdDev[0], 2) + pow(armourBlockStdDev[1], 2) + pow(armourBlockStdDev[2], 2))/3.0);
        imshow("mask", mask);

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
