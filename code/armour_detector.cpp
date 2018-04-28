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

    node["tanAngle"] >> params.tanAngle;
    node["deviationAngle"] >> params.deviationAngle;
    node["armourPixelAvg"] >> params.armourPixelAvg;
    fs.release();
}

bool ArmourDetector::detect(const Mat& srcImage)
{
    Mat dstImage = preprocess(srcImage);
    Mat value = detectValue;

    //存储初步找到的团块
    vector<vector<Point> > blocks = searchBlocks(dstImage.clone());

    //检验数，配合数组，检测数组的实际长度
    int lampsNum = 0, armoursNum = 0;;

    //存储找到的所有灯柱块
    vector<RotatedRect> lampBlocks = calcBlocksInfo(blocks, lampsNum);

    //将vector存储到数组中
    RotatedRect lamps[lampsNum];
    for(unsigned i = 0; i < lampBlocks.size(); i++)
        lamps[i] = lampBlocks[i];

    //为中间结果显示准备图像
    Mat drawImage = srcImage.clone();

    //查看搜索出的每一个灯柱块
    Tool::drawVectorBlocks(drawImage, lampBlocks, Scalar(200, 150, 100));

    //存储筛选过符合条件的所有对灯柱对最小包围矩形即装甲板区域
    float directAngle[lampsNum];
    RotatedRect armourBlocks[lampsNum];
    extracArmourBlocks(armourBlocks, lamps, srcImage, dstImage, value,
                       directAngle, lampsNum, armoursNum);

    //查看搜索出的每一个独立的团块
    Tool::drawArrayBlocks(drawImage, armourBlocks, lampsNum, armoursNum, Scalar(100, 150, 200));

    if(armoursNum == 0)
    {
        return false;
    }

    //对每个装甲板区域评分

    markArmourBlocks(srcImage, dstImage, armourBlocks, directAngle, lampsNum, armoursNum);

    Tool::drawVectorBlocks(drawImage,
               vector<RotatedRect>(1, optimalArmourBlocks.front().block),
               Scalar(180, 200, 220));

    return true;
}

Rect2d ArmourDetector::getBestArmourBlock() const
{
    return optimalArmourBlocks.front().block.boundingRect();
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
                                        const RotatedRect* lampBlocks,
                                        const Mat srcImage,
                                        const Mat dstImage,
                                        const Mat value,
                                        float* directAngle,
                                        int lampsNum,
                                        int& armoursNum)
{
    int screenNum = 0;
    int pairNum = 0;
    float angle = 0;
    Point angleI[lampsNum], angleJ[lampsNum];
    RotatedRect screenLamps[lampsNum];
    RotatedRect pairLamps[lampsNum][2];
    RotatedRect initLightBlocks[2];
    RotatedRect initArmourBlock;

    int sequence[lampsNum];
    for(unsigned i = 0; i < lampsNum; i++)
    {
        sequence[i] = 0;
        directAngle[i] = 0;
    }

    //非空判定，如果为空的话在下面遍历的时候会出现一个bug，i-1溢出成2^32-1，使循环卡死
    if(lampsNum != 0)
    {
        for(unsigned int i = 0; i < lampsNum - 1; i++)
        {
            Point vecI = calVectorY(lampBlocks[i]);

            for(unsigned int j = i + 1; j < lampsNum; j++)
            {
                if(abs(lampBlocks[i].center.y - lampBlocks[j].center.y) <
                        params.tanAngle*abs(lampBlocks[i].center.x - lampBlocks[j].center.x))
                {
                    //计算灯柱的与y轴夹角最小的方向向量
                    Point vecJ = calVectorY(lampBlocks[j]);

                    //两向量的夹角
                    angle = acos((vecI.x*vecJ.x + vecI.y*vecJ.y)/
                            (sqrt(pow(vecI.x, 2) + pow(vecI.y, 2)) *
                             sqrt(pow(vecJ.x, 2) + pow(vecJ.y, 2))));

                    //弧度制转角度制
                    angle *= 180/CV_PI;

                    if(angle < params.angleRange)
                    {
                        if((lampBlocks[i].size.area() > 0.2*lampBlocks[j].size.area())
                                &&(lampBlocks[j].size.area() > 0.2*lampBlocks[i].size.area()))
                        {
                            //获取所有不重复的外接矩形
                            if(sequence[i] == 0)
                            {
                                screenLamps[screenNum] = lampBlocks[i];
                                sequence[i] = 1;
                                screenNum++;
                            }

                            if(sequence[j] == 0)
                            {
                                screenLamps[screenNum] = lampBlocks[j];
                                sequence[j] = 1;
                                screenNum++;
                            }

                            //获取匹配对数，储存夹角
                            angleI[pairNum] = vecI;
                            angleJ[pairNum] = vecJ;
                            directAngle[pairNum] = angle;
                            pairLamps[pairNum][0] = lampBlocks[i];
                            pairLamps[pairNum][1] = lampBlocks[j];
                            pairNum++;
                        }
                    }
                }
            }
        }
    }

    for(unsigned i = 0; i < pairNum; i++)
    {
        int labelValue = 0;

        initLightBlocks[0] = pairLamps[i][0];
        initLightBlocks[1] = pairLamps[i][1];

        //外接正矩形连通域数量检测
        domainCountDetect(initLightBlocks, screenLamps, dstImage,
                          value, labelValue, screenNum, lampsNum, 2);

        if(labelValue == 2)
        {
            initArmourBlock = getArmourRotated(initLightBlocks, 2);

            Point vecArmourY = calVectorY(initArmourBlock);
            Point vecArmourX = calVectorX(initArmourBlock);
            float length = sqrt(pow(vecArmourX.x, 2) + pow(vecArmourX.y, 2));

            //计算灯柱相对于装甲板的垂直偏向角
            float deviationAngleI = acos((angleI[i].x*vecArmourY.x
                                        + angleI[i].y*vecArmourY.y)/
                                    (sqrt(pow(angleI[i].x, 2) + pow(angleI[i].y, 2)) *
                                     sqrt(pow(vecArmourY.x, 2) + pow(vecArmourY.y, 2))));

            float deviationAngleJ = acos((angleJ[i].x*vecArmourY.x
                                        + angleJ[i].y*vecArmourY.y)/
                                    (sqrt(pow(angleJ[i].x, 2) + pow(angleJ[i].y, 2)) *
                                     sqrt(pow(vecArmourY.x, 2) + pow(vecArmourY.y, 2))));

            //弧度制转角度制，计算装甲板偏向角
            deviationAngleI *= 180/CV_PI;
            deviationAngleJ *= 180/CV_PI;

            deviationAngleI = min(deviationAngleI, abs(90 - deviationAngleI));
            deviationAngleJ = min(deviationAngleJ, abs(90 - deviationAngleJ));

            int left = initArmourBlock.boundingRect().x,
                top = initArmourBlock.boundingRect().y,
                width = initArmourBlock.boundingRect().width,
                height = initArmourBlock.boundingRect().height;

            //两灯柱的最大短边
            float lightWidth = max(min(initLightBlocks[0].size.height,
                                       initLightBlocks[0].size.width),
                                   min(initLightBlocks[1].size.height,
                                       initLightBlocks[1].size.width));

            if(left > 0 && left + width < dstImage.cols &&
                    top > 0 && top + height < dstImage.rows)
            {
                if(4*lightWidth < length
                   && deviationAngleI < params.deviationAngle
                   && deviationAngleJ < params.deviationAngle)
                {
                    directAngle[armoursNum] = directAngle[i] + deviationAngleI + deviationAngleJ;
                    armourBlocks[armoursNum] = initArmourBlock;

                    cout<<"deviationAngleI and deviationAngleJ:"<<
                          deviationAngleI<<"\t"<<deviationAngleJ<<endl;

                    cout<<"number:"<<i<<"\t"<<"left:"<<armourBlocks[armoursNum].center.x<<"\t"
                       <<"angle:"<<directAngle[armoursNum]<<"\n"<<endl;

                    armoursNum++;
                }
            }
        }
    }
}

Point ArmourDetector::calVectorX(const RotatedRect rotated)
{
    Point2f corners[4];
    rotated.points(corners);

    //获取矩形的中心点的y坐标
    float centery = 0;
    for(unsigned i = 0; i < 4; i++)
    {
        centery += corners[i].y;
    }
    centery /= 4;

    //求出左右两组点的中点
    Point top[2], bottom[2];
    int numTop = 0, numBottom = 0;

    for(unsigned i = 0; i < 4; i++)
    {
        if(corners[i].y < centery)
        {
            top[numTop] = corners[i];
            numTop++;
        }
        else
        {
            bottom[numBottom] = corners[i];
            numBottom++;
        }
    }

    //求出底边中点为终点点的向量,方向沿x轴正方向
    Point vec, vecLeft, vecRight;

    if(top[0].x < top[1].x)
    {
        if(bottom[0].x < bottom[1].x)
        {
            vecLeft.x = (top[0].x + bottom[0].x)/2;
            vecLeft.y = (top[0].y + bottom[0].y)/2;
            vecRight.x = (top[1].x + bottom[1].x)/2;
            vecRight.y = (top[1].y + bottom[1].y)/2;
        }
        else
        {
            vecLeft.x = (top[0].x + bottom[1].x)/2;
            vecLeft.y = (top[0].y + bottom[1].y)/2;
            vecRight.x = (top[1].x + bottom[0].x)/2;
            vecRight.y = (top[1].y + bottom[0].y)/2;
        }
    }
    else
    {
        if(bottom[0].x < bottom[1].x)
        {
            vecLeft.x = (top[1].x + bottom[0].x)/2;
            vecLeft.y = (top[1].y + bottom[0].y)/2;
            vecRight.x = (top[0].x + bottom[1].x)/2;
            vecRight.y = (top[0].y + bottom[1].y)/2;
        }
        else
        {
            vecLeft.x = (top[1].x + bottom[1].x)/2;
            vecLeft.y = (top[1].y + bottom[1].y)/2;
            vecRight.x = (top[0].x + bottom[0].x)/2;
            vecRight.y = (top[0].y + bottom[0].y)/2;
        }
    }

    vec = Point(vecRight.x - vecLeft.x, vecRight.y - vecLeft.y);

    return vec;
}

Point ArmourDetector::calVectorY(const RotatedRect rotated)
{
    Point2f corners[4];
    rotated.points(corners);

    //获取矩形的中心点的y坐标
    float centery = 0;
    for(unsigned i = 0; i < 4; i++)
    {
        centery += corners[i].y;
    }
    centery /= 4;

    //求出上下两组点的中点
    float topx = 0, topy = 0, bottomx = 0, bottomy = 0;
    for(unsigned i = 0; i < 4; i++)
    {
        if(corners[i].y < centery)
        {
            topx += corners[i].x;
            topy += corners[i].y;
        }
        else
        {
            bottomx += corners[i].x;
            bottomy += corners[i].y;
        }
    }

    topx /= 2; topy /= 2;
    bottomx /= 2; bottomy /= 2;

    //求出底边中点为终点点的向量,方向沿y轴正方向
    Point vec(bottomx - topx, bottomy - topy);

    //cout<<"vec:"<<vec<<endl;

    return vec;
}

void ArmourDetector::domainCountDetect(const RotatedRect* initLightBlocks,
                                       const RotatedRect* screenLamps,
                                       const Mat& dstImage,
                                       const Mat& value,
                                       int& labelValue,
                                       const int screenNum,
                                       int lampsNum,
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

    int doubleHeight = 2*height;
    if(top + doubleHeight >= dstImage.rows)
        doubleHeight = dstImage.rows - top - 1;

    Rect armourRect = Rect(Point(left, top), Point(left + width, top + doubleHeight));
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

    //通过不重复连通域查找排除地面可能造成的灯柱倒影,并减小下面连通域检测的计算量
    for(unsigned int i = 0; i < screenNum; i++)
    {
        Point p1 = armourRect.tl();
        Point p2 = armourRect.br();
        Point p3 = screenLamps[i].boundingRect().tl();
        Point p4 = screenLamps[i].boundingRect().br();
        if(p2.y > p3.y && p1.y < p4.y && p2.x > p3.x && p1.x < p4.x)
            labelValue++;
    }

    //再一次进行连通域检测，排除灯柱附近的一些噪点与错误匹配
    if(labelValue == 2)
    {
        labelValue = 0;

        //越界矫正
        int soleHeight = max(initLightBlocks[0].size.height, initLightBlocks[0].size.width);
        if(bottom + soleHeight > dstImage.rows)
            soleHeight = dstImage.rows - bottom - 1;

        vector<vector<Point> > contours;
        Mat roi = value(Rect(left, top, width, height + soleHeight));
        findContours(roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        labelValue = contours.size();
    }
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
                                   double& tanAngle,
                                   double& deviationAngle,
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
    tanAngle = 0;//区间范围内像素所占比例
    deviationAngle = 0;//区间外像素所占比例

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
     //tanAngle = armourRangPixel / armourPixelCount;
     deviationAngle = (notArmourRangPixel / armourPixelCount)*100;
}

void ArmourDetector::markArmourBlocks(const Mat& srcImage,
                                      const Mat& dstImage,
                                      const RotatedRect* armourBlocks,
                                      const float* directAngle,
                                      int lampsNum,
                                      int armoursNum)
{
    //清除之前运算的结果
    optimalArmourBlocks.clear();

    //通过评分选出最优装甲板
    if(armoursNum > 2)
    {
        int armourArea[2];
        RotatedRect initArmour[2];
        float angle[2];

        //初始化
        armourArea[0] = armourBlocks[0].size.area();
        initArmour[0] = armourBlocks[0];
        angle[0] = directAngle[0];        
        armourArea[1] = 0;

        //剪去旋转矩形的多余边角，得到装甲板的平行四边形区域
        //cutEdgeOfRect(fpoints);

        for(unsigned int i = 1; i < armoursNum; i++)
        {            
            if(armourBlocks[i].size.area() > armourArea[0])
            {
                armourArea[0] = armourBlocks[i].size.area();
                initArmour[0] = armourBlocks[i];
                angle[0] = directAngle[i];
            }
        }

        for(unsigned int i = 0; i < armoursNum; i++)
        {
            if(armourBlocks[i].size.area() < armourArea[0]
                    && armourBlocks[i].size.area() > armourArea[1])
            {
                armourArea[1] = armourBlocks[i].size.area();
                initArmour[1] = armourBlocks[i];
                angle[1] = directAngle[i];
            }
        }

        for(unsigned int i = 0; i < 2; i++)
        {
            float shortEdge = min(armourBlocks[i].size.height, armourBlocks[i].size.width);
            float longEdge = max(armourBlocks[i].size.height, armourBlocks[i].size.width);

            //float angle = min(abs(armourBlocks[i].angle), 90 - abs(armourBlocks[i].angle));

            float grade = angle[i];

            optimalArmourBlocks.push_back(OptimalArmourBlock(initArmour[i], grade));
        }

    }

    if(armoursNum == 2)
    {
        int armourAreaI = armourBlocks[0].size.area();
        int armourAreaJ = armourBlocks[1].size.area();

        if(armourAreaI > 2.5*armourAreaJ || armourAreaJ > 2.5*armourAreaI)
        {
            if(armourAreaI >armourAreaJ)
                optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[0], armourAreaI));
            else
                optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[1], armourAreaJ));
        }
        else
        {
            for(unsigned int i = 0; i < 2; i++)
               optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[i], directAngle[i]));
        }
    }

    if(armoursNum == 1)
    {
        optimalArmourBlocks.push_back(OptimalArmourBlock(armourBlocks[0], 1));
    }

    //将装甲板区域按分从小到大排序，找出最佳区域
    sort(optimalArmourBlocks.begin(), optimalArmourBlocks.end());
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

