#include "armour_tracker.h"

namespace HCVC
{
ArmourTracker::ArmourTracker()
{
}

void ArmourTracker::init(const Mat &srcImage, Rect2d armourBlock)
{    
    TrackerKCF::Params param;
    param.desc_pca = TrackerKCF::GRAY | TrackerKCF::CN| TrackerKCF::CUSTOM;
    param.desc_npca = 0;
    param.compress_feature = true;
    param.compressed_size = 2;
    param.resize = true;
    param.pca_learning_rate = 1.9;
    //param.split_coeff = true;

    tracker = TrackerKCF::create(param);

    //获取检测结果传递图像的V通道图像
//    initRect = armourBlock;
//    roi = srcImage(Rect(armourBlock.x, armourBlock.y, armourBlock.width, armourBlock.height));
//    Mat initHSV;
//    cvtColor(roi, initHSV, CV_BGR2HSV);
//    Mat hsvImage[3], initValue;
//    split(initHSV, hsvImage);
//    inRange(hsvImage[2], 200, 255, initValue);
//    medianBlur(initValue, initValue, 3);

//    //获取每个连通域的外接矩形
//    vector<vector<Point> >contours;
//    findContours(initValue, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//    RotatedRect initLightRect[contours.size()];
//    for(unsigned int i = 0; i < contours.size(); i++)
//        initLightRect[i] = minAreaRect(contours[i]);

//    //根据检测结果的外接正矩形获取矩形内接装甲板的长度
//    if(contours.size() == 2)
//        gamma = min(abs((initLightRect[0].angle + initLightRect[1].angle)/2),
//                90 - abs((initLightRect[0].angle + initLightRect[1].angle)/2));
//    if(contours.size() > 2)
//    {
//        float initBlockArea = 0;
//        RotatedRect initMaxAreaBlock;
//        for(unsigned int i = 0; i < contours.size(); i++)
//        {
//            if(initLightRect[i].size.area() > initBlockArea)
//            {
//                initBlockArea = initLightRect[i].size.area();
//                initMaxAreaBlock = initLightRect[i];
//            }
//        }
//        gamma = min(abs(initMaxAreaBlock.angle),90 - abs(initMaxAreaBlock.angle));
//    }
//    initArmourLength = max(armourBlock.width, armourBlock.height)*cos(gamma);

    tracker->init(srcImage, armourBlock);
}

bool ArmourTracker::track(Mat srcImage)
{
    //追踪目标区域
    if(tracker->update(srcImage, armourBlock) == false)
    {
       return false;
    }

    correctBorders(armourBlock, srcImage);

    //获取个性后矩形框的Mat形式，方便之后处理
    updateRoi = srcImage(Rect2d(armourBlock.x, armourBlock.y, armourBlock.width, armourBlock.height));

    Rect2d minBoundRect;

    if(3*armourBlock.area() > roi.cols*roi.rows)
    //对矩形框进行矫正，获取其最小外接矩形
        minBoundRect = refineRect(updateRoi, srcImage);

    //imshow("updateRoi", updateRoi);

    armourBlock = Rect2d(minBoundRect.x + armourBlock.x - armourBlock.height/4,
                         minBoundRect.y + armourBlock.y - armourBlock.height/2,
                         minBoundRect.width, minBoundRect.height);

    correctBorders(armourBlock, srcImage);

    //画出追踪的区域
    rectangle(srcImage, armourBlock, Scalar(255, 0, 0), 2, 1);

//    fourierTransform(updateRoi);

    return true;
}

Rect2d ArmourTracker::refineRect(Mat& updateRoi, Mat& srcImage)
{
    //提取初始跟踪区域V通道二值化图
    Mat updateHSV;
    cvtColor(updateRoi, updateHSV, CV_BGR2HSV);
    Mat hsvImages[3];
    split(updateHSV, hsvImages);
    inRange(hsvImages[2], 200, 255, updateValue);
    medianBlur(updateValue, updateValue, 3);

    //提取扩大范围跟踪区域V通道二值化图

    Rect2d amplifyRect = Rect2d(armourBlock.x - armourBlock.height/4,
                                armourBlock.y - armourBlock.height/2,
                                armourBlock.width + armourBlock.height/2,
                                9*armourBlock.height/5);

    correctBorders(amplifyRect, srcImage);

    adjustRoi = srcImage(amplifyRect);

    Mat adjustHSV, hsvImage[3];
    cvtColor(adjustRoi, adjustHSV, CV_BGR2HSV);
    split(adjustHSV, hsvImage);
    inRange(hsvImage[2], 200, 255, adjustValue);
    medianBlur(adjustValue, adjustValue, 3);

    //获取更新后矩形框内连通域轮廓
    vector<vector<Point> > contours;
    findContours(updateValue, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //drawContours(updateRoi, contours, -1, Scalar(0,255,255), 2);

    RotatedRect rotatedRect[contours.size()];
    RotatedRect minArmourRect;
    Rect2d minBoundRect;
    int boundNum = 1;

    //获取连通域最小外接矩形
    for(unsigned int a = 0; a < contours.size(); a++)
         rotatedRect[a] = minAreaRect(contours[a]);    

    //根据更新后框内连通域数量进行不同的矫正
    if(contours.size() == 0)
        minBoundRect = Rect(armourBlock.height/4, armourBlock.height/2,
                            armourBlock.width, armourBlock.height);
    if(contours.size() == 1)
    {
        searchmatchDomains(minArmourRect, rotatedRect, contours.size(), boundNum);

//        if(boundNum == 0)
//        {
//            if(abs(rotatedRect[0].angle) <= 4 ||abs(rotatedRect[0].angle >= 86))
//                minBoundRect = minArmourRect.boundingRect2f();
//            else
//            {
//                if(armourBlock.x < 0 || armourBlock.x + armourBlock.width > srcImage.cols
//                        ||armourBlock.y < 0 || armourBlock.y + armourBlock.height > srcImage.rows)
//                   refineOverBorder(minArmourRect, rotatedRect, contours.size());
//                else
//                   refineNonOverBorder(minArmourRect, rotatedRect, contours.size());
//            }
//        }

        if(3*minArmourRect.size.area() > roi.rows*roi.cols)
            minBoundRect = minArmourRect.boundingRect2f();
    }

    if(contours.size() == 2)
    {
        if(rotatedRect[0].size.area() <= 1.5*rotatedRect[1].size.area()
           &&rotatedRect[1].size.area() <= 1.5*rotatedRect[0].size.area()
           &&(abs(rotatedRect[0].angle - rotatedRect[1].angle) < 9
           ||abs(abs(rotatedRect[0].angle - rotatedRect[1].angle) - 90) < 9))
            minBoundRect =Rect(armourBlock.height/4, armourBlock.height/2,
                               armourBlock.width, armourBlock.height);
        else
        {
             searchmatchDomains(minArmourRect, rotatedRect, contours.size(), boundNum);
             if(3*minArmourRect.size.area() > roi.rows*roi.cols)
                 minBoundRect = minArmourRect.boundingRect2f();
        }
    }

    if(contours.size() > 2)
    {
        searchmatchDomains(minArmourRect, rotatedRect, contours.size(), boundNum);
        if(3*minArmourRect.size.area() > roi.rows*roi.cols)
            minBoundRect = minArmourRect.boundingRect2f();
    }

    //rectangle(adjustRoi, minBoundRect, Scalar(255,255,0));

    //imshow("updateValue", updateValue);
    //imshow("adjustValue", adjustValue);
    //imshow("adjustRoi", adjustRoi);

    return minBoundRect;
}

void ArmourTracker::refineOverBorder(RotatedRect& minArmourRect,
                                     RotatedRect* rotatedRect,
                                     int rotatedSize)
{
    Point2f corners[4];
    vector<Point2f> topCorners;
    rotatedRect[0].points(corners);

    //获取旋转矩形上方两点
    for(unsigned int i = 0; i < 4; i++)
    {
        if(corners[i].y < rotatedRect[0].center.y)
            topCorners.push_back(corners[i]);
    }

    minArmourRect.angle = rotatedRect[0].angle;

    //根据旋转矩形的相对更新矩形的为止与上方两点的夹角分四类进行矫正
    if(2*rotatedRect[0].center.x < updateValue.cols)
    {
        if((topCorners[0].y - topCorners[1].y)/(topCorners[0].x - topCorners[1].x) < 0)
        {
            minArmourRect.center.x = rotatedRect[0].center.x +
                    (updateValue.cols - rotatedRect[0].center.x)*
                    pow(cos(rotatedRect[0].angle), 2)/2;
            minArmourRect.center.y = rotatedRect[0].center.y -
                    (updateValue.cols - rotatedRect[0].center.y)
                    *sin(2*abs(rotatedRect[0].angle))/4;
            minArmourRect.size.width = rotatedRect[0].size.width/2 +
                    (updateValue.cols - rotatedRect[0].center.x)*
                    cos(rotatedRect[0].angle);
            minArmourRect.size.height = rotatedRect[0].size.height;
        }
        else
        {
            minArmourRect.center.x = rotatedRect[0].center.x +
                    (updateValue.cols - rotatedRect[0].center.x)*
                    pow(cos(90 + rotatedRect[0].angle), 2)/2;
            minArmourRect.center.y = rotatedRect[0].center.y +
                    (updateValue.cols - rotatedRect[0].center.x)*
                    sin(2*(90 - abs(rotatedRect[0].angle)))/4;
            minArmourRect.size.height = rotatedRect[0].size.height/2 +
                    (updateValue.cols - rotatedRect[0].center.x)*
                    cos(90 + rotatedRect[0].angle);
            minArmourRect.size.width = rotatedRect[0].size.width;
        }
    }
    else
    {
        if((topCorners[0].y - topCorners[1].y)/(topCorners[0].x - topCorners[1].x) < 0)
        {
            minArmourRect.center.x = rotatedRect[0].center.x -
                    rotatedRect[0].center.x*pow(cos(rotatedRect[0].angle), 2)/2;
            minArmourRect.center.y = rotatedRect[0].center.y +
                    rotatedRect[0].center.x*sin(2*abs(rotatedRect[0].angle))/4;
            minArmourRect.size.width = rotatedRect[0].size.width/2 +
                    rotatedRect[0].center.x*cos(rotatedRect[0].angle);
            minArmourRect.size.height = rotatedRect[0].size.height;
        }
        else
        {
            minArmourRect.center.x = rotatedRect[0].center.x -
                    rotatedRect[0].center.x*pow(cos(90 + rotatedRect[0].angle), 2)/2;
            minArmourRect.center.y = rotatedRect[0].center.y -
                    rotatedRect[0].center.x*sin(2*(90 - abs(rotatedRect[0].angle)))/4;
            minArmourRect.size.height = rotatedRect[0].size.height/2 +
                    rotatedRect[0].center.x*cos(90 + rotatedRect[0].angle);
            minArmourRect.size.width = rotatedRect[0].size.width;
        }
    }
}

void ArmourTracker::refineNonOverBorder(RotatedRect& minArmourRect,
                                        RotatedRect* rotatedRect,
                                        int rotatedSize)
{
    Point2f corners[4];
    vector<Point2f> topCorners;
    rotatedRect[0].points(corners);

    //获取旋转矩形上方两点
    for(unsigned int i = 0; i < 4; i++)
    {
        if(corners[i].y < rotatedRect[0].center.y)
            topCorners.push_back(corners[i]);
    }

    minArmourRect.angle = rotatedRect[0].angle;

    //根据旋转矩形的相对更新矩形的为止与上方两点的夹角分四类进行矫正
    if(2*rotatedRect[0].center.x < updateValue.cols)
    {
        if((topCorners[0].y - topCorners[1].y)/(topCorners[0].x - topCorners[1].x) < 0)
        {
            minArmourRect.center.x = rotatedRect[0].center.x +
                    initArmourLength*cos(rotatedRect[0].angle)/2;
            minArmourRect.center.y = rotatedRect[0].center.y +
                    initArmourLength*sin(rotatedRect[0].angle)/2;
            minArmourRect.size.width = rotatedRect[0].size.width/2 +
                    initArmourLength*(1 + sin(rotatedRect[0].angle - gamma));
            minArmourRect.size.height = rotatedRect[0].size.height;
        }
        else
        {
            minArmourRect.center.x = rotatedRect[0].center.x +
                    initArmourLength*cos(90 + rotatedRect[0].angle)/2;
            minArmourRect.center.y = rotatedRect[0].center.y +
                    initArmourLength*sin(90 + rotatedRect[0].angle)/2;
            minArmourRect.size.height = rotatedRect[0].size.height/2 +
                    initArmourLength*(1 + sin(90 + rotatedRect[0].angle - gamma));
            minArmourRect.size.width = rotatedRect[0].size.width;
        }
    }
    else
    {
        if((topCorners[0].y - topCorners[1].y)/(topCorners[0].x - topCorners[1].x) < 0)
        {
            minArmourRect.center.x = rotatedRect[0].center.x -
                    initArmourLength*cos(rotatedRect[0].angle)/2;
            minArmourRect.center.y = rotatedRect[0].center.y -
                    initArmourLength*sin(rotatedRect[0].angle)/2;
            minArmourRect.size.width = rotatedRect[0].size.width/2 +
                    initArmourLength*(1 + sin(rotatedRect[0].angle - gamma));
            minArmourRect.size.height = rotatedRect[0].size.height;
        }
        else
        {
            minArmourRect.center.x = rotatedRect[0].center.x -
                    initArmourLength*cos(90 + rotatedRect[0].angle)/2;
            minArmourRect.center.y = rotatedRect[0].center.y -
                    initArmourLength*sin(90 + rotatedRect[0].angle)/2;
            minArmourRect.size.height = rotatedRect[0].size.height/2 +
                    initArmourLength*(1 + sin(90 + rotatedRect[0].angle - gamma));
            minArmourRect.size.width = rotatedRect[0].size.width;
        }
    }
}

void ArmourTracker::searchmatchDomains(RotatedRect& minArmourRect,
                                       RotatedRect* rotatedRect,
                                       int rotatedSize,
                                       int& boundNum)
{
    //获取放大后矩形框内连通域数量
    Point2f rotatedCenter[rotatedSize];
    vector<vector<Point> > contours;
    findContours(adjustValue, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    RotatedRect updateBlocks[rotatedSize];
    RotatedRect adjustBlocks[contours.size() - 1];

    //对矩形框进行计数
    unsigned int updateBlockNum = 0;
    unsigned int preNum = 0;
    unsigned int exceptNum = 0;

    //将放大后矩形框分更新后矩形与非更新后矩形两类
    for(unsigned int i = 0; i < contours.size(); i++)
    {
        preNum = updateBlockNum;
        for(unsigned int j = 0; j < rotatedSize; j++)
        {
            rotatedCenter[j].x = rotatedRect[j].center.x + armourBlock.height/4;
            rotatedCenter[j].y = rotatedRect[j].center.y + armourBlock.height/2;

            //判断点与矩形的关系进行分类
            if(pointPolygonTest(contours[i], rotatedCenter[j], false) >= 0)
            {
                updateBlocks[updateBlockNum] = minAreaRect(contours[i]);
                updateBlockNum++;
            }
        }
        if(preNum == updateBlockNum)
        {
            adjustBlocks[exceptNum] = minAreaRect(contours[i]);
            exceptNum++;
        }
    }

//    cout<<"updateBlockNum:"<<contours.size()<<"\t"<<rotatedSize<<"\t"
//       <<updateBlockNum<<"\t"<<exceptNum<<endl;

    vector<RotatedRect> armours;
    RotatedRect updateClone[updateBlockNum];
    RotatedRect adjustClone[exceptNum];
    int updateNum;
    int adjustNum;

    updateNum = cloneScreen(updateBlocks, rotatedSize, updateClone, updateBlockNum);
    adjustNum = cloneScreen(adjustBlocks, contours.size() - 1, adjustClone, exceptNum);

    //对更新后矩形框内连通域数量大于2的进行匹配，寻找最优匹配
    if(updateNum >= 2)
    {
        armours = updateScreen(updateClone, updateNum, updateBlockNum);

        if(armours.size() == 0)
            armours = adjustScreen(updateClone, updateBlockNum, updateNum,
                                     adjustClone, exceptNum, adjustNum);
        if(armours.size() == 1)
            minArmourRect = armours[0];
        if(armours.size() > 1)
            minArmourRect = armourConfidence(armours);
    }

    //对改性后矩形框内连通域与放大后矩形框内非更新矩形框内连通域矩形匹配
    if(updateNum == 1)
    {
        armours = adjustScreen(updateClone, updateBlockNum, updateNum,
                                 adjustClone, exceptNum, adjustNum);

        if(armours.size() == 0)
            boundNum--;
        if(armours.size() == 1)
            minArmourRect = armours[0];
        if(armours.size() > 1)
            minArmourRect = armourConfidence(armours);
    }
}

int ArmourTracker::cloneScreen(RotatedRect* blocks,
                               int blockSize,
                               RotatedRect* clone,
                               int cloneBlockNum)
{
    int num = 0;

    //对单个矩形矩形筛选，初步选出优先矩
    for(unsigned int i = 0; i < blockSize; i++)
    {
        if(blocks[i].angle > -20)
        {
            if(blocks[i].size.height > 0.8*blocks[i].size.width)
            {
                clone[num] = blocks[i];
                num++;
            }
        }

        if(blocks[i].angle < -70)
        {
            if(blocks[i].size.width > 0.8*blocks[i].size.height)
            {
                clone[num] = blocks[i];
                num++;
            }
        }
    }

    return num;
}

vector<RotatedRect> ArmourTracker::updateScreen(RotatedRect* updateClone,
                                                int updateNum,
                                                int updateBlockNum)
{
    vector<RotatedRect> armours;
    RotatedRect armourRotated;

    //对筛选后矩形进行匹配筛选
    if(updateNum >= 2)
    {
        for(unsigned i = 0; i < updateNum - 1; i++)
        {
            for(unsigned j = i + 1; j < updateNum; j++)
            {
                if(updateClone[i].size.area() > 0.15*updateClone[j].size.area()
                        && updateClone[j].size.area() > 0.15*updateClone[i].size.area())
                {
                    RotatedRect matchDomains[2];

                    float includedAngle = atan(abs((updateClone[i].center.y - updateClone[j].center.y)/
                            (updateClone[i].center.x - updateClone[j].center.x)))*180/CV_PI;
                    if(includedAngle < 30)
                    {
                        float angleI = min(abs(updateClone[i].angle), 90 - abs(updateClone[i].angle));
                        float angleJ = min(abs(updateClone[j].angle), 90 - abs(updateClone[j].angle));
                        if(abs(angleI - angleJ < 15))
                        {
                            //获取两矩形的外接矩形
                            matchDomains[0] = updateClone[i];
                            matchDomains[1] = updateClone[j];
                            armourRotated = getArmourRotated(matchDomains, 2);
                            armours.push_back(armourRotated);
                        }
                    }
                }
            }
        }
    }

    return armours;
}

vector<RotatedRect> ArmourTracker::adjustScreen(RotatedRect* updateClone,
                                                int updateBlockNum,
                                                int updateNum,
                                                RotatedRect* adjustClone,
                                                int exceptNum,
                                                int adjustNum)
{
    vector<RotatedRect> armours;
    RotatedRect armourRotated;       

    //对筛选后矩形进行匹配筛选
    if(adjustNum > 1)
    {
        for(unsigned int i = 0; i < updateNum; i++)
        {
            for(unsigned int j = 0; j < adjustNum; j++)
            {
                if(updateClone[i].size.area() > 0.15*adjustClone[j].size.area()
                        && adjustClone[j].size.area() > 0.15*updateClone[i].size.area())
                {
                    RotatedRect matchDomains[2];

                    float includedAngle = atan(abs((updateClone[i].center.y - adjustClone[j].center.y)/
                            (updateClone[i].center.x - adjustClone[j].center.x)))*180/CV_PI;
                    if(includedAngle < 30)
                    {
                        float angleI = min(abs(updateClone[i].angle), 90 - abs(updateClone[i].angle));
                        float angleJ = min(abs(adjustClone[j].angle), 90 - abs(adjustClone[j].angle));
                        if(abs(angleI - angleJ < 15))
                        {
                            //获取两矩形的外接矩形
                            matchDomains[0] = updateClone[i];
                            matchDomains[1] = adjustClone[j];
                            armourRotated = getArmourRotated(matchDomains, 2);
                            armours.push_back(armourRotated);
                        }
                    }
                }
            }
        }
    }

    return armours;
}

RotatedRect ArmourTracker::armourConfidence(vector<RotatedRect>& armours)
{
    RotatedRect appraiseArmour[armours.size()];
    float appraiseGrade[armours.size()];
    unsigned int num = 0;

    for(unsigned i = 0; i < armours.size(); i++)
    {
        float grade = 0;
        Point2f fpoints[4];
        armours[i].points(fpoints);

        //浮点数转换整数
        Point points[4];
        for(unsigned int i = 0; i < 4; i++)
        {
            points[i] = Point(static_cast<int>(fpoints[i].x), static_cast<int>(fpoints[i].y));
        }

        const Point* pts = points;
        const int npts = 4;

        //创建掩码区域为包含装甲板的旋转矩形
        Mat mask(adjustValue.size(), CV_8UC1, Scalar(0));
        //多边形填充
        fillConvexPoly(mask, pts, npts, Scalar(255));

        //使用位“与”运算获取旋转矩形内连通域
        bitwise_and(mask, adjustValue, mask);

        //获取旋转矩形内连通域数量
        vector<vector<Point> >contours;
        findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        //对连通域数量为2的旋转矩形进行评分
        if(contours.size() == 2)
        {
            RotatedRect light[2];
            light[0] = minAreaRect(contours[0]);
            light[1] = minAreaRect(contours[1]);

            float angle_0 = min(abs(light[0].angle), 90 - abs(light[0].angle));
            float angle_1 = min(abs(light[1].angle), 90 - abs(light[1].angle));

            float slantAngle = min(abs(armours[i].angle), 90 - abs(armours[i].angle));

            double similarity = matchShapes(contours[0], contours[1], CV_CONTOURS_MATCH_I1, 0);

            float largeArea = max(contourArea(contours[0]), contourArea(contours[1]));
            float smallArea = min(contourArea(contours[0]), contourArea(contours[1]));

            //计算两轮廓的分数，由旋转矩形倾斜角，两连通域旋转角差, 轮廓相似度组成
            grade = (slantAngle*CV_PI/180 + 0.01)/5 * (abs(angle_0 - angle_1) + 1)*similarity/3;

            appraiseArmour[num] = armours[i];
            appraiseGrade[num] = grade;
            num++;
        }
    }

    RotatedRect optimalArmour = sortArmour(appraiseArmour, appraiseGrade, armours.size(), num);
    return optimalArmour;
}

RotatedRect ArmourTracker::sortArmour(RotatedRect* appraiseArmour,
                                      float* appraiseGrade,
                                      int arrayNum,
                                      unsigned int num)
{
    RotatedRect optimalArmour = appraiseArmour[0];
    float lowestGrade = appraiseGrade[0];

    //选出分数最低的旋转矩形为最优矩
    for(unsigned i = 1; i < num; i++)
    {
        if(appraiseGrade[i] < lowestGrade)
            optimalArmour = appraiseArmour[i];
    }

    return optimalArmour;
}

void ArmourTracker::correctBorders(Rect2d& initRect, Mat& srcImage)
{
    Rect2d region = initRect;

    //对越界矩形边界进行纠正
    if (initRect.x < 0){initRect.x = 0;initRect.width += region.x;}
    if (initRect.x + initRect.width > srcImage.cols)
    {
        initRect.width = srcImage.cols - region.x;
    }
    if (initRect.y < 0){initRect.y = 0;initRect.height += region.y;};
    if ( initRect.y + initRect.height > srcImage.rows)
    {
        initRect.height = srcImage.rows - region.y;
    }
}

RotatedRect ArmourTracker::getArmourRotated(RotatedRect* matchDomains, int matchSize)
{
    RotatedRect armourRotated;
    vector<Point> armourPoints;

    for(unsigned int i = 0; i < matchSize; i++)
    {
        Point2f lightPoints[4];
        matchDomains[i].points(lightPoints);

        for(unsigned int j = 0; j < 4; j++)
            armourPoints.push_back(lightPoints[j]);
    }
    armourRotated = minAreaRect(armourPoints);

    return armourRotated;
}

Rect2d ArmourTracker::getArmourBlock() const
{
    return armourBlock;
}

void ArmourTracker::fourierTransform(Mat& src)
{
    Mat src_gray;
    cvtColor(src,src_gray,CV_RGB2GRAY);//灰度图像做傅里叶变换

    int m = getOptimalDFTSize(src_gray.rows);//2,3,5的倍数有更高效率的傅里叶转换
    int n = getOptimalDFTSize(src_gray.cols);

    Mat dst;
    ///把灰度图像放在左上角，在右边和下边扩展图像，扩展部分填充为0；
    copyMakeBorder(src_gray,dst,0,m-src_gray.rows,0,n-src_gray.cols,BORDER_CONSTANT,Scalar::all(0));

    //新建一个两页的array，其中第一页用扩展后的图像初始化，第二页初始化为0
    Mat planes[] = {Mat_<float>(dst), Mat::zeros(dst.size(), CV_32F)};
    Mat  completeI;
    merge(planes,2,completeI);//把两页合成一个2通道的mat

    //对上边合成的mat进行傅里叶变换，支持原地操作，傅里叶变换结果为复数。通道1存的是实部，通道2存的是虚部。
    dft(completeI,completeI);

    split(completeI,planes);//把变换后的结果分割到各个数组的两页中，方便后续操作
    magnitude(planes[0],planes[1],planes[0]);//求傅里叶变换各频率的幅值，幅值放在第一页中。

    Mat magI = planes[0];
    //傅立叶变换的幅度值范围大到不适合在屏幕上显示。高值在屏幕上显示为白点，
    //而低值为黑点，高低值的变化无法有效分辨。为了在屏幕上凸显出高低变化的连续性，我们可以用对数尺度来替换线性尺度:
    magI+= 1;
    log(magI,magI);//取对数
    magI= magI(Rect(0,0,src_gray.cols,src_gray.rows));//前边对原始图像进行了扩展，
                                                      //这里把对原始图像傅里叶变换取出，剔除扩展部分。


    //这一步的目的仍然是为了显示。 现在我们有了重分布后的幅度图，
    //但是幅度值仍然超过可显示范围[0,1] 。我们使用 normalize() 函数将幅度归一化到可显示范围。
    normalize(magI,magI,0,1,CV_MINMAX);//傅里叶图像进行归一化。


    //重新分配象限，使（0,0）移动到图像中心，
    //傅里叶变换之前要对源图像乘以（-1）^(x+y)进行中心化。
    //这是是对傅里叶变换结果进行中心化
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat tmp;
    Mat q0(magI,Rect(0,0,cx,cy));
    Mat q1(magI,Rect(cx,0,cx,cy));
    Mat q2(magI,Rect(0,cy,cx,cy));
    Mat q3(magI,Rect(cx,cy,cx,cy));


    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);

    imshow("magI", magI);
}
}
