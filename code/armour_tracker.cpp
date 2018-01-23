#include "armour_tracker.h"

namespace HCVC
{
ArmourTracker::ArmourTracker()
{
}

void ArmourTracker::init(const Mat &srcImage, Rect2d armourBlock)
{
    TrackerKCF::Params param;
    param.desc_pca = TrackerKCF::GRAY | TrackerKCF::CN;
    param.desc_npca = 0;
    param.compress_feature = true;
    param.compressed_size = 2;
    param.output_sigma_factor=0.1;
    param.resize=true;
    param.max_patch_size=80*80;

    tracker = TrackerKCF::create(param);

    //tracker->setFeatureExtractor(sobelExtractor);

    tracker->init(srcImage, armourBlock);
}

bool ArmourTracker::track(Mat srcImage)
{
    Rect2d armourBlock;
    //追踪目标区域
    if(tracker->update(srcImage, armourBlock) == false)
    {
       return false;
    }

    //画出追踪的区域
    rectangle(srcImage, armourBlock, Scalar(255, 0, 0), 2, 1);

    return true;
}

void ArmourTracker::sobelExtractor(const Mat img, const Rect roi, Mat& feat)
{
    Mat sobel[2];
    Mat patch;
    Rect region=roi;

    //! [insideimage]
    // extract patch inside the image
    if(roi.x<0){region.x=0;region.width+=roi.x;}
    if(roi.y<0){region.y=0;region.height+=roi.y;}
    if(roi.x+roi.width>img.cols)region.width=img.cols-roi.x;
    if(roi.y+roi.height>img.rows)region.height=img.rows-roi.y;
    if(region.width>img.cols)region.width=img.cols;
    if(region.height>img.rows)region.height=img.rows;
    //! [insideimage]

    patch=img(region).clone();
    cvtColor(patch,patch, CV_BGR2GRAY);

    //! [padding]
    // add some padding to compensate when the patch is outside image border
    int addTop,addBottom, addLeft, addRight;
    addTop=region.y-roi.y;
    addBottom=(roi.height+roi.y>img.rows?roi.height+roi.y-img.rows:0);
    addLeft=region.x-roi.x;
    addRight=(roi.width+roi.x>img.cols?roi.width+roi.x-img.cols:0);

    copyMakeBorder(patch,patch,addTop,addBottom,addLeft,addRight,BORDER_REPLICATE);
    //! [padding]

    //! [sobel]
    Sobel(patch, sobel[0], CV_32F,1,0,1);
    Sobel(patch, sobel[1], CV_32F,0,1,1);

    merge(sobel,2,feat);
    //! [sobel]

    //! [postprocess]
    feat.convertTo(feat,CV_64F);
    feat=feat/255.0-0.5; // normalize to range -0.5 .. 0.5
    //! [postprocess]
}
}
