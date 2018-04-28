#include "video.h"

namespace HCVC {
Video::Video()
{

}

bool Video::init(string path)
{
    srcFilePath = path;
    srcFile.open(srcFilePath);
    srcFile.set(CAP_PROP_FRAME_WIDTH, 800);
    srcFile.set(CAP_PROP_FRAME_HEIGHT, 600);

    return srcFile.isOpened();
}

void Video::getNextFrame(Mat &frame)
{
    srcFile.read(frame);
}

VideoCapture &Video::operator >>(Mat &frame)
{
    srcFile >> frame;
    return srcFile;
}
}
