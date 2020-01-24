#ifndef IMAGEDATASOURCE_H_
#define IMAGEDATASOURCE_H_

#include <opencv2/core.hpp>

namespace planecalib
{

class ImageDataSource
{
public:
	ImageDataSource():
		mDownsampleCount(0)
	{
	}

    virtual ~ImageDataSource(void)
    {

    }

    virtual void dropFrames(int count) = 0;
    virtual bool update(void) = 0;

    double getCaptureTime(void) const {return mCaptureTime;}

    void setDownsample(int count) {mDownsampleCount=count;}

    const cv::Size &getSourceSize() const {return mSourceSize;}

    cv::Size getSize() const {return mImgGray.size();}

    const cv::Mat1b &getImgGray() const {return mImgGray;}
    const cv::Mat3b &getImgColor() const {return mImgColor;}

protected:
    cv::Size mSourceSize;
    int mDownsampleCount;

    double mCaptureTime;
    cv::Mat1b mImgGray;
    cv::Mat3b mImgColor;

    void setSourceSize(const cv::Size &sz);
    void update(const cv::Mat3b &source, double captureTime);
};

}

#endif /* IMAGEDATASOURCE_H_ */
