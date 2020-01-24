/*
 * ImageDataSource.cpp
 *
 *  Created on: 16.4.2014
 *      Author: dherrera
 */

#include "ImageDataSource.h"
#include <opencv2/imgproc.hpp>
#include "cvutils.h"

namespace planecalib
{

void ImageDataSource::setSourceSize(const cv::Size &sz)
{
	mSourceSize = sz;
}

void ImageDataSource::update(const cv::Mat3b &source, double captureTime)
{
	mCaptureTime = captureTime;

	//Switch channels around
	cv::Mat3b sourceRgb;
	cv::cvtColor(source, sourceRgb, cv::COLOR_BGR2RGB);

    //Downsample color image
    mImgColor = cv::Mat3b();
    cvutils::DownsampleImage(sourceRgb, mImgColor, mDownsampleCount);

    // convert to gray format
    mImgGray = cv::Mat1b();
    cv::cvtColor(mImgColor, mImgGray, cv::COLOR_RGB2GRAY);
}

}
