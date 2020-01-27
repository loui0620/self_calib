#ifndef PLANECALIBHANDLER_H_
#define PLANECALIBHANDLER_H_
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <Eigen/Dense>

#include "Profiler.h"
#include "Keyframe.h"
#include "PoseTracker.h"
#include "PlaneCalibSystem.h"
//#include "planecalib/PlaneCalibHandler.h"
#include "HomographyCalibration.h"
#include "UserInterfaceInfo.h"

namespace planecalib
{
// Pre-decalred for compiler
class PlaneCalibSystem;
class ImageDataSource;
class OpenCVDataSource;

class PlaneCalibHandler : public PlaneCalibSystem
{
public:

    bool init();
    bool initImageSrc();
    bool getFinished() {return mQuit;}

    void RecordInputFrame(cv::Mat3b &im);
    void Update(void);
	void DoHomographyBA();
	void DoFullBA();
    void exit();

	int getSystemKeyframeSize() const { return mSystem->getKeyframeSize(); }

	bool mIsCalibrated;

private:
    bool mInitialized = false;
    int mFrameCount = 0;
    
    int mDownsampleInputCount;
    bool mUsingCamera = false;
    volatile bool mQuit = false;

    std::unique_ptr<PlaneCalibSystem> mSystem;
    std::unique_ptr<ImageDataSource> mImageSrc;

    bool mRecordVideo = false;
    bool mFrameByFrame = false;
    bool mAdvanceFrame = true;
    bool mRecordOneFrame = true;

    float mFPS;
	std::chrono::high_resolution_clock::time_point mLastFPSCheck;
	std::chrono::high_resolution_clock::duration mFPSUpdateDuration;
	std::chrono::high_resolution_clock::duration mFPSSampleAccum;
	int mFPSSampleCount;

    bool mShowProfiler = true;
    bool mShowProfilerTotals = false;

    int mRecordId;
    std::string mRecordFileFormat;
};

}



#endif