#include "PlaneCalibHandler.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include "OpenCVDataSource.h"
#include "SequenceDataSource.h"
#include "UserInterfaceInfo.h"

#include "flags.h"
#include <gflags/gflags.h>

namespace planecalib
{


bool PlaneCalibHandler::init() 
{
    std::cout << "PlaneCalibApp init." << std::endl;
	//UserInterfaceInfo::Instance().setScreenSize(cv::Size2i(System::ScreenGetWidth(), System::ScreenGetHeight()));
	mIsCalibrated = false;

#ifdef ENABLE_LOG
	std::cout << "Logging is enabled." << std::endl;
#else
	std::cout << "Logging is disabled." << std::endl;
#endif

    Profiler::Instance().setCurrentThreadName("render");
    char glogstr[] =
    //"planecalib";
            "";//Empty str disables the creation  of a log file from glog. Only ceres uses glog. Not needed.


    //google::InitGoogleLogging(glogstr);
	
	if (!initImageSrc())
	{
		MYAPP_LOG << "Couldn't initialize image source.\n";
		return false;
	}

	//Determine downscale at input
	int width = mImageSrc->getSourceSize().width;
	mDownsampleInputCount = 0;
	while(width > FLAGS_InputMaxImageWidth)
	{
		width = (width+1)/2;
		mDownsampleInputCount++;
	}
	int scale = 1<<mDownsampleInputCount;

	mImageSrc->setDownsample(mDownsampleInputCount);
	mImageSize = eutils::FromSize(mImageSrc->getSize());
	MYAPP_LOG << "Input image size after downsampling: " << eutils::ToSize(mImageSize) << "\n";

	//Get first frame
	if(!mImageSrc->update())
    {
		MYAPP_LOG << "Couldn't get first frame from image source.\n";
    	return false;
    }

	//System
	cv::Mat1b imageGray = mImageSrc->getImgGray();
	cv::Mat3b imageColor = mImageSrc->getImgColor();
	mSystem.reset( new PlaneCalibSystem() );
	mSystem->init(mImageSrc->getCaptureTime(), imageColor, imageGray);
	mSystem->setSingleThreaded(FLAGS_SingleThreaded);

    //Record vars
	mRecordId = 0;
	mRecordFileFormat = FLAGS_RecordPath + "frame%.4d.jpg";

    mInitialized = true;
    return true;
}

bool PlaneCalibHandler::initImageSrc()
{
    bool fakeRun = false;
    if (fakeRun) {
        std::string videoFilename = FLAGS_VideoFile;
        MYAPP_LOG << "Video file: " << videoFilename << "\n";
        return 0;
    }

    mUsingCamera = false;
	if(!FLAGS_VideoFile.empty())
	{
		//Use video file
	    std::string videoFilename = FLAGS_VideoFile;

		MYAPP_LOG << "Video file: " << videoFilename << "\n";
		OpenCVDataSource *source = new OpenCVDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(videoFilename))
	    {
			MYAPP_LOG << "Error opening video.\n";
	        return NULL;
	    }

		MYAPP_LOG << "Opened video file succesfully\n";
	}
	else if(!FLAGS_ImageSequenceFormat.empty())
	{
		//Use image sequence
		std::string sequence = FLAGS_ImageSequenceFormat;

		MYAPP_LOG << "Image sequence: " << sequence << "\n";
		SequenceDataSource *source = new SequenceDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(sequence, FLAGS_ImageSequenceStartIdx))
	    {
			MYAPP_LOG << "Error opening sequence.\n";
	        return NULL;
	    }

		MYAPP_LOG << "Opened image sequence succesfully\n";
	}
	else
	{
		//Use camera
		OpenCVDataSource *source = new OpenCVDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(FLAGS_CameraId))
	    {
			MYAPP_LOG << "Error opening camera.\n";
	        return false;
	    }

	    mUsingCamera = true;
		MYAPP_LOG << "Camera opened succesfully\n";
	}

	MYAPP_LOG << "Image source size: " << mImageSrc->getSourceSize() << "\n";
    printf("image source inited.\n");
	return true;

}

void PlaneCalibHandler::RecordInputFrame(cv::Mat3b &im)
{
    //Convert to bgr
	cv::Mat3b bgr;
	cv::cvtColor(im, bgr, cv::COLOR_RGB2BGR);

	//Save
	char buffer[1024];
	sprintf(buffer, mRecordFileFormat.c_str(), mFrameCount);

	cv::imwrite(buffer, bgr);
}

void PlaneCalibHandler::Update()
{
    bool stateUpdated = false;

    if(!mFrameByFrame || mAdvanceFrame) // if not STEP input OR 
	{
        bool isFrameAvailable;
        {
        	ProfileSection section("updateFrame");

        	//Drop frames
        	mImageSrc->dropFrames(FLAGS_DropFrames);

			isFrameAvailable = mImageSrc->update();
        }
        if (!isFrameAvailable) printf("FRAME EMPTY\n");
        if(isFrameAvailable)
		{
            ProfileSection section("execute");

			mFrameCount++;
			mAdvanceFrame = false;

			MYAPP_LOG << "\nFrame #" << mFrameCount << "\n";
			bool outputResult = true;

			/*
			Eigen::Vector2i scrrenSize(1280, 720);
			Eigen::Vector3f normal( 8.167486e-03, 6.826394e-03, 1.646491e-02);
			Eigen::Vector2f focal(763.897229443986700,  759.564980657802830);
			Eigen::Vector2f center(706.737854664114250, 137.246360609633650);
			if (mFrameCount == 20) {
				
				MYAPP_LOG << std::fixed << std::setprecision(2) << std::setw(6);
				MYAPP_LOG << "Image Size: " << scrrenSize << "\n";
				MYAPP_LOG << "Closed-form Fx=" << mSystem->getCalib().getInitialAlpha() << "\n";
				MYAPP_LOG << "\nRefined:\n";

				MYAPP_LOG << std::fixed << std::setprecision(3) << std::setw(4);
				MYAPP_LOG << " Normal=[" << normal << "]\n";

				MYAPP_LOG << std::fixed << std::setprecision(2) << std::setw(6);
				MYAPP_LOG << " F  = " << focal(0) << " " << focal(1) << "\n";
				MYAPP_LOG << " PP  = " << center(0) << " " << center(1) << "\n";

				MYAPP_LOG << std::fixed << std::setprecision(3) << std::setw(4);
				MYAPP_LOG << " Distortion = [" << mSystem->getCamera().getDistortionModel().getParams().transpose() << "]";
				
			}
			*/
			//Read new input frame
			cv::Mat1b imageGray = mImageSrc->getImgGray();
			cv::Mat3b imageColor = mImageSrc->getImgColor();

            std::string nameGray = mRecordFileFormat + "gray_" + std::to_string(mFrameCount) + ".png";
            std::string nameColor = mRecordFileFormat + "color_" + std::to_string(mFrameCount) + ".png";
            cv::imwrite(nameGray, imageGray);
            cv::imwrite(nameColor, imageColor);

			//Record
			if (mRecordOneFrame)
			{
				mRecordOneFrame = false;
				RecordInputFrame(imageColor);
			}

			//Process new frame
			auto tic = std::chrono::high_resolution_clock::now();
			mSystem->processImage(mImageSrc->getCaptureTime(), imageColor, imageGray);
			mFPSSampleAccum += std::chrono::high_resolution_clock::now()-tic;
			mFPSSampleCount++;
			
			stateUpdated = true;
			
			//if (mSystem->getTracker().isLost())
			//	mFrameByFrame = true;
		}
	}

	//mSlam.idle();

    {
        ProfileSection section("draw");

	    //Text
	    std::stringstream ss;
		ss << "FPS " << (int)mFPS << "\n";
		ss << "Frame " << mFrameCount << "\n";
        

	    if(mShowProfiler)
		{
		    Profiler::Instance().logStats(ss);
		}

	    Eigen::Vector2i screenSize = UserInterfaceInfo::Instance().getScreenSize();
		float viewportAspect = static_cast<float>(screenSize.x()) / screenSize.y();

	    //Map stats in the bottom
        float kDefaultFontHeight = 10;
		std::vector<Eigen::Vector2f> corners;
		corners.push_back(Eigen::Vector2f(0.0f, (float)screenSize.y() - 2 * kDefaultFontHeight));
		corners.push_back(Eigen::Vector2f((float)screenSize.x(), (float)screenSize.y() - 2 * kDefaultFontHeight));
		corners.push_back(Eigen::Vector2f(0.0f, (float)screenSize.y()));
		corners.push_back(Eigen::Vector2f((float)screenSize.x(), (float)screenSize.y()));
	    
		{
			int frameCount = mSystem->getMap().getKeyframes().size();
			int count3D = mSystem->getMap().getFeatures().size();

			MYAPP_LOG << "Keyframes: " << frameCount << ", Features: " << count3D << ", Matches: " << mSystem->getTracker().mTotalMatchSuccess << "/" << mSystem->getTracker().mTotalMatchAttempts << std::endl;
            
		//	switch (mSlam.getTracker().getPoseType() )
		//	{
		//		case EPoseEstimationType::PureRotation:
		//			ts.setColor(StaticColors::Yellow());
		//			ts << " PURE ROTATION";
		//			break;
		//		case EPoseEstimationType::Essential:
		//			ts.setColor(StaticColors::Green());
		//			ts << " ESSENTIAL MODEL";
		//			break;
		//		case EPoseEstimationType::Invalid:
		//			ts.setColor(StaticColors::Red());
		//			ts << " LOST";
		//			break;
		//	}
		//	ts.setColor(StaticColors::White());

		//	//Expander status
		//	switch (mSlam.getMapExpander().getStatus())
		//	{
		//	case ESlamMapExpanderStatus::CheckingFrame:
		//		ts << ", expander checking";
		//		break;
		//	case ESlamMapExpanderStatus::AddingFrame:
		//		ts << ", expander adding";
		//		break;
		//	case ESlamMapExpanderStatus::SingleFrameBA:
		//		ts << ", expander one-frame BA";
		//		break;
		//	}

		//	//BA status
		//	if (mSlam.isBARunning())
		//	{
		//		ts << ", ";
		//		if (mSlam.getActiveRegion()->getAbortBA())
		//		{
		//			ts.setColor(StaticColors::Yellow());
		//			ts << "aborting BA";
		//			ts.setColor(StaticColors::White());
		//		}
		//		else
		//		{
		//			ts << "BA is running";
		//		}
		//	}
		//	else if (mSlam.getActiveRegion()->getShouldBundleAdjust())
		//	{
		//		ts << ", ";
		//		ts.setColor(StaticColors::Yellow());
		//		ts << "BA pending";
		//		ts.setColor(StaticColors::White());
		//	}
		}
    }

	//Update FPS
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsedDuration = now - mLastFPSCheck;
	if (elapsedDuration > mFPSUpdateDuration)
	{
		if (mFPSSampleCount)
			mFPS = mFPSSampleCount / std::chrono::duration_cast<std::chrono::duration<float>>(mFPSSampleAccum).count();
		
		mFPSSampleCount = 0;
		mFPSSampleAccum = std::chrono::high_resolution_clock::duration(0);

		mLastFPSCheck = now;
	}

	if (stateUpdated && mRecordVideo)
	{
		//recordOutputFrame();
	}
}

void PlaneCalibHandler::DoHomographyBA()
{
	mSystem->doHomographyBA();
	mSystem->doHomographyCalib(true);
}

void PlaneCalibHandler::DoFullBA()
{
	mSystem->doFullBA();
}

void PlaneCalibHandler::exit()
{
}

}