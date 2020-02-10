#include <memory>
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp> // For runtime clock

#include "planecalib/flags.h"

#include "planecalib/ImageDataSource.h"
#include "planecalib/OpenCVDataSource.h"
#include "planecalib/SequenceDataSource.h"
#include "planecalib/PlaneCalibHandler.h"

namespace planecalib
{
/*
///////////////////////////////////////////////////////
DEFINE_int32_flag(PyramidMaxTopLevelWidth, 1280, "Maximum width of the highest pyramid level for a frame.");
DEFINE_int32_flag(SBIMaxWidth, 60, "Maximum width for the Small Blurry Image, input will be downsampled until width is less than this.");
DEFINE_int32_flag(FeatureDetectorThreshold, 10, "Threshold for the keypoint detector");
DEFINE_int32_flag(MatcherPixelSearchDistance, 8, "The search distance for matching features (distance from point projection or from epiplar line). Units in pixels of the highest pyramid level.");
DEFINE_int32_flag(CameraId, 0, "Id of the camera to open (OpenCV).");

DEFINE_string_flag(VideoFile, "/home/lycheng/Desktop/iphone.avi", "Name of the video file to use (e.g. rotation3.mp4). If both VideoFile and SequenceFormat are empty, the camera is used.");
DEFINE_string_flag(ImageSequenceFormat, "", "sprintf format for the sequence (e.g. \"/cityOfSights/CS_BirdsView_L0/Frame_%.5d.jpg\". This is appended to the data path. If both VideoFile and SequenceFormat are empty, the camera is used.");
DEFINE_int32_flag(ImageSequenceStartIdx, 0, "Start index for the image sequence.");
DEFINE_int32_flag(DropFrames, 5, "The system will ignore this many frames per iteration, effectively lowering the frame rate or skipping frames in a video.");
DEFINE_int32_flag(InputMaxImageWidth, 1280, "Maximum width of input image. Input will be downsampled to be under this width.");
DEFINE_bool_flag(SingleThreaded, true, "Use a single thread for easier debugging.");
DEFINE_string_flag(RecordPath, "/home/lycheng/Desktop/record/", "Path where the frames will be stored in case of recording.");
DEFINE_string_flag(RecordVideoFile, "video.avi", "Output video file for recording.");

DEFINE_int32_flag(WindowWidth, 1280, "Initial width of the window.");
DEFINE_int32_flag(WindowHeight, 720, "Initial height of the window.");
///////////////////////////////////////////////////////
*/


}


int main() {
	
    std::unique_ptr<planecalib::PlaneCalibHandler> mainHandler;
    mainHandler.reset( new planecalib::PlaneCalibHandler );
    if (!mainHandler->init())
        printf("Calib Handler init failed.\n");

	double t1 = cv::getTickCount();
    
    int runCount = 0;
    while (!mainHandler->mIsCalibrated) 
	{
		int kayFrameCount = mainHandler->getSystemKeyframeSize();
        mainHandler->Update();
		
		if (runCount % 25 == 0 && runCount > 1) {
			mainHandler->DoHomographyBA();
			printf("Perform Homography Bundle Adjustment.\n");
		}

		if (kayFrameCount >= 20) {
			mainHandler->DoHomographyBA();
			mainHandler->DoFullBA();
			mainHandler->mIsCalibrated = true;
			printf("Execute Full BA Optimizing.\n");
			break;
		}
        runCount++;
    }
	double t2 = cv::getTickCount();
	double t = (t2 - t1) / cv::getTickFrequency();
	printf("Time taken: %.2f sec.\n\n",t);
	
    return 0;
}
