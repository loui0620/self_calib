/*
 * flags.h
 *
 * Copyright(C) 2014, University of Oulu, all rights reserved.
 * Copyright(C) 2014, NVIDIA Corporation, all rights reserved.
 * Third party copyrights are property of their respective owners.
 * Contact: Daniel Herrera C. (dherrera@ee.oulu.fi),
 *          Kihwan Kim(kihwank@nvidia.com)
 * Author : Daniel Herrera C.
 */

#ifndef dtslam_FLAGS_H_
#define dtslam_FLAGS_H_

#include <stdint.h>
#include <string>

namespace planecalib
{
	const static int32_t FLAGS_PyramidMaxTopLevelWidth = 640;
	const static int32_t FLAGS_SBIMaxWidth = 60;
	const static int32_t FLAGS_FeatureDetectorThreshold = 10;
	const static int32_t FLAGS_MatcherPixelSearchDistance = 8;
	const static int32_t FLAGS_CameraId = 0;

	const static std::string FLAGS_VideoFile = "/home/louis/Desktop/Video.mp4";
	const static std::string FLAGS_ImageSequenceFormat = "";
	const static int32_t FLAGS_ImageSequenceStartIdx = 0;
	const static int32_t FLAGS_DropFrames = 5;
	const static int32_t FLAGS_InputMaxImageWidth = 640;
	const static bool FLAGS_SingleThreaded = true;
	const static std::string FLAGS_RecordPath = "/home/lycheng/Desktop/record/";
	const static std::string FLAGS_RecordVideoFile = "video.avi";

	const static int32_t FLAGS_WindowWidth = 1280;
	const static int32_t FLAGS_WindowHeight = 720;

	/*
	DECLARE_int32_flag(PyramidMaxTopLevelWidth);
	DECLARE_int32_flag(SBIMaxWidth);
	DECLARE_int32_flag(FeatureDetectorThreshold);
	DECLARE_int32_flag(FrameKeypointGridSize);              ***

	DECLARE_int32_flag(TrackerMaxFeatures);
	DECLARE_int32_flag(TrackerMaxFeaturesPerOctave);
	DECLARE_int32_flag(TrackerMinMatchCount);

	DECLARE_int32_flag(TrackerSelectFeaturesGridSize);
	DECLARE_int32_flag(MatcherPixelSearchDistance);
	DECLARE_int32_flag(TrackerOutlierPixelThreshold);

	DECLARE_int32_flag(MatcherNonMaximaPixelSize);
	DECLARE_int32_flag(MatcherMaxZssdScore);
	DECLARE_double_flag(MatcherBestScorePercentThreshold);
	DECLARE_double_flag(WarperMaxCornerDrift);


	DECLARE_double_flag(ExpanderMinTriangulationAngle);
	DECLARE_int32_flag(ExpanderMinNewTriangulationsForKeyFrame);
	DECLARE_double_flag(ExpanderNewCoverageRatioForKeyFrame);

	DECLARE_int32_flag(PoseMinRansacIterations);
	DECLARE_int32_flag(PoseMaxRansacIterations);
	DECLARE_int32_flag(PoseMinTrackLength);
	DECLARE_double_flag(PoseStableRatioThreshold);

	DECLARE_bool_flag(DisableBA);
	DECLARE_int32_flag(GlobalBAFrameCount);

	DECLARE_int32_flag(CameraId);
	DECLARE_string_flag(VideoFile);
	DECLARE_string_flag(ImageSequenceFormat);
	DECLARE_int32_flag(ImageSequenceStartIdx);
	DECLARE_int32_flag(DropFrames);
	DECLARE_int32_flag(InputMaxImageWidth);
	DECLARE_bool_flag(SingleThreaded);
	DECLARE_string_flag(RecordPath);
	DECLARE_string_flag(RecordVideoFile);
	*/
}

#endif 
