#ifndef PNPESTIMATION_H_
#define PNPESTIMATION_H_

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "CameraModel.h"
#include "Map.h"
#include "BaseRansac.h"
#include "FeatureMatcher.h"
#include "cvutils.h"
#include "PoseEstimationCommon.h"

namespace planecalib {

class PoseReprojectionError3D;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PnPRansac
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct PnPIterationData
{
	std::vector<MatchReprojectionErrors> reprojectionErrors;
};

class PnPRansac: public BaseRansac<std::pair<Eigen::Matrix3dr, Eigen::Vector3d>, PnPIterationData, 4>
{
public:
	PnPRansac();
	~PnPRansac();

	void setData(const std::vector<FeatureMatch> &matches, const CameraModel *camera);
	void setData(const std::vector<Eigen::Vector3f> *refPoints, const std::vector<Eigen::Vector2f> *imgPoints, const std::vector<float> *scales, const CameraModel *camera);

	std::vector<std::pair<Eigen::Matrix3dr, Eigen::Vector3d>> modelFromMinimalSet(const std::vector<int> &constraintIndices);
	void getInliers(const std::pair<Eigen::Matrix3dr, Eigen::Vector3d> &model, int &inlierCount, float &errorSumSq, PnPIterationData &data);

protected:
	int mMatchCount;

	const std::vector<Eigen::Vector3f> *mRefPoints;
	const std::vector<Eigen::Vector2f> *mImgPoints; //This is normalized world coordinates (xn)

	std::unique_ptr<std::vector<Eigen::Vector3f>> mOwnRefPoints;
	std::unique_ptr<std::vector<Eigen::Vector2f>> mOwnImgPoints;

	std::vector<std::unique_ptr<PoseReprojectionError3D>> mErrorFunctors;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PnPRefiner
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PnPRefiner
{
public:
	PnPRefiner()		
	{
	}

	void setCamera(const CameraModel *camera) { mCamera = camera; }
	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = mOutlierPixelThreshold*mOutlierPixelThreshold;
	}


	void getInliers(const std::vector<Eigen::Vector3f> &refPoints,
		const std::vector<Eigen::Vector2f> &imgPoints,
		const std::vector<float> &scales,
			const Eigen::Matrix3dr &R,
			const Eigen::Vector3d &translation,
			int &inlierCount,
			std::vector<MatchReprojectionErrors> &errors);

	void refinePose(const std::vector<FeatureMatch> &matches,
			Eigen::Matrix3fr &rotation,
			Eigen::Vector3f &translation,
			int &inlierCount,
			std::vector<MatchReprojectionErrors> &errors);
	void refinePose(const std::vector<Eigen::Vector3f> &refPoints,
		const std::vector<Eigen::Vector2f> &imgPoints,
		const std::vector<float> &scales,
		Eigen::Matrix3fr &rotation,
		Eigen::Vector3f &translation,
		int &inlierCount,
		std::vector<MatchReprojectionErrors> &errors);

protected:
	const CameraModel *mCamera;
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
};

}

#endif /* PNPESTIMATION_H_ */
