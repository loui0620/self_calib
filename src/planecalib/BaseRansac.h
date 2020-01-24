#ifndef BASERANSAC_H_
#define BASERANSAC_H_

#include <vector>
#include <random>
#include <cassert>
#include <limits>
#include "log.h"
#include "Profiler.h"

namespace planecalib
{

template<class TModel, class TIterationData, int KMinConstraintCount>
class BaseRansac {
public:
	BaseRansac():
		//mGenerator(Profiler::Instance().now())
		mGenerator(1), //Fix to avoid random tests
		mConstraintCount(0),
		mBestErrorSumSq(std::numeric_limits<float>::infinity()),
		mBestInlierCount(0)
	{}

	virtual ~BaseRansac() {}

	float getOutlierErrorThreshold() const {return mOutlierErrorThreshold;}

	int getIterationsMade() const {return mIterationsMade;}
	const TModel &getBestModel() const {return mBestModel;}
	const int &getBestInlierCount() const {return mBestInlierCount;}
	std::unique_ptr<TIterationData> &getBestIterationData() {return mBestIterationData;}

	void setParams(float outlierErrorThreshold, int minIterations, int maxIterations, int acceptableInlierCount)
	{
		mOutlierErrorThreshold=outlierErrorThreshold;
		mOutlierErrorThresholdSq=outlierErrorThreshold*outlierErrorThreshold;
		mMinIterations = minIterations;
		mMaxIterations = maxIterations;
		mAcceptableInlierCount = acceptableInlierCount;
	}

	void setBestModel(const TModel &model)
	{
		mBestModel = model;
		mBestIterationData.reset(new TIterationData());

		mBestInlierCount = 0;
		mBestErrorSumSq = std::numeric_limits<float>::infinity();

		int inlierCount;
		float errorSumSq;
		getInliers(mBestModel, inlierCount, errorSumSq, *mBestIterationData);

		mBestInlierCount = inlierCount;
		mBestErrorSumSq = errorSumSq;

		finalizeModel();
	}

	int getMinConstraintCount() const {return KMinConstraintCount;}

	void doRansac();

	virtual std::vector<TModel> modelFromMinimalSet(const std::vector<int> &constraintIndices) = 0;
	virtual void getInliers(const TModel &model, int &inlierCount, float &errorSumSq, TIterationData &inliers) = 0;
	virtual void finalizeModel() {}

protected:
	float mOutlierErrorThreshold;
	float mOutlierErrorThresholdSq;
	int mMinIterations;
	int mMaxIterations;
	int mAcceptableInlierCount;
	int mIterationsMade;

	std::default_random_engine mGenerator;

	int mConstraintCount;

	TModel mBestModel;
	float mBestErrorSumSq;
	int mBestInlierCount;
	std::unique_ptr<TIterationData> mBestIterationData;

	void getRandomConstraintIndices(std::vector<int> &indices);
	void ransacIteration();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Template implementation
/////////////////////////////////////////////////////////////////////////////////////////////////////

template<class TModel, class TIterationData, int KMinConstraintCount>
void BaseRansac<TModel, TIterationData, KMinConstraintCount>::getRandomConstraintIndices(std::vector<int> &indices)
{
	assert(mConstraintCount >= getMinConstraintCount());

	std::uniform_int_distribution<int> distribution(0, mConstraintCount-1);

	indices.resize(getMinConstraintCount());
	for(int i=0; i<getMinConstraintCount(); ++i)
	{
		bool randomOk=false;
		while(!randomOk)
		{
			//Generate random index
			indices[i] = distribution(mGenerator);

			//Check for repeated indices
			randomOk = true;
			for(int j=0; j<i; ++j)
			{
				if(indices[j]==indices[i])
				{
					randomOk = false;
					break;
				}
			}
		}
	}
}

template<class TModel, class TIterationData, int KMinConstraintCount>
void BaseRansac<TModel, TIterationData, KMinConstraintCount>::doRansac()
{
	if(mConstraintCount < getMinConstraintCount())
		return;

	mBestInlierCount = 0;
	mBestErrorSumSq = std::numeric_limits<float>::infinity();
	//Calculate initial model with all
	//std::vector<int> allIndices(mConstraintCount);
	//for(int i=0; i<mConstraintCount; ++i)
	//	allIndices[i] = i;
	//mBestModel = modelFromInliers(allIndices);
	//getInliers(mBestModel, mBestInliers);
	//DTSLAM_LOG << "Ransac: inliers with all constraints=" << mBestInliers.size()<<"\n";

	//This can be parallelized
	for(mIterationsMade=0; mIterationsMade<mMaxIterations;)
	{
		ransacIteration();
		++mIterationsMade;

		if(mIterationsMade >= mMinIterations && mBestInlierCount >= mAcceptableInlierCount)
			break;
	}

	//Final model from inliers
	finalizeModel();
}


template<class TModel, class TIterationData, int KMinConstraintCount>
void BaseRansac<TModel, TIterationData, KMinConstraintCount>::ransacIteration()
{
	std::vector<int> minIndices;
	getRandomConstraintIndices(minIndices);

	const std::vector<TModel> models = modelFromMinimalSet(minIndices);

	for(auto it=models.begin(); it!=models.end(); ++it)
	{
		const TModel &model = *it;
		std::unique_ptr<TIterationData> data(new TIterationData());
		int inlierCount;
		float errorSumSq;
		getInliers(model, inlierCount, errorSumSq, *data);

		//Sync here if parallelized
		//if(inlierCount > mBestInlierCount)
		if(errorSumSq < mBestErrorSumSq)
		{
			mBestModel = model;
			mBestInlierCount = inlierCount;
			mBestErrorSumSq = errorSumSq;
			mBestIterationData = std::move(data);
		}
	}
}

}

#endif /* BASERANSAC_H_ */
