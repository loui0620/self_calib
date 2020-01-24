/*
 * cvutils.h
 *
 *  Created on: 6.2.2014
 *      Author: dherrera
 */

#ifndef EUTILS_H_
#define EUTILS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<unsigned char,1,32>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double, 1, 6 >);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d);

namespace Eigen
{
	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3fr;
	typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Matrix4fr;

	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dr;

	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdr;
	typedef Eigen::Matrix<float , Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfr;

	typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ArrayXdr;
	typedef Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ArrayXfr;
}

#include <cassert>
#include <opencv2/core/core.hpp>

namespace planecalib
{

class eutils
{
public:
	static bool IsInside(const Eigen::Vector2i &imageSize, const Eigen::Vector2f &p)
	{
		if (p.x() < 0 || p.y() < 0 || p.x() >= imageSize.x() || p.y() >= imageSize.y())
			return false;
		else
			return true;
	}

	static Eigen::Vector2f HomographyPoint(const Eigen::Matrix3f &H, const Eigen::Vector2f &p)
	{
		return (H*p.homogeneous()).eval().hnormalized();
	}

	static Eigen::Matrix3fr RotationY(float angle)
	{
		Eigen::Matrix3fr m;
		m << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
		return m;
	}

	static Eigen::Matrix3fr RotationX(float angle)
	{
		Eigen::Matrix3fr m;
		m << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
		return m;
	}

	static Eigen::Vector2i FromSize(const cv::Size2i &sz) 
	{
		return Eigen::Vector2i(sz.width, sz.height);
	}
	static cv::Size2i ToSize(const Eigen::Vector2i &sz)
	{
		return cv::Size2i(sz.x(), sz.y());
	}

	template <class T>
	static Eigen::Matrix<T, 2, 1> FromCV(const cv::Point_<T> &p)
	{
		return Eigen::Matrix<T, 2, 1>(p.x,p.y);
	}

	template <class T, int m, int n>
	static Eigen::Matrix<T, m, n> FromCV(const cv::Matx<T, m, n> &mat)
	{
		Eigen::Matrix<T, m, n> res;
		for (int x = 0; x < n; x++)
			for (int y = 0; y < m; y++)
				res(y, x) = mat(y, x);
		return res;
	}

	template <class T, int m, int n>
	static Eigen::Matrix<T, m, n> FromCV(const cv::Mat &mat)
	{
		assert(mat.rows == m && mat.cols == n);

		Eigen::Matrix<T, m, n> res;
		for (int x = 0; x < n; x++)
			for (int y = 0; y < m; y++)
				res(y, x) = mat.at<T>(y, x);
		return res;
	}

	template <class T>
	static cv::Point_<T> ToCVPoint(const Eigen::Matrix<T, 2, 1> &p)
	{
		return cv::Point_<T>(p.x(), p.y());
	}
	template <class T>
	static cv::Point3_<T> ToCVPoint(const Eigen::Matrix<T, 3, 1> &p)
	{
		return cv::Point3_<T>(p.x(), p.y(), p.z());
	}

	template <class T, int m, int n, int options>
	static cv::Matx<T,m,n> ToCV(const Eigen::Matrix<T, m, n, options> &emat)
	{
		cv::Matx<T, m, n> cmat;
		Eigen::Map<Eigen::Matrix3fr> cmat_map(cmat.val);
		cmat_map = emat;
		return cmat;
	}

	static void GetBasis(const Eigen::Vector3f &x, Eigen::Vector3f &basis1, Eigen::Vector3f &basis2)
	{
		const double kThreshold = 0.1;

		//basis1 = cross(C,x)
		//basis2 = cross(basis1,x)
		//Check that the point we use is not colinear with x
		if (x[0] > kThreshold || x[0] < -kThreshold || x[2] > kThreshold || x[2] < -kThreshold)
		{
			//Use C=[0,1,0]
			basis1[0] = x[2];
			basis1[1] = (0);
			basis1[2] = -x[0];

			basis2[0] = -x[0] * x[1];
			basis2[1] = x[0] * x[0] + x[2] * x[2];
			basis2[2] = -x[1] * x[2];
		}
		else
		{
			//Use C=[1,0,0]
			basis1[0] = (0);
			basis1[1] = -x[2];
			basis1[2] = x[1];

			basis2[0] = x[1] * x[1] + x[2] * x[2];
			basis2[1] = -x[0] * x[1];
			basis2[2] = -x[0] * x[2];
		}
		//basis1.normalize();
		//basis2.normalize();
	}

	static Eigen::Matrix3fr GetTranslateHomography(const Eigen::Vector2f &disp)
	{
		Eigen::Matrix3fr T;
		T << 1, 0, disp[0], 0, 1, disp[1], 0, 0, 1;
		return T;
	}
};

}

#endif /* CVUTILS_H_ */
