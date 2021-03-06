#ifndef DISTORTION_MODEL_H
#define DISTORTION_MODEL_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "eigen_extensions.h"

class Frustum
{
public:
	Frustum(int smoothing = 1, double bin_depth = 1.0):
		max_dist_(10),
		bin_depth_(bin_depth)
	{
		num_bins_ = ceil(max_dist_ / bin_depth_);
		counts_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
		total_numerators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
		total_denominators_ = Eigen::VectorXf::Ones(num_bins_) * smoothing;
		multipliers_ = Eigen::VectorXf::Ones(num_bins_);
	}
	//! z value, not distance to origin.
	int index(double z) const
	{
		return std::min(num_bins_ - 1, (int)std::floor(z / bin_depth_));
	}

	void undistort(double* z) const
	{
		*z *= multipliers_.coeffRef(index(*z));
	}

	void distort(double* z) const
	{
		*z /= multipliers_.coeffRef(index(*z));
	}

	void interpolatedUndistort(double* z) const
	{
		int idx = index(*z);
		double start = bin_depth_ * idx;
		int idx1;
		if(*z - start < bin_depth_ / 2)
			idx1 = idx;
		else
			idx1 = idx + 1;
		int idx0 = idx1 - 1;
		if(idx0 < 0 || idx1 >= num_bins_ || counts_(idx0) < 50 || counts_(idx1) < 50) {
			undistort(z);
			return;
		}
		double z0 = (idx0 + 1) * bin_depth_ - bin_depth_ * 0.5;

		double coeff1 = (*z - z0) / bin_depth_;
		double coeff0 = 1.0 - coeff1;
		double mult = coeff0 * multipliers_.coeffRef(idx0) + coeff1 * multipliers_.coeffRef(idx1);
		*z *= mult;
	}

	void interpolatedDistort(double* z) const
	{
		int idx = index(*z);
		double start = bin_depth_ * idx;
		int idx1;
		if(*z - start < bin_depth_ / 2)
			idx1 = idx;
		else
			idx1 = idx + 1;
		int idx0 = idx1 - 1;
		if(idx0 < 0 || idx1 >= num_bins_ || counts_(idx0) < 50 || counts_(idx1) < 50) {
			distort(z);
			return;
		}
		double z0 = (idx0 + 1) * bin_depth_ - bin_depth_ * 0.5;

		double coeff1 = (*z - z0) / bin_depth_;
		double coeff0 = 1.0 - coeff1;
		double mult = coeff0 * multipliers_.coeffRef(idx0) + coeff1 * multipliers_.coeffRef(idx1);
		*z *= mult;
	}

	void serialize(std::ostream& out, bool binary = false) const
	{
		if (binary)
		{
			eigen_extensions::serializeScalar(max_dist_, out);
			eigen_extensions::serializeScalar(num_bins_, out);
			eigen_extensions::serializeScalar(bin_depth_, out);
			eigen_extensions::serialize(counts_, out);
			eigen_extensions::serialize(total_numerators_, out);
			eigen_extensions::serialize(total_denominators_, out);
			eigen_extensions::serialize(multipliers_, out);
		}
		else
		{
			out << max_dist_ << std::endl;
			out << num_bins_ << std::endl;
			out << bin_depth_ << std::endl;
			eigen_extensions::serializeASCII(counts_, out);
			eigen_extensions::serializeASCII(total_numerators_, out);
			eigen_extensions::serializeASCII(total_denominators_, out);
			eigen_extensions::serializeASCII(multipliers_, out);
		}
	}

	void deserialize(std::istream& in, bool binary = false)
	{
		if (binary)
		{
			eigen_extensions::deserializeScalar(in, &max_dist_);
			eigen_extensions::deserializeScalar(in, &num_bins_);
			eigen_extensions::deserializeScalar(in, &bin_depth_);
			eigen_extensions::deserialize(in, &counts_);
			eigen_extensions::deserialize(in, &total_numerators_);
			eigen_extensions::deserialize(in, &total_denominators_);
			eigen_extensions::deserialize(in, &multipliers_);
		}
		else
		{
			in >> max_dist_;
			in >> num_bins_;
			in >> bin_depth_;
			eigen_extensions::deserializeASCII(in, &counts_);
			eigen_extensions::deserializeASCII(in, &total_numerators_);
			eigen_extensions::deserializeASCII(in, &total_denominators_);
			eigen_extensions::deserializeASCII(in, &multipliers_);
		}
	}

protected:
	double max_dist_;
	int num_bins_;
	double bin_depth_;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::VectorXf counts_;
	//Eigen::VectorXf total_multipliers_;
	Eigen::VectorXf multipliers_;
	Eigen::VectorXf total_numerators_;
	Eigen::VectorXf total_denominators_;


	friend class DistortionModel;
};

class DistortionModel
{
public:
	DistortionModel() {}
	~DistortionModel()
	{
		deleteFrustums ();
	}
	template<typename PointT>
	void undistort(pcl::PointCloud<PointT> &cloud, double fx=525, double fy=525, double cx=320, double cy=240) const
	{
#pragma omp parallel for
		for(int v = 0; v < cloud.height; ++v) {
			for(int u = 0; u < cloud.width; ++u) {
				PointT &pt = cloud(u, v);
				if(pcl_isnan (pt.z))
					continue;
				double z = pt.z;
				frustum(v, u).interpolatedUndistort(&z);
				pt.z = z;
				pt.x = z * (u - cx) / fx;
				pt.y = z * (v - cy) / fy;
			}
		}

	}

	template<typename PointT>
	void distort(pcl::PointCloud<PointT> &cloud, double fx=525, double fy=525, double cx=320, double cy=240) const
	{
#pragma omp parallel for
		for(int v = 0; v < cloud.height; ++v) {
			for(int u = 0; u < cloud.width; ++u) {
				PointT &pt = cloud(u, v);
				if(pcl_isnan (pt.z))
					continue;
				double z = pt.z;
				frustum(v, u).interpolatedDistort(&z);
				pt.z = z;
				pt.x = z * (u - cx) / fx;
				pt.y = z * (v - cy) / fy;
			}
		}

	}
	void load(const std::string &filename, bool binary = false)
	{
		std::ifstream in (filename.c_str ());
		if (binary)
		{
			eigen_extensions::deserializeScalar(in, &bin_width_);
			eigen_extensions::deserializeScalar(in, &bin_height_);
			eigen_extensions::deserializeScalar(in, &bin_depth_);
			eigen_extensions::deserializeScalar(in, &num_bins_x_);
			eigen_extensions::deserializeScalar(in, &num_bins_y_);
		}
		else
		{
			in >> bin_width_;
			in >> bin_height_;
			in >> bin_depth_;
			in >> num_bins_x_;
			in >> num_bins_y_;
		}

		deleteFrustums ();

		frustums_.resize(num_bins_y_);
		for(size_t y = 0; y < frustums_.size(); ++y) {
			frustums_[y].resize(num_bins_x_, NULL);
			for(size_t x = 0; x < frustums_[y].size(); ++x) {
				frustums_[y][x] = new Frustum;
				frustums_[y][x]->deserialize(in, binary);
			}
		}
		in.close ();
	}

	void save(const std::string &filename, bool binary=false) const
	{
		std::ofstream out (filename.c_str ());
		if (binary)
		{
			eigen_extensions::serializeScalar(bin_width_, out);
			eigen_extensions::serializeScalar(bin_height_, out);
			eigen_extensions::serializeScalar(bin_depth_, out);
			eigen_extensions::serializeScalar(num_bins_x_, out);
			eigen_extensions::serializeScalar(num_bins_y_, out);
		}
		else
		{
			out << bin_width_ << std::endl;
			out << bin_height_ << std::endl;
			out << bin_depth_ << std::endl;
			out << num_bins_x_ << std::endl;
			out << num_bins_y_ << std::endl;
		}

		for(int y = 0; y < num_bins_y_; ++y)
			for(int x = 0; x < num_bins_x_; ++x)
				frustums_[y][x]->serialize(out, binary);
		out.close ();
	}

protected:
	int width_;
	int height_;
	int bin_width_;
	int bin_height_;
	double bin_depth_;
	int num_bins_x_;
	int num_bins_y_;

	//! frustums_[y][x]
	std::vector< std::vector<Frustum*> > frustums_;

	void deleteFrustums()
	{
		for(size_t y = 0; y < frustums_.size(); ++y)
			for(size_t x = 0; x < frustums_[y].size(); ++x)
				if(frustums_[y][x])
					delete frustums_[y][x];
	}
	//! depth is in meters
	Frustum& frustum(int y, int x)
	{
		int xidx = x / bin_width_;
		int yidx = y / bin_height_;
		return (*frustums_[yidx][xidx]);
	}
	const Frustum& frustum(int y, int x) const
	{
		int xidx = x / bin_width_;
		int yidx = y / bin_height_;
		return (*frustums_[yidx][xidx]);
	}
};

#endif // DISTORTION_MODEL_H

