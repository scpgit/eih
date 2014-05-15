#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "distortion_model.h"

#include <boost/shared_ptr.hpp>

#include <string>

namespace synth
{
//! Noise model
enum
{
	SYNTH_SHADOW_POINTS=1,
	SYNTH_HIGH_FREQ=2,
	SYNTH_LOW_FREQ=4,
	SYNTH_DISCRETIZE=8,
	SYNTH_NOISY_TRAJ=16
};

class SyntheticSequence
{
public:
	typedef boost::shared_ptr<SyntheticSequence> Ptr;
	typedef boost::shared_ptr<const SyntheticSequence> ConstPtr;

	//! Load from directory
	SyntheticSequence (const std::string &dir);

	// Get the cloud at a specified frame
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
	cloudAt (size_t frame, bool distort=false) const;

	//! Get the transform
	Eigen::Affine3f
	poseAt (size_t frame) const;

	//! Get timestamp
	double
	timestampAt (size_t frame);

	//! Get the size
	size_t
	size () const;

	//! Set a bitmask of the noise model
	inline void
	setNoiseBitmask (int noise_bitmask)
	{noise_bitmask_ = noise_bitmask;}

	inline void
	setVisualizer (pcl::visualization::PCLVisualizer::Ptr vis)
	{vis_ = vis;}

	void
	setDistortionModel (const std::string &distortion_model, const std::string &rss_dist="NONE");

protected:
	//! Distort a cloud in place
	void
	distortCloud (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;
	//! Apply shadow points to cloud
	void
	addShadowPoints (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;

	//! Apply high frequency noise
	void
	addHighFrequencyNoise (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;

	//! Apply nonlinear distortion
	void
	addLowFrequencyDistortion (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;

	//! Re-discretize cloud
	void
	discretizeCloud (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;

	void
	reprojectToPinhole (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const;

	//! Visualization
	void
	visualizeCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const std::string &caption) const;


	std::vector<std::string> pcd_files_;
	std::vector<Eigen::Affine3f> poses_;
	std::vector<double> timestamps_;
	int noise_bitmask_;
	Eigen::MatrixXf distortion_model_;
	bool has_distortion_model_;
	bool rss_distortion_;
	DistortionModel distortion_model_rss_;

	pcl::visualization::PCLVisualizer::Ptr vis_;
};
}
