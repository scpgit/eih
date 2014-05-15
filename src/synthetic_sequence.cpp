#include <cmath>
#include <synthetic_sequence.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <eigen_extensions.h>
#include <boost/random.hpp>

/////////////////////////
// convenience
// /////////////////////
//
bool
getTimestampFromFilepath (
		const std::string &filepath,
		double &timestamp)
{
	// For now, we assume the file is of the form frame_[22-char POSIX timestamp]_*
	char timestamp_str[256];
	int result = std::sscanf (boost::filesystem::basename(boost::filesystem::path(filepath)).c_str (),"frame_%22s_%*s",timestamp_str);
	if (result > 0)
	{
		boost::posix_time::ptime cur_date = boost::posix_time::from_iso_string (timestamp_str);
		boost::posix_time::ptime zero_date (
				boost::gregorian::date (1970,boost::gregorian::Jan,1));
		timestamp = static_cast<double> ((cur_date - zero_date).total_microseconds ()) * 1E-6;
		return (true);
	}
	return (false);
}

///////////////////////////
synth::SyntheticSequence::SyntheticSequence (const std::string &dir):
		  noise_bitmask_ (SYNTH_HIGH_FREQ | SYNTH_DISCRETIZE | SYNTH_LOW_FREQ),
		  has_distortion_model_ (false)
{
	std::vector<std::string> pose_files;
	boost::filesystem::directory_iterator end_itr;
	for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
	{
		std::string extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->path ()));
		std::string basename = boost::filesystem::basename(itr->path());
		std::string pathname = itr->path().string ();
		if (extension == ".pcd" || extension == ".PCD")
		{
			pcd_files_.push_back (pathname);
		}
		else if (extension == ".txt" || extension == ".TXT")
		{
			pose_files.push_back (pathname);
		}
	}
	std::sort (pcd_files_.begin (), pcd_files_.end ());
	std::sort (pose_files.begin (), pose_files.end ());
	assert (pose_files.size () == pcd_files_.size ());
	// Grab motions
	poses_.resize (pcd_files_.size ());
	timestamps_.resize (pcd_files_.size ());
	for (size_t i = 0; i < poses_.size (); i++)
	{
		Eigen::Matrix4f pose_mat;
		eigen_extensions::loadASCII (pose_files[i], &pose_mat);
		poses_[i] = pose_mat;
		// Pull timestamp from string
		assert (getTimestampFromFilepath (pose_files[i], timestamps_[i]));
	}

}

bool
reprojectPoint (const pcl::PointXYZRGBA &pt, int &u, int &v)
{
	float focal_length_x_ = 525.;
	float focal_length_y_ = 525.;
	float principal_point_x_ = 320;
	float principal_point_y_ = 240;
	u = (pt.x * focal_length_x_ / pt.z) + principal_point_x_;
	v = (pt.y * focal_length_y_ / pt.z) + principal_point_y_;
	return (pt.z > 0 && u >= 0 && u < 640 && v >= 0 && v < 480);
}

void
remapCloud (pcl::PointCloud<pcl::PointXYZRGBA> const &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out)
{
	cloud_out = pcl::PointCloud<pcl::PointXYZRGBA> (640, 480);
	// Initialize to nan
#pragma omp parallel for
	for (size_t i = 0; i < cloud_out.size (); i++)
	{
		cloud_out[i].z = std::numeric_limits<float>::quiet_NaN ();
	}
	cloud_out.is_dense = false;
	for (size_t i = 0; i < cloud_in.size (); i++)
	{
		const pcl::PointXYZRGBA &pt = cloud_in.at (i);
		if (isnan (pt.x) ||isnan (pt.y) || isnan (pt.z))
			continue;
		int u, v;
		if (!reprojectPoint (pt, u, v))
			continue;
		pcl::PointXYZRGBA &pt_out = cloud_out (u, v);
		if (isnan (pt_out.z) || pt_out.z > pt.z)
			pt_out = pt;
	}
}


/////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
synth::SyntheticSequence::cloudAt (size_t frame, bool distort) const
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_orig (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile (pcd_files_[frame], *cloud_orig);
	// Remap
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	bool remap = false;
	if (remap)
	{
		cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
		remapCloud (*cloud_orig, *cloud);
	}
	else
		cloud = cloud_orig;
	if (vis_)
		visualizeCloud (cloud, "Undistorted");
	if (distort)
		distortCloud (*cloud);
	if (vis_)
		visualizeCloud (cloud, "Distorted");
	return (cloud);
}

/////////////////////////
Eigen::Affine3f 
synth::SyntheticSequence::poseAt (size_t frame) const
{
	return (poses_[frame]);
}
////////////////////
size_t
synth::SyntheticSequence::size () const
{
	return (pcd_files_.size ());
}

/////////////////
double
synth::SyntheticSequence::timestampAt (size_t frame)
{
	return (timestamps_[frame]);
}

/////////////////////////////////////////
void
synth::SyntheticSequence::distortCloud (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
	if (noise_bitmask_ & SYNTH_LOW_FREQ)
		addLowFrequencyDistortion (cloud);
	if (noise_bitmask_ & SYNTH_HIGH_FREQ)
		addHighFrequencyNoise (cloud);
	if (noise_bitmask_ & SYNTH_SHADOW_POINTS)
		addShadowPoints (cloud);
	if (noise_bitmask_ & SYNTH_DISCRETIZE)
		discretizeCloud (cloud);
	reprojectToPinhole (cloud);
}

/////////////////////////////////////////
void
synth::SyntheticSequence::addShadowPoints (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
#pragma omp parallel for
	for (size_t i = 0; i < cloud.size (); i++)
	{
		pcl::PointXYZRGBA &pt = cloud.at (i);

	}
}

//! Sample from a zero-mean, unit variance normal distribution of dimension 1
namespace
{
float
normalSample(float mean, float std_dev)
{
	float total = 0;
	for(size_t i = 0; i < 12; i++)
		total += (float)rand()/(float)RAND_MAX;
	float unit_noise = total - 6.;
	return (mean + std_dev * unit_noise);
}
}
/////////////////////////////////////////
void
synth::SyntheticSequence::addHighFrequencyNoise (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
	// Using the results of Nguyen, Izadi, Lovell 2012
	pcl::console::TicToc tt;
	tt.tic ();
#pragma omp parallel
	{
		boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator (
				boost::mt19937 (int (time (0))^omp_get_thread_num ()),
				boost::normal_distribution<> ());
#pragma omp for
for (size_t i = 0; i < cloud.size (); i++)
{
	pcl::PointXYZRGBA &pt = cloud.at (i);
	if (isnan (pt.z))
		continue;
	float sigma_z = 0.0012 + 0.0019 * (pt.z - 0.4)*(pt.z - 0.4);
	pt.z += sigma_z * generator ();
	//pt.z = normalSample (pt.z, sigma_z);
}
	}
}

/////////////////////////////////////////
void
synth::SyntheticSequence::addLowFrequencyDistortion (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
	if (!has_distortion_model_)
	{
		PCL_WARN ("[synth::SyntheticSequence::addLowFrequencyDistortion] No disortion model provided!\n");
		return;
	}
	if (rss_distortion_)
	{
		distortion_model_rss_.distort (cloud);
	}
#pragma omp parallel for
	for (int u = 0; u < cloud.width; u++)
	{
		for (int v = 0; v < cloud.height; v++)
		{
			pcl::PointXYZRGBA &pt = cloud (u, v);
			if (pcl_isnan(distortion_model_(v, u)))
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
			else
				pt.z *= distortion_model_(v, u);
		}
	}
}

/////////////////////////////////////////
void
synth::SyntheticSequence::discretizeCloud (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
	// Using Z = FB/D && 1/8 pixel measurements
	// (int) 8D = FB / Z
	float FB = 525.*0.075;
#pragma omp parallel for
	for (size_t i = 0; i < cloud.size (); i++)
	{
		pcl::PointXYZRGBA &pt = cloud.at (i);
		if (isnan (pt.z))
			continue;
		float disparity = FB / pt.z;
		// Disparity must be stored in 1/8th pixels
		disparity = std::ceil (8*disparity) / 8.;
		pt.z = FB / disparity;
	}

}

/////////////////////////////////////////////////////////////////////
void
synth::SyntheticSequence::reprojectToPinhole (pcl::PointCloud<pcl::PointXYZRGBA> &cloud) const
{
	// TODO factor out fx, fy, cx, cy
#pragma omp parallel for
	for (int x = 0; x < cloud.width; x++)
	{
		for (int y = 0; y < cloud.height; y++)
		{
			pcl::PointXYZRGBA &pt = cloud (x,y);
			if (isnan (pt.z))
				continue;
			pt.x = static_cast<float> (x - 320)*pt.z / 525.;
			pt.y = static_cast<float> (y - 240)*pt.z / 525.;
		}
	}
	cloud.is_dense = false;
}

/////////////////
void
synth::SyntheticSequence::visualizeCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
		const std::string &caption) const
{
	if (!vis_->updatePointCloud (cloud, "cloud"))
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> handler (cloud, 127, 127, 127);
		vis_->addPointCloud (cloud, handler, "cloud");
	}
	PCL_INFO ("Visualizing: %s\n", caption.c_str ());
	vis_->spin ();
}

//////////////////
void
synth::SyntheticSequence::setDistortionModel (const std::string &distortion_model, const std::string &rss_dist)
{
	rss_distortion_ = (rss_dist != std::string ("NONE"));
	//rss_distortion_ = boost::filesystem::extension (distortion_model).compare(".eig") != 0;
	if (rss_distortion_)
		distortion_model_rss_.load (rss_dist);
	eigen_extensions::load (distortion_model, &distortion_model_);
	has_distortion_model_ = true;
}

