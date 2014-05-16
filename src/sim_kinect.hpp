#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <openrave-core.h>
#include <iostream>
#include <fstream>
#include <utils/macros.h>
#include "utils/logging.hpp"
#include "utils/openrave_userdata_utils.hpp"

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <GL/glew.h>
#include <pcl/pcl_config.h>

#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif
#ifdef GLUT_IS_A_FRAMEWORK
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/range_likelihood.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/range_image/range_image_planar.h>

#include <pcl/visualization/cloud_viewer.h>

class SimKinect {
public:
	SimKinect(osg::Group* root);
	void SetPose(const OpenRAVE::Transform& pose);
	void SetIntrinsics(float f);
	void Update();
	float* GetDepthImage();
	unsigned char* GetColorImage(); // XXX this is bad if the row size is not a multiple of 4

	uint16_t t_gamma[2048];

	Scene::Ptr scene_;
	Camera::Ptr camera_;
	RangeLikelihood::Ptr range_likelihood_;

	int window_width_;
	int window_height_;
	bool paused_;
	bool write_file_;
};
