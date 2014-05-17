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

class SimKinect {
public:
	typedef boost::shared_ptr<SimKinect> Ptr;
	typedef boost::shared_ptr<const SimKinect> ConstPtr;

	SimKinect(int argc, char** argv, const std::string& filename);

	void initializeGL(int argc, char** argv);
	void initializeOpenRave(const std::string& filename);

	void setPose(const Eigen::Isometry3d& pose);
	void setIntrinsics(float f);

    void getMeasurement();

    void writeScoreImage(const float* score_buffer, const std::string& fname);
    void writeDepthImage(const float* depth_buffer, const std::string& fname);
    void writeDepthImageUint(const float* depth_buffer, const std::string& fname);
    void writeRgbImage(const uint8_t* rgb_buffer, const std::string& fname);

    // OpenRave utils
    void extractGeomFromLink(const OpenRAVE::KinBody::Link& link);
    void extractGeomFromORGeom(const OpenRAVE::KinBody::Link::Geometry& geom, const OpenRAVE::Transform& T);
    void trimeshFromOpenRaveMesh(const OpenRAVE::KinBody::Link::Geometry& geom, const OpenRAVE::Transform& T);

public:
	pcl::simulation::Scene::Ptr scene_;
	pcl::simulation::Camera::Ptr camera_;
	pcl::simulation::RangeLikelihood::Ptr rl_;

private:
	uint16_t t_gamma[2048];

	int width_;
	int height_;
};
