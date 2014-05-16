#include "sim_kinect.hpp"

SimKinect::SimKinect(int argc, char** argv, const std::string& filename)
{
	height_ = 640;
	width_ = 480;

	//initializeGL(argc, argv);

	initializeOpenRave(filename);

	/*
	camera_ = pcl::simulation::Camera::Ptr (new pcl::simulation::Camera());
	scene_ = pcl::simulation::Scene::Ptr (new pcl::simulation::Scene());

	rl_ = pcl::simulation::RangeLikelihood::Ptr (new pcl::simulation::RangeLikelihood (1, 1, height_, width_, scene_));

	// TODO: Check camera intrinsics -- Different values reported for the Kinect
	rl_->setCameraIntrinsicsParameters (width_,height_, 576.09757860, 576.09757860, 321.06398107, 242.97676897);

	rl_->setComputeOnCPU (false);
	rl_->setSumOnCPU (true);
	rl_->setUseColor (true);

	// roll, pitch, yaw
	camera_->set(0.471703, 1.59862, 3.10937, 0, 0.418879, -12.2129);

	// TODO: Figure out what t_gamma is?
	for (int i=0; i<2048; i++)
	{
		float v = (float)i/2048.0;
		v = v*v*v*6;
		t_gamma[i] = v*6*256;
	}
	*/
}

void SimKinect::initializeGL(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);// was GLUT_RGBA
	glutInitWindowPosition (10, 10);
	glutInitWindowSize (10, 10);
	//glutInitWindowSize (width_, height_);
	glutCreateWindow ("OpenGL range likelihood");

	GLenum err = glewInit ();
	if (GLEW_OK != err)
	{
		std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
		exit (-1);
	}

	std::cout << "Status: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;
	if (glewIsSupported ("GL_VERSION_2_0"))
		std::cout << "OpenGL 2.0 supported" << std::endl;
	else
	{
		std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
		exit(1);
	}

	std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;
	const GLubyte* version = glGetString (GL_VERSION);
	std::cout << "OpenGL Version: " << version << std::endl;
}

void SimKinect::initializeOpenRave(const std::string& filename)
{
	RaveInitialize(true, OpenRAVE::Level_Info);
	OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
	bool success = env->Load(filename);
	ALWAYS_ASSERT(success);

	std::vector<OpenRAVE::RobotBasePtr> robots;
	env->GetRobots(robots);
	OpenRAVE::RobotBasePtr robot = robots[0];

	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env->GetBodies(bodies);
	for (int i=0; i < bodies.size(); ++i) {
		OpenRAVE::KinBody& body = *bodies[i];
		LOG_INFO("Parsing kinbody %s", body.GetName().c_str());

		// TODO: Generalize to all objects in the scene except for the robot (PR2)
		if (body.GetName().compare("mug")==0)
		{
			const std::vector<OpenRAVE::KinBody::LinkPtr>& links = body.GetLinks();
			for (int i=0; i < links.size(); ++i) {
				LOG_INFO("    Parsing kinbody link %s", links[i]->GetName().c_str());
				if (links[i]->GetGeometries().size() > 0) {
					extractGeomFromLink(*links[i]);
				}
			}
		}
	}
}

/*
void convertToIsometry3d(const OpenRAVE::Transform& T, Eigen::Isometry3d* m) {
	OpenRAVE::RaveTransformMatrix orT(T);
}

osg::Node* osgNodeFromGeom(const OpenRAVE::KinBody::Link::Geometry& geom)
{
	osg::Geode* geode = new osg::Geode;

	osg::StateSet* state = geode->getOrCreateStateSet();
	osg::Material* mat = new osg::Material;
	OpenRAVE::Vector diffuse = geom.GetDiffuseColor();
	mat->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4(diffuse.x, diffuse.y, diffuse.z, 1) );
	OpenRAVE::Vector amb = geom.GetAmbientColor();
	mat->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4(amb.x,amb.y,amb.z,1)*.5 );
	mat->setTransparency(osg::Material::FRONT_AND_BACK,geom.GetTransparency());
	state->setAttribute(mat);

	osgUtil::SmoothingVisitor sv;
	geode->accept(sv);

	return geode;
}
*/

void SimKinect::modelFromOpenRaveMesh(const OpenRAVE::KinBody::Link::TRIMESH& mesh)
{

	std::cout << "Num vertices in mug: " << mesh.vertices.size() << std::endl;
	std::cout << "Vertices: " << std::endl;
	for(int i = 0; i < mesh.vertices.size(); ++i) {
		const OpenRAVE::Vector& v = mesh.vertices[i];
		//std::cout << v.x << " " << v.y << " " << v.z  << std::endl;
	}

	std::cout << "Num indices in mug: " << mesh.indices.size() << std::endl;
	std::cout << "Indices: " << std::endl;

	/*
	osg::Vec3Array* vec = new osg::Vec3Array();
	vec->resize( mesh.vertices.size());
	for(int i = 0; i < mesh.vertices.size(); ++i) {
		const Vector& v = mesh.vertices[i];
		(*vec)[i].set( v.x, v.y, v.z );
	}

	osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_LINE_STRIP );
	for(int i = 0; i < mesh.indices.size(); ++i)
		deui->push_back( mesh.indices[ i ] );


	osg::Vec4Array* color = new osg::Vec4Array();
	color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

	osg::Geometry* geom = new osg::Geometry;
	geom->setVertexArray( vec );
	geom->setColorArray( color );
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );

	geom->addPrimitiveSet( deui );




	pcl::simulation::Vertices vertices;
	pcl::simulation::Indices indices;

	// Assume by default that color information is available
	    pcl::PointCloud<pcl::PointXYZRGB> newcloud;
	    pcl::fromPCLPointCloud2 (plg->cloud, newcloud);

	    PCL_DEBUG("RGB Triangle mesh: ");
	    PCL_DEBUG("Mesh polygons: %ld", plg->polygons.size ());
	    PCL_DEBUG("Mesh points: %ld", newcloud.points.size ());

	    Eigen::Vector4f tmp;
	    for(size_t i=0; i< plg->polygons.size (); ++i)
	    { // each triangle/polygon
	      pcl::Vertices apoly_in = plg->polygons[i];
	      for(size_t j = 0; j < apoly_in.vertices.size (); ++j)
	      { // each point
	        uint32_t pt = apoly_in.vertices[j];
	        tmp = newcloud.points[pt].getVector4fMap();
	        vertices.push_back (Vertex (Eigen::Vector3f (tmp (0), tmp (1), tmp (2)),
	                                    Eigen::Vector3f (newcloud.points[pt].r/255.0f,
	                                                     newcloud.points[pt].g/255.0f,
	                                                     newcloud.points[pt].b/255.0f)));
	        indices.push_back (indices.size ());
	      }
	    }
	  }

	  PCL_DEBUG("Vertices: %ld", vertices.size ());
	  PCL_DEBUG("Indices: %ld", indices.size ());

	  glGenBuffers (1, &vbo_);
	  glBindBuffer (GL_ARRAY_BUFFER, vbo_);
	  glBufferData (GL_ARRAY_BUFFER, vertices.size () * sizeof (vertices[0]), &(vertices[0]), GL_STATIC_DRAW);
	  glBindBuffer (GL_ARRAY_BUFFER, 0);

	  glGenBuffers (1, &ibo_);
	  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo_);
	  glBufferData (GL_ELEMENT_ARRAY_BUFFER, indices.size () * sizeof (indices[0]), &(indices[0]), GL_STATIC_DRAW);
	  glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);

	  if (indices.size () > std::numeric_limits<GLuint>::max ())
	    PCL_THROW_EXCEPTION(PCLException, "Too many vertices");

	  size_ = static_cast<GLuint>(indices.size ());
	*/

}

void SimKinect::extractGeomFromORGeom(const OpenRAVE::KinBody::Link::Geometry& geom)
{

	/*

	// 2. read mesh and setup model:
	std::cout << "About to read: " << argv[2] << std::endl;
	pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
	pcl::io::loadPolygonFile (argv[2], mesh);
	pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));

	// Not sure if PolygonMesh assumes triangles if to, TODO: Ask a developer
	PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud));
	scene_->add (model);

	std::cout << "Just read " << argv[2] << std::endl;
	std::cout << mesh.polygons.size () << " polygons and "
			<< mesh.cloud.data.size () << " triangles\n";
	*/


	switch(geom.GetType())
	{
	//  Extract geometry from collision Mesh
	case OpenRAVE::KinBody::Link::GEOMPROPERTIES::GeomTrimesh:{
		modelFromOpenRaveMesh(geom.GetCollisionMesh());
		//pcl::simulation::TriangleMeshModel::Ptr model =
		//scene_->add(model);
		break;
	}
	default:
		LOG_ERROR("Not implemented handler for geometry of type %i", geom.GetType());
		break;
	}
}

void SimKinect::extractGeomFromLink(const OpenRAVE::KinBody::Link& link) {
	// each geom is a child
	const std::vector<OpenRAVE::KinBody::Link::GeometryPtr>& geoms = link.GetGeometries();
	for (int i=0; i < geoms.size(); ++i)
	{
		extractGeomFromORGeom(*geoms[i]);

		// extract transform here from geoms[i]->GetTransform()

		// Model extraction part here
	}
}

/*
void SimKinect::getMeasurement()
{
	float* reference = new float[rl_->getRowHeight() * rl_->getColWidth()];
	const float* depth_buffer = rl_->getDepthBuffer();
	// Copy one image from our last as a reference.
	for (int i=0, n=0; i<rl_->getRowHeight(); ++i)
	{
		for (int j=0; j<rl_->getColWidth(); ++j)
		{
			reference[n++] = depth_buffer[i*rl_->getWidth() + j];
		}
	}

	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
	std::vector<float> scores;
	int n = 1;
	poses.push_back (pose_in);
	rl_->computeLikelihoods (reference, poses, scores);
	std::cout << "camera: " << camera_->getX ()
    		   << " " << camera_->getY ()
    		   << " " << camera_->getZ ()
    		   << " " << camera_->getRoll ()
    		   << " " << camera_->getPitch ()
    		   << " " << camera_->getYaw ()
    		   << std::endl;

	delete [] reference;
}

void SimKinect::writeScoreImage(const float* score_buffer, const std::string& fname)
{
	int npixels = rl_->getWidth() * rl_->getHeight();
	uint8_t* score_img = new uint8_t[npixels * 3];

	float min_score = score_buffer[0];
	float max_score = score_buffer[0];
	for (int i=1; i<npixels; i++)
	{
		if (score_buffer[i] < min_score) min_score = score_buffer[i];
		if (score_buffer[i] > max_score) max_score = score_buffer[i];
	}

	for (int y = 0; y <  height_; ++y)
	{
		for (int x = 0; x < width_; ++x)
		{
			int i = y*width_ + x ;
			int i_in= (height_-1 -y) *width_ + x ; // flip up

			float d = (score_buffer[i_in]-min_score)/(max_score-min_score);
			score_img[3*i+0] = 0;
			score_img[3*i+1] = d*255;
			score_img[3*i+2] = 0;
		}
	}

	// Write to file:
	pcl::io::saveRgbPNGFile (fname, score_img, width_, height_);

	delete [] score_img;
}

void SimKinect::writeDepthImage(const float* depth_buffer, const std::string& fname)
{
	int npixels = rl_->getWidth() * rl_->getHeight();
	uint8_t* depth_img = new uint8_t[npixels * 3];

	float min_depth = depth_buffer[0];
	float max_depth = depth_buffer[0];
	for (int i=1; i<npixels; i++)
	{
		if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
		if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
	}

	for (int y = 0; y <  height_; ++y)
	{
		for (int x = 0; x < width_; ++x)
		{
			int i= y*width_ + x ;
			int i_in= (height_-1 -y) *width_ + x ; // flip up down


			float zn = 0.7;
			float zf = 20.0;
			float d = depth_buffer[i_in];
			float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
			float b = 0.075;
			float f = 580.0;
			uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
			if (kd < 0) kd = 0;
			else if (kd>2047) kd = 2047;

			int pval = t_gamma[kd];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255-lb;
				break;
			case 1:
				depth_img[3*i+0] = 255;
				depth_img[3*i+1] = lb;
				depth_img[3*i+2] = 0;
				break;
			case 2:
				depth_img[3*i+0] = 255-lb;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = 0;
				break;
			case 3:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255;
				depth_img[3*i+2] = lb;
				break;
			case 4:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 255-lb;
				depth_img[3*i+2] = 255;
				break;
			case 5:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 255-lb;
				break;
			default:
				depth_img[3*i+0] = 0;
				depth_img[3*i+1] = 0;
				depth_img[3*i+2] = 0;
				break;
			}
		}
	}

	// Write to file:
	pcl::io::saveRgbPNGFile (fname, depth_img, width_, height_);

	delete [] depth_img;
}


void SimKinect::writeDepthImageUint(const float* depth_buffer, const std::string& fname)
{
	int npixels = rl_->getWidth() * rl_->getHeight();
	unsigned short * depth_img = new unsigned short[npixels ];

	float min_depth = depth_buffer[0];
	float max_depth = depth_buffer[0];
	for (int i=1; i<npixels; i++)
	{
		if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
		if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
	}

	for (int y = 0; y <  height_; ++y)
	{
		for (int x = 0; x < width_; ++x)
		{
			int i= y*width_ + x ;
			int i_in= (height_-1 -y) *width_ + x ; // flip up down

			float zn = 0.7;
			float zf = 20.0;
			float d = depth_buffer[i_in];

			unsigned short z_new = (unsigned short)  floor( 1000*( -zf*zn/((zf-zn)*(d - zf/(zf-zn)))));
			if (z_new < 0) z_new = 0;
			else if (z_new>65535) z_new = 65535;

			if ( z_new < 18000){
				std::cout << z_new << " " << d << " " << x << "\n";
			}

			float z = 1000*( -zf*zn/((zf-zn)*(d - zf/(zf-zn))));
			float b = 0.075;
			float f = 580.0;
			uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
			if (kd < 0) kd = 0;
			else if (kd>2047) kd = 2047;

			int pval = t_gamma[kd];
			int lb = pval & 0xff;
			depth_img[i] = z_new;
		}
	}

	// Write to file:
	pcl::io::saveShortPNGFile (fname, depth_img, width_, height_, 1);

	delete [] depth_img;
}


void SimKinect::writeRgbImage(const uint8_t* rgb_buffer, const std::string& fname)
{
	int npixels = rl_->getWidth() * rl_->getHeight();
	uint8_t* rgb_img = new uint8_t[npixels * 3];

	for (int y = 0; y <  height_; ++y)
	{
		for (int x = 0; x < width_; ++x)
		{
			int px= y*width_ + x ;
			int px_in= (height_-1 -y) *width_ + x ; // flip up down
			rgb_img [3* (px) +0] = rgb_buffer[3*px_in+0];
			rgb_img [3* (px) +1] = rgb_buffer[3*px_in+1];
			rgb_img [3* (px) +2] = rgb_buffer[3*px_in+2];
		}
	}

	// Write to file:
	pcl::io::saveRgbPNGFile (fname, rgb_img, width_, height_);

	delete [] rgb_img;
}
*/
