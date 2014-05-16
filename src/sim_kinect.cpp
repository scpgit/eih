#include "sim_kinect.hpp"

class KinBodyGroup {
public:
	std::vector<OpenRAVE::KinBody::LinkPtr> links; // links with geometry
	std::vector<Eigen::Isometry3d*> trans; // corresponding nodes
	//void update() {
	//	for (int i=0; i < links.size(); ++i) {
	//		trans[i] = links[i]->GetTransform();
	//	}
	//}
};

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

void extractGeomFromORGeom(const OpenRAVE::KinBody::Link::Geometry& geom)
{
	switch(geom.GetType())
	{
	//  Extract geometry from collision Mesh
	case OpenRAVE::KinBody::Link::GEOMPROPERTIES::GeomTrimesh:{
		OpenRAVE::KinBody::Link::TRIMESH& mesh = geom.GetCollisionMesh();

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
		break;
	}
	default:
		LOG_ERROR("Not implemented handler for geometry of type %i", geom.GetType());
		break;
	}
}

void extractGeomFromLink(const OpenRAVE::KinBody::Link& link) {
	// each geom is a child
	const std::vector<OpenRAVE::KinBody::Link::GeometryPtr>& geoms = link.GetGeometries();
	for (int i=0; i < geoms.size(); ++i)
	{
		extractGeomFromORGeom(*geoms[i]);

		// extract transform here from geoms[i]->GetTransform()

		// Model extraction part here
	}
}

int main()
{
	RaveInitialize(true, OpenRAVE::Level_Info);
	OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
	bool success = env->Load("/home/sachin/Workspace/eih/data/pr2-test.env.xml");
	ALWAYS_ASSERT(success);

	std::vector<OpenRAVE::RobotBasePtr> robots;
	env->GetRobots(robots);
	OpenRAVE::RobotBasePtr robot = robots[0];

	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env->GetBodies(bodies);
	for (int i=0; i < bodies.size(); ++i) {
		OpenRAVE::KinBody& body = *bodies[i];
		LOG_INFO("Parsing kinbody %s", body.GetName().c_str());

		/*
		// each link is a child
		KinBodyGroup* group = new KinBodyGroup;
		const std::vector<OpenRAVE::KinBody::LinkPtr>& links = body.GetLinks();
		for (int i=0; i < links.size(); ++i) {
			LOG_INFO("Parsing kinbody link %s", links[i]->GetName().c_str());
			if (links[i]->GetGeometries().size() > 0) {
				// osgNodeFromLink(*links[i]);
			}
		}
		*/

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

	std::cin.get();
}
