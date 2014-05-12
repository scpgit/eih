#include "sim_kinect.hpp"
#include <openrave-core.h>
#include <iostream>
#include <fstream>
#include "osgviewer/osgviewer.hpp"
using namespace OpenRAVE;
using namespace std;


int main() {
	RaveInitialize(true, OpenRAVE::Level_Debug);
	EnvironmentBasePtr env = RaveCreateEnvironment();
	bool success = env->Load("/home/sachin/Workspace/eih/data/pr2-table.env.xml");
	ALWAYS_ASSERT(success);

	vector<RobotBasePtr> robots;
	env->GetRobots(robots);
	RobotBasePtr robot = robots[0];

	OSGViewerPtr viewer(new OSGViewer(env));

	FakeKinect fk(viewer->m_root.get());
	viewer->UpdateSceneData();

	OpenRAVE::Transform T;
	T.trans = OpenRAVE::Vector(0,0,3);
	T.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0,1,0)*M_PI);
	fk.SetPose(T);

	cin.get();

	ofstream out("/tmp/depthimg.txt");
	out << (*fk.GetDepthImage()) << endl;

	viewer->Idle();
}
