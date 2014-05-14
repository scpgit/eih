#include "sim_kinect.hpp"
#include <openrave-core.h>
#include <iostream>
#include <fstream>
#include "osgviewer/osgviewer.hpp"
using namespace OpenRAVE;
using namespace std;


int main() {
	RaveInitialize(true, OpenRAVE::Level_Info);
	EnvironmentBasePtr env = RaveCreateEnvironment();
	bool success = env->Load("/home/sachin/Workspace/eih/data/pr2-test.env.xml");
	ALWAYS_ASSERT(success);

	vector<RobotBasePtr> robots;
	env->GetRobots(robots);
	RobotBasePtr robot = robots[0];

	std::vector<double> dofval;
	std::vector<int> dofind;
	robot->GetDOFValues(dofval);

	for(std::vector<double>::iterator it = dofval.begin(); it != dofval.end(); ++it) {
		std::cout << *it << std::endl;
	}

	OSGViewerPtr viewer(new OSGViewer(env));

	FakeKinect fk(viewer->m_root.get());
	viewer->UpdateSceneData();

	cin.get();

	OpenRAVE::Transform T;
	T.trans = OpenRAVE::Vector(0,0,3);
	T.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0,1,0)*M_PI);
	fk.SetPose(T);

	ofstream out("/tmp/depthimg.txt");
	out << (*fk.GetDepthImage()) << endl;

	viewer->Idle();
}
