#include "sim_kinect.hpp"

int main(int argc, char** argv)
{
	SimKinect::Ptr kinect(new SimKinect(argc, argv, "/home/sachin/Workspace/eih/data/pr2-test.env.xml"));

	std::cin.get();
}
