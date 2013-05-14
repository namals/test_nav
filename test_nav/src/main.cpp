#include <test_nav/NavManager.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_nav");
	NavManager nav_manager;
	ros::spin();
	return 0;
}
