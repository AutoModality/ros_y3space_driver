#include <y3space_driver/Y3SpaceDriver.h>
#include <thread>
#include <memory>
#include <signal.h>
#include <am_utils/am_ros2_utility.h>

std::string NODE_NAME_ = "Y3SpaceDriver";


std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;

int main(int argc, char **argv)
{

  	rclcpp::init(argc, argv);
  	
	am::Node::node = std::make_shared<am::AMLifeCycle>(NODE_NAME_);

	std::shared_ptr<Y3SpaceDriver> y3_space_driver = std::make_shared<Y3SpaceDriver>();

    rclcpp::spin(am::Node::node);

	rclcpp::shutdown();

	return 0;
}
