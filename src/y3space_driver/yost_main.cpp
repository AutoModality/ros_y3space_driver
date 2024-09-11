#include <am_utils/am_ros2_utility.h>
#include <y3space_driver/yost_node.h>
#include <y3space_driver/yost_stats.h>
#include <signal.h>

std::shared_ptr<am::Y3SpaceDriver> yost_class = nullptr;

void signal_handler(int signal)
{

	if(yost_class)
	{
		ROS_INFO("Disconnecting from the serial port");
		yost_class->serialDisConnect();
	}

	rclcpp::shutdown();

	exit(0);
}

std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<am::YostNode> yost_node = std::make_shared<am::YostNode>("y3space_driver");
    std::shared_ptr<am::YostStats> yost_stats = std::make_shared<am::YostStats>(yost_node->stats_list_);
    am::Node::node = yost_node;

    yost_class = std::make_shared<am::Y3SpaceDriver>(yost_stats);
    yost_node->setClass(yost_class);

    ROS_INFO_STREAM(am::Node::node->get_name() << ": running...");

    rclcpp::spin(am::Node::node);

    rclcpp::shutdown();

    return 0;
}