#include <Y3SpaceDriver.h>

std::string NODE_NAME_ = "Y3SpaceDriver";

int main(int argc, char **argv)
{
  	ros::init(argc, argv, NODE_NAME_);
  	ros::NodeHandle nh;
  	ros::NodeHandle pnh("~");

  	std::string port;
  	int baudrate;
  	int timeout;
  	std::string mode;
  	std::string frame;

    pnh.param<std::string>("port", port, "/dev/ttyACM0");
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<int>("timeout", timeout, 60000);
    pnh.param<std::string>("mode", mode, "relative");
    pnh.param<std::string>("frame", frame, "imu_link");

  	Y3SpaceDriver driver(nh, pnh, port, baudrate, timeout, mode, frame);
  	driver.run();
}
