#include <Y3SpaceDriver.h>
#include <thread>
#include <memory>
#include <signal.h>

std::shared_ptr<std::thread> imu_thread;
std::shared_ptr<Y3SpaceDriver> imu_driver;

bool stop_signal;
std::string NODE_NAME_ = "Y3SpaceDriver";

//!
//! IMU Thread Function
//!
void imu_thread_function()
{
	//Initialize
	imu_driver->initStream();
	while(!stop_signal)
	{
		imu_driver->readAndPublish();
	}
}


//!
//! Signal Handler Method
//!
void signal_handler(int signal)
{
	stop_signal = true;
	ros::shutdown();
	exit(0);
}

int main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);
  	ros::init(argc, argv, NODE_NAME_, ros::init_options::NoSigintHandler);
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

  	imu_driver = std::make_shared<Y3SpaceDriver>(nh, pnh, port, baudrate, timeout, mode, frame);
  	//driver.run();

  	stop_signal = false;
  	imu_thread = std::make_shared<std::thread>(&imu_thread_function);
  	imu_thread->join();
}
