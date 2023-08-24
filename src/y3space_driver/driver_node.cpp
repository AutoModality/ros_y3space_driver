#include <y3space_driver/Y3SpaceDriver.h>
#include <thread>
#include <memory>
#include <signal.h>
#include <am_utils/am_ros2_utility.h>
#include <iostream>
#include <libusb-1.0/libusb.h>
std::string NODE_NAME_ = "Y3SpaceDriver";


std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;
std::shared_ptr<Y3SpaceDriver> y3_space_driver = nullptr;




int reset_usb()
{
	libusb_context *ctx = nullptr;
    libusb_device_handle *handle = nullptr;

    // Initialize libusb
    if (libusb_init(&ctx) < 0) 
	{
        std::cerr << "Failed to initialize libusb" << std::endl;
        return 1;
    }

    // Find the USB device by Vendor ID and Product ID
    handle = libusb_open_device_with_vid_pid(ctx, 2476, 1010);
    if (!handle) {
        std::cerr << "Failed to find USB device" << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    // Reset the USB device
    if (libusb_reset_device(handle) < 0) {
        std::cerr << "Failed to reset USB device" << std::endl;
        libusb_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    // Clean up
    libusb_close(handle);
    libusb_exit(ctx);

    std::cout << "USB device reset successfully" << std::endl;

	return 0;
}

void signal_handler(int signal)
{	

	if(y3_space_driver)
	{
		ROS_INFO("Disconnecting from the serial port");
		y3_space_driver->serialDisConnect();
	}
	
	rclcpp::shutdown();


	reset_usb();

	exit(0);
}


int main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);


  	rclcpp::init(argc, argv);
  	
	am::Node::node = std::make_shared<am::AMLifeCycle>(NODE_NAME_);

	y3_space_driver = std::make_shared<Y3SpaceDriver>();

    rclcpp::spin(am::Node::node);

	rclcpp::shutdown();

	return 0;
}
