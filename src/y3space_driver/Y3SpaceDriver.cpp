#include <numeric>

#include <y3space_driver/Y3SpaceDriver.h>


const std::string Y3SpaceDriver::logger = "[ Y3SpaceDriver ] ";
const std::string Y3SpaceDriver::MODE_ABSOLUTE = "absolute";
const std::string Y3SpaceDriver::MODE_RELATIVE = "relative";

Y3SpaceDriver::Y3SpaceDriver(): SerialInterface()
{
	getParams();

    this->initSerial(m_port, m_baudrate, m_timeout);

    this->serialConnect();

    initDevice();

    ROS_INFO_STREAM(this->logger << "Ready\n");
    m_imuPub = am::Node::node->create_publisher<sensor_msgs::msg::Imu>(m_imu_topic, 100);
    //this->m_tempPub = this->m_nh.advertise<std_msgs::msg::Float64>("/imu/temp", 10);
    //this->m_rpyPub = this->m_nh.advertise<geometry_msgs::msg::Vector3Stamped>("/imu/rpy", 10);

    pub_timer_ = am::Node::node->create_wall_timer(am::toDuration(0.001), std::bind(&Y3SpaceDriver::pubTimerCB, this));
}


void Y3SpaceDriver::pubTimerCB()
{
    if(m_imuPub->get_subscription_count() == 0)
    {
        return;
    }

    sensor_msgs::msg::Imu imu_msg;
    if(getImuMessage(imu_msg) >= 0)
    {
        m_imuPub->publish(imu_msg);
        return;
    }
    //ROS_WARN("Could not retrieve an imu message");
    
}

int Y3SpaceDriver::getImuMessage(sensor_msgs::msg::Imu &imu_msg)
{
	//We assume the device is initialized
    /*if(tared_)
    {
        //Getting tared orientation as Quaternion
	    this->serialWriteString(GET_TARED_ORIENTATION_AS_QUATERNION_WITH_HEADER);
    }
    else
    {
        //Getting untared orientation as Quaternion
	    this->serialWriteString(GET_UNTARED_ORIENTATION_AS_QUATERNION_WITH_HEADER);
    }*/

    this->serialWriteString(GET_UNTARED_ORIENTATION_AS_QUATERNION_WITH_HEADER);
	
	std::string quaternion_msg = this->serialReadLine();
	std::vector<double>quaternion_arr = parseString<double>(quaternion_msg);	

	this->serialWriteString(GET_CORRECTED_GYRO_RATE);
	std::string gyro_msg = this->serialReadLine();
	std::vector<double>gyro_arr = parseString<double>(gyro_msg);


	this->serialWriteString(GET_CORRECTED_ACCELEROMETER_VECTOR);
	std::string accel_msg = this->serialReadLine();
	std::vector<double>accel_arr = parseString<double>(accel_msg);

	// Prepare IMU message
	rclcpp::Time sensor_time = getReadingTime(quaternion_arr[1]);
    if (sensor_time > am::ClockNow() + rclcpp::Duration(0,4000000)) {
        ROS_DEBUG_STREAM("Sensor time too far in future. Dropping.");
        return -1;
    }

    tf2::Quaternion q(quaternion_arr[2], quaternion_arr[3], quaternion_arr[4], quaternion_arr[5]);
    if(tared_)
    {
        static tf2::Quaternion q_init;
        static bool first_run = true;

        if(first_run)
        {
            q_init = tf2::Quaternion(quaternion_arr[2], quaternion_arr[3], quaternion_arr[4], quaternion_arr[5]);

            q_init = q_init.inverse();

            first_run = false;
        }

        q = q*q_init;
        q = q.normalize();
    }

	imu_msg.header.stamp           = sensor_time;
	
	imu_msg.header.frame_id        = m_frame;
	imu_msg.orientation.x          = q.x();
	imu_msg.orientation.y          = q.y();
	imu_msg.orientation.z          = q.z();
	imu_msg.orientation.w          = q.w();

	imu_msg.angular_velocity.x     = gyro_arr[0];
	imu_msg.angular_velocity.y     = gyro_arr[1];
	imu_msg.angular_velocity.z     = gyro_arr[2];

	imu_msg.linear_acceleration.x  = 9.8*accel_arr[0];
	imu_msg.linear_acceleration.y  = 9.8*accel_arr[1];
	imu_msg.linear_acceleration.z  = 9.8*accel_arr[2];

    if (debug_)
        ROS_WARN_STREAM_THROTTLE(1.0, "Publishing IMU. ts: " << rclcpp::Time(imu_msg.header.stamp).seconds());

	return 0;
}

void Y3SpaceDriver::getParams()
{
	
	am::getParam<bool>("debug", debug_, debug_);
	am::getParam<bool>("magnetometer_enabled", magnetometer_enabled_, magnetometer_enabled_);
    am::getParam<bool>("tare", tared_, tared_);
	am::getParam<double>("timestamp_offset", timestamp_offset_, 0.012);
    am::getParam<std::string>("port", m_port, m_port);
    am::getParam<std::string>("mode", m_mode, m_mode);
    am::getParam<std::string>("frame", m_frame, m_frame);
    am::getParam<std::string>("imu_topic", m_imu_topic, m_imu_topic);
    am::getParam<int>("baudrate", m_baudrate, m_baudrate);
    am::getParam<int>("timeout", m_timeout, m_timeout);
    //am::getParam<int>("frequency", m_frequency, m_frequency);
    am::getParam<int>("frequency", imu_frequency_, imu_frequency_);

    //Create a non ros thread
    long time_out = 1000000/imu_frequency_;

    ROS_INFO("debug: %s", (debug_?"True":"False"));
    ROS_INFO("magnetometer_enabled_: %s", (magnetometer_enabled_?"True":"False"));
    ROS_INFO("tared: %s", (tared_?"True":"False"));
    ROS_INFO("timestamp_offset: %f",timestamp_offset_);
    ROS_INFO("m_baudrate: %d",m_baudrate);
    ROS_INFO("m_timeout: %d",m_timeout);
    ROS_INFO("imu_frequency_: %d",imu_frequency_);
    ROS_INFO("m_port: %s",m_port.c_str());
    ROS_INFO("m_mode: %s",m_mode.c_str());
    ROS_INFO("frame: %s",m_frame.c_str());
    ROS_INFO("m_imu_topic: %s",m_imu_topic.c_str());
}

rclcpp::Time Y3SpaceDriver::getYostRosTime(long sensor_time)
{
	rclcpp::Time result(sensor_time % 1000000, sensor_time / 1000000);

	return result;
}

Y3SpaceDriver::~Y3SpaceDriver()
{
	//resetSensor();

	serialDisConnect();
}

void Y3SpaceDriver::restoreFactorySettings()
{
    this->serialWriteString(RESTORE_FACTORY_SETTINGS);
}

const std::string Y3SpaceDriver::getSoftwareVersion()
{
    this->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    const std::string buf = this->serialReadLine();
    ROS_INFO_STREAM(this->logger << "Software version: " << buf);
    return buf;
}

const std::string Y3SpaceDriver::getAxisDirection()
{
    this->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if ( buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if ( buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if( buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if( buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y: Left, Z: Up";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "Axis Direction: " << ret);
    return ret;
}

void Y3SpaceDriver::startGyroCalibration(void)
{
    ROS_INFO("Starting Auto Gyro Calibration...");
    this->serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  
    //rclcpp::Duration(5.0).sleep();
    ROS_INFO("Gyro calibration is completed");
}

void Y3SpaceDriver::setMIMode(bool on)
{
    if(on)
    {
        this->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        this->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

void Y3SpaceDriver::tareDevice()
{
    this->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
}

void Y3SpaceDriver::setMagnetometer(bool on)
{
    if(on)
    {
        this->serialWriteString(SET_MAGNETOMETER_ENABLED);
    }
    else
    {
        this->serialWriteString(SET_MAGNETOMETER_DISABLED);
    }
}

/*
 * **********************************************
 * Device Setup
 * ..............................................
 */
void Y3SpaceDriver::initDevice()
{
	this->startGyroCalibration();
	this->getSoftwareVersion();
	this->setAxisDirection();
	this->setHeader();
	this->setMagnetometer(magnetometer_enabled_);
    /*if(tared_)
    {
        //taring the device will reset the heading to the EAST using the Magnetometer 
        this->tareDevice();
    }*/
	this->setFilterMode();
	this->serialWriteString(SET_REFERENCE_VECTOR_CONTINUOUS);

	this->resetTimeStamp();

	sleep(2);

	this->serialWriteString(GET_FILTER_MODE);
	ROS_INFO("GET_FILTER_MODE: %s", this->serialReadLine().c_str());
	// this->setFrequency();
	// this->setStreamingSlots();
	this->getAxisDirection();
	this->getCalibMode();
	this->getMIMode();
	this->getMagnetometerEnabled();
	this->flushSerial();

	this->syncTimeStamp();
}

void Y3SpaceDriver::resetTimeStamp()
{
	this->serialWriteString(UPDATE_CURRENT_TIMESTAMP);
	ros_time_start_ = am::ClockNow();
	ROS_DEBUG_STREAM("RESETTING SENSOR TIME STAMP at " << ros_time_start_.seconds());
}

void Y3SpaceDriver::syncTimeStamp()
{
	std::vector<double> latency;
	std::vector<double> sensor_time;
	std::string sensor_msg;
	std::vector<double>sensor_msg_arr;
	rclcpp::Time sensor_time_ros;

	// Zero Yost time
	this->resetTimeStamp();

	// Calculate message latency
	for (int i = 0; i < 5; ++i)
	{
		this->serialWriteString(GET_UNTARED_ORIENTATION_AS_QUATERNION_WITH_HEADER);
		sensor_msg = this->serialReadLine();
		sensor_msg_arr = parseString<double>(sensor_msg);

		sensor_time.push_back(sensor_msg_arr[1] / 1000000 );

		latency.push_back ((am::ClockNow().seconds() - ros_time_start_.seconds() - sensor_time.back()) /2 );
	}

	double average = std::accumulate( latency.begin(), latency.end(), 0.0) / latency.size();
    if (debug_)
    	ROS_INFO_STREAM(this->logger << "Average Yost IMU message latency is " << average);

	msg_latency_ = average;
}

void Y3SpaceDriver::setFilterMode()
{
	this->serialWriteString(SET_FILTER_MODE_KALMAN);
}

const std::string Y3SpaceDriver::getCalibMode()
{
    this->serialWriteString(GET_CALIB_MODE);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Bias";
        }
        else if ( buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "Calibration Mode: " << ret);
    return ret;
}

const std::string Y3SpaceDriver::getMIMode()
{
    this->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "MI Mode: " << ret);
    return ret;
}

const std::string Y3SpaceDriver::getMagnetometerEnabled()
{
    this->serialWriteString(GET_MAGNETOMETER_ENABLED);

    const std::string buf = this->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            ROS_WARN_STREAM(this->logger << "Buffer indicates: " + buf);
            return "Unknown";
        }
    }();

    ROS_INFO_STREAM(this->logger << "Magnetometer enabled state: " << ret);
    return ret;
}


std::string Y3SpaceDriver::getFrequencyMsg(int frequency)
{
	float period = 1000000.0/(float)frequency;
	std::string msg = ":82,"+std::to_string((int)period)+",0,0\n";
	std::cout << msg << std::endl;
	return msg;
}

void Y3SpaceDriver::setFrequency()
{
	this->serialWriteString(getFrequencyMsg(imu_frequency_).c_str());
}

void Y3SpaceDriver::setHeader()
{
	// Ask for timestamp and success byte
	this->serialWriteString(SET_HEADER_TS_SUCCESS);
	// Returns no data
}

void Y3SpaceDriver::setStreamingSlots()
{
	//Set Streaming Slot
	this->serialWriteString(SET_STREAMING_SLOTS_AUTOMODALITY);
	this->serialWriteString(GET_STREAMING_SLOTS);
	ROS_INFO("Y3SpaceDriver: GET_STREAMING_SLOTS POST CONFIGURATION: %s", this->serialReadLine().c_str());
}

void Y3SpaceDriver::setAxisDirection()
{
	//AXIS DIRECTION
	this->serialWriteString(SET_AXIS_DIRECTIONS_FLU);

}

rclcpp::Time Y3SpaceDriver::toRosTime(double sensor_time)
{
	rclcpp::Time res(sensor_time / 1000000, ((long) sensor_time % 1000000) * 1000);

	return res;
}

/* Returns Yost sensor time converted to ROS time
*  @param sensor_time internal clock time of sensor in microseconds
*/
rclcpp::Time Y3SpaceDriver::getReadingTime(uint64_t sensor_time)
{
	rclcpp::Duration ros_sensor_time(0, (sensor_time*1000));

	if (ros_sensor_time.seconds() > 3.0)
		syncTimeStamp();

	// Add in 2x msg_latency to account for two messages -- initial sync message and current message
	rclcpp::Time result = ros_time_start_ + ros_sensor_time;// + rclcpp::Duration(msg_latency_ * 2);

	if (debug_) {
        double delay = am::ClockNow().seconds()-result.seconds();
        if (delay < -0.004)
        {
            ROS_ERROR_STREAM("time delay negative! " << delay);
        }
		ROS_INFO_THROTTLE(1,"\tros_time_start: %f", ros_time_start_.seconds());
		ROS_INFO_THROTTLE(1,"\tRaw Sensor Time: %f, result: %f, msg latency: %f\n\t\tage of data: %f sec",
				ros_sensor_time.seconds(), result.seconds(), msg_latency_, delay);
	}
	
	return result;
}

geometry_msgs::msg::Vector3 Y3SpaceDriver::getRPY(geometry_msgs::msg::Quaternion &q)
{
	geometry_msgs::msg::Vector3 vect;

	/*tf2::Quaternion tf_q;
	tf2::convert(q,tf_q);
	tf2::Matrix3x3 m(tf_q);
	m.getRPY(vect.x, vect.y, vect.z);
	return vect;*/

	tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
	tf2::Matrix3x3 m(tfq);
	m.getRPY(vect.x, vect.y, vect.z);
	return vect;
}

geometry_msgs::msg::Quaternion Y3SpaceDriver::getQuaternion(double roll, double pitch, double yaw)
{
	tf2::Quaternion q;
	q.setRPY(roll,pitch,yaw);
	return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion Y3SpaceDriver::toENU(geometry_msgs::msg::Quaternion q)
{
	tf2::Quaternion q_FLU(q.x, q.y, q.z, q.w);
	tf2::Quaternion q_TF(0.0, 0.0, 0.707, 0.707);
	tf2::Quaternion q_ENU = q_TF * q_FLU;

	return tf2::toMsg(q_ENU);
}

double Y3SpaceDriver::getDegree(double rad)
{
	return rad * 180.0 / M_PI;
}

