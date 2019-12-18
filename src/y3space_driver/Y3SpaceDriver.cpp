#include <Y3SpaceDriver.h>


const std::string Y3SpaceDriver::logger = "[ Y3SpaceDriver ] ";
const std::string Y3SpaceDriver::MODE_ABSOLUTE = "absolute";
const std::string Y3SpaceDriver::MODE_RELATIVE = "relative";

Y3SpaceDriver::Y3SpaceDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string &port,
		int baudrate, int timeout, const std::string &mode, const std::string &frame):
    SerialInterface(port, baudrate, timeout),
    m_pnh(pnh),
    m_nh(nh),
    m_mode(mode),
    m_frame(frame),
	debug_(false)
{
	getParams();

    this->serialConnect();
    this->m_imuPub = this->m_nh.advertise<sensor_msgs::Imu>("/mavros/imu/data", 10);
    //this->m_tempPub = this->m_nh.advertise<std_msgs::Float64>("/imu/temp", 10);
    //this->m_rpyPub = this->m_nh.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy", 10);
}

void Y3SpaceDriver::getParams()
{
	m_pnh.param<int>("frequency", imu_frequency_, 400);
	m_pnh.param<bool>("debug", debug_, false);
}

Y3SpaceDriver::~Y3SpaceDriver()
{
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
    ROS_INFO_STREAM(this->logger << "Starting Auto Gyro Calibration...");
    this->serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  
    ros::Duration(5.0).sleep();
    ROS_INFO_STREAM(this->logger << "Proceeding");
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


std::string Y3SpaceDriver::getFrequencyMsg(int frequency)
{
	float period = 1000000.0/(float)frequency;
	std::string msg = ":82,"+std::to_string((int)period)+",0,0\n";
	std::cout << msg << std::endl;
	return msg;
}

//! Run the serial sync
void Y3SpaceDriver::run()
{
    std::vector<double> parsedVals;
    sensor_msgs::Imu imuMsg;
    geometry_msgs::Vector3Stamped imuRPY;
    std_msgs::Float64 tempMsg;


    this->startGyroCalibration();
    this->getSoftwareVersion();

    //AXIS DIRECTION
    this->serialWriteString(SET_AXIS_DIRECTIONS_FLU);
    this->serialReadLine();
    this->getAxisDirection();

    //Request for Particular Frequency
    this->serialWriteString(getFrequencyMsg(imu_frequency_).c_str());
    ROS_INFO("Get Streaming Frequency: %s", this->serialReadLine().c_str());


    //Set Streaming Slot
    this->serialWriteString(SET_STREAMING_SLOTS_AUTOMODALITY);
    this->serialWriteString(GET_STREAMING_SLOTS);
    ROS_INFO("GET_STREAMING_SLOTS POST CONFIGURATION: %s", this->serialReadLine().c_str());


    //Ask for timestamp
    this->serialWriteString(SET_TIME_STAMP_REQUEST);
    ROS_INFO("SET_TIME_STAMP_REQUEST: %s", this->serialReadLine().c_str());

    this->getCalibMode();
    this->getMIMode();
    /*if (m_mode == MODE_ABSOLUTE)
    {
        ROS_INFO_STREAM(this->logger << "Using absolute driver stream configuration");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    }
    else if (m_mode == MODE_RELATIVE)
    {
        ROS_INFO_STREAM(this->logger << "Using relative driver stream configuration");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    else
    {
        ROS_WARN_STREAM(this->logger << "Unknown driver mode set... Defaulting to relative");
        this->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }*/


    //this->serialWriteString(GET_HEADER_SETTING);
    //ROS_INFO("2. GET_HEADER_SETTING: %s", this->serialReadLine().c_str());
    //this->serialWriteString(GET_RAW_ACCEL_DATA);
    //ROS_INFO("GET_RAW_ACCEL_DATA: %s", this->serialReadLine().c_str());
    //this->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    //this->serialWriteString(TARE_WITH_CURRENT_QUATERNION);




    //this->serialWriteString(GET_STREAMING_SLOTS);
    //ROS_INFO("GET_STREAMING_SLOTS PRE CONFIGURATION: %s", this->serialReadLine().c_str());



    //this->serialWriteString(SET_EULER_ANGLE_DECOMP_ORDER_XYZ);
    //this->serialWriteString(GET_EULER_DECOMPOSTION_ORDER);
    //ROS_INFO("GET_EULER_DECOMPOSTION_ORDER: %s", this->serialReadLine().c_str());
    //ROS_INFO("GET_EULER_DECOMPOSTION_ORDER: %s", this->serialReadLine().c_str());


    this->flushSerial();


    this->serialWriteString(START_STREAMING);
    ROS_INFO_STREAM(this->logger << "Ready\n");

    //Complete HACK to make sure that the buffer is empty
    for(int k = 0; k < 10; k++)
    {
    	this->serialReadLine();
    }


    ros::Rate rate(1000);
    int line = -2;
    int expected_lines_ = 6;
    while(ros::ok())
    {
        while(this->available() > 0)
        {
            std::string buf = this->serialReadLine();
            /*if(line == 3)
            {
                ROS_INFO("BUFFER[%d]: %s",line, buf.c_str());

            }*/

            std::string parse;
            std::stringstream ss(buf);
            std::stringstream ss_tmp(buf);
            double i;
            //Wait for the beginning of the message
            if(line < 0)
            {
            	// Parse data from the line
            	while (ss_tmp >> i)
            	{
            		//time stamp is a large number which is at the beginning of the message
            		if(i / 1000.0 > 10)
            		{
            			//We found the time stamp
            			if(line == -1)
            			{
                			ROS_INFO("found the start: %f", i);
            			}
            			line++;
            			break;
            		}
            		if (ss_tmp.peek() == ',')
            			ss_tmp.ignore();
            	}
            }
            if(line > -1)
            {
            	line += 1;
            	// Parse data from the line
            	while (ss >> i)
            	{
            		parsedVals.push_back(i);
            		if (ss.peek() == ',')
            			ss.ignore();
            	}

            	// Should stop reading when line == number of tracked streams
            	if(line == expected_lines_)
            	{
            		int j = 0;
            		for_each(parsedVals.begin(), parsedVals.end(), [&](double x){if(x / 1000.0 > 10) {j++;}});
            		if(j > 1)
            		{
            			ROS_INFO("something went wrong. there are more than one timestamp in here. recalculating the offset...");
            			if(debug_)
            			{
            				ROS_INFO("parsedVals size: %d", (int)parsedVals.size());
            				int cnt = 0;
            				for_each(parsedVals.begin(), parsedVals.end(), [&](double x){ROS_INFO("Value[%d] = %f", cnt++, x);});
            				ROS_INFO("---------------------------------");
            			}
            			parsedVals.clear();
            			line=-1;
            			continue;
            		}


            		if(debug_)
            		{
            			ROS_INFO("parsedVals size: %d", (int)parsedVals.size());
            			int cnt = 0;
            			for_each(parsedVals.begin(), parsedVals.end(), [&](double x){ROS_INFO("Value[%d] = %f", cnt++, x);});
            			ROS_INFO("---------------------------------");
            		}


            		// Reset line tracker
            		line = 0;

            		/* for 6,38,39,41,44
            		 *	Array Structure:
            		 *	idx 0 --> timestamp
            		 *  idx 1-4 --> untared orientation as Quaternion 6
            		 * 	idx 5-7 --> untared orientation as Euler Angles 7
            		 * 	idx 8-10 --> corrected gyroscope vector 38
            		 * 	idx 11-13 --> corrected accelerometer vector 39
            		 * 	idx 14-16 --> corrected compass vector 40
            		 * 	idx 17-19 --> corrected linear acceleration 41
            		 */

            		// Prepare IMU message
            		imuMsg.header.stamp           = ros::Time::now();
            		imuMsg.header.frame_id        = "body_FLU";

            		imuMsg.orientation.x          = parsedVals[1];
            		imuMsg.orientation.y          = parsedVals[2];
            		imuMsg.orientation.z          = parsedVals[3];
            		imuMsg.orientation.w          = parsedVals[4];

            		imuMsg.angular_velocity.x     = parsedVals[8];
            		imuMsg.angular_velocity.y     = parsedVals[9];
            		imuMsg.angular_velocity.z     = parsedVals[10];

            		imuMsg.linear_acceleration.x  = 9.8*parsedVals[11];
            		imuMsg.linear_acceleration.y  = 9.8*parsedVals[12];
            		imuMsg.linear_acceleration.z  = 9.8*parsedVals[13];

            		// Prepare temperature messages
					tempMsg.data = parsedVals[14];

            		// Clear parsed values
					parsedVals.clear();

            		/*imuRPY.vector = getRPY(imuMsg.orientation);

            		imuRPY.vector.x = getDegree(imuRPY.vector.x);
            		imuRPY.vector.y = getDegree(imuRPY.vector.y);
            		imuRPY.vector.z = getDegree(imuRPY.vector.z);*/

					//imuRPY.vector.x = getDegree((double)parsedVals[5]);
					//imuRPY.vector.y = getDegree((double)parsedVals[6]);
					//imuRPY.vector.z = getDegree((double)parsedVals[7]);
					//geometry_msgs::Quaternion q = getQuaternion((double)parsedVals[5],(double)parsedVals[6],(double)parsedVals[7]);
					//std::cout << q << std::endl;

					imuMsg.orientation = toENU(imuMsg.orientation);

            		this->m_imuPub.publish(imuMsg);
            		//this->m_tempPub.publish(tempMsg);
            		//this->m_rpyPub.publish(imuRPY);
            	}
            }

        }

        // Throttle ROS at fixed Rate
        rate.sleep();
        ros::spinOnce();
    }
}

geometry_msgs::Vector3 Y3SpaceDriver::getRPY(geometry_msgs::Quaternion &q)
{
	geometry_msgs::Vector3 vect;

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

geometry_msgs::Quaternion Y3SpaceDriver::getQuaternion(double roll, double pitch, double yaw)
{
	tf::Quaternion q;
	q.setRPY(roll,pitch,yaw);
	geometry_msgs::Quaternion q_msg;
	quaternionTFToMsg(q, q_msg);
	return q_msg;
}

geometry_msgs::Quaternion Y3SpaceDriver::toENU(geometry_msgs::Quaternion q)
{
	tf::Quaternion q_FLU(q.x, q.y, q.z, q.w);
	tf::Quaternion q_TF(0.0, 0.0, 0.707, 0.707);
	tf::Quaternion q_ENU = q_TF * q_FLU;

	geometry_msgs::Quaternion q_MSG;
	quaternionTFToMsg(q_ENU, q_MSG);

	return q_MSG;
}

double Y3SpaceDriver::getDegree(double rad)
{
	return rad * 180.0 / M_PI;
}

