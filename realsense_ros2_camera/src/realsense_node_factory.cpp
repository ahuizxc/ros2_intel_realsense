// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/sr300_node.h"
#include "../include/rs415_node.h"
#include "../include/rs435_node.h"
#include <iostream>
#include <map>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory()
{
    RCUTILS_LOG_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
    RCUTILS_LOG_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

    signal(SIGINT, signalHandler);
    auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
    tryGetLogSeverity(severity);
    // if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
    //     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    rs2::log_to_console(severity);
}

void RealSenseNodeFactory::onInit()
{
    try{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		// auto nh = getNodeHandle();
		// auto privateNh = getPrivateNodeHandle();
		std::string serial_no("");
		this->get_parameter_or("serial_no", serial_no, std::string(""));
		
		std::string rosbag_filename("");
        this->get_parameter_or("rosbag_filename", rosbag_filename, std::string(""));
        if (!rosbag_filename.empty())
        {
			RCUTILS_LOG_INFO("publish topics from rosbag file: " << rosbag_filename.c_str());
			auto pipe = std::make_shared<rs2::pipeline>();
			rs2::config cfg;
			cfg.enable_device_from_file(rosbag_filename.c_str(), false);
			cfg.enable_all_streams();
			pipe->start(cfg); //File will be opened in read mode at this point
			auto _device = pipe->get_active_profile().get_device();
			_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(this.get(),_device, serial_no));
		}
        else
        {
			auto list = _ctx.query_devices();
			if (0 == list.size())
			{
				RCUTILS_LOG_ERROR("No RealSense devices were found! Terminating RealSense Node...");
				rclcpp::shutdown();
				exit(1);
			}

			bool found = false;
			for (auto&& dev : list)
			{
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				RCUTILS_LOG_DEBUG("Device with serial number " << sn << " was found.");
				if (serial_no.empty())
				{
					_device = dev;
					serial_no = sn;
					found = true;
					break;
				}
				else if (sn == serial_no)
				{
					_device = dev;
					found = true;
					break;
				}
			}

			if (!found)
			{
				RCUTILS_LOG_ERROR("The requested device with serial number %s" << serial_no << " is NOT found!");
				rclcpp::shutdown();
				exit(1);
			}

			_ctx.set_devices_changed_callback([this](rs2::event_information& info)
			{
				if (info.was_removed(_device))
				{
					RCUTILS_LOG_ERROR("The device has been disconnected! Terminating RealSense Node...");
					rclcpp::shutdown();
					exit(1);
				}
			});

			// TODO
			auto pid_str = _device.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
			uint16_t pid;
			std::stringstream ss;
			ss << std::hex << pid_str;
			ss >> pid;
			switch(pid)
			{
			case SR300_PID:
				_realSenseNode = std::unique_ptr<SR300Node>(new SR300Node(this.get(), _device, serial_no));
				break;
			case RS400_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS405_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS410_PID:
			case RS460_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS415_PID:
				_realSenseNode = std::unique_ptr<RS415Node>(new RS415Node(this.get(),_device, serial_no));
				break;
			case RS420_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS420_MM_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS430_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS430_MM_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS430_MM_RGB_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			case RS435_RGB_PID:
				_realSenseNode = std::unique_ptr<RS435Node>(new RS435Node(this.get(),_device, serial_no));
				break;
			case RS_USB2_PID:
				_realSenseNode = std::unique_ptr<BaseD400Node>(new BaseD400Node(this.get(),_device, serial_no));
				break;
			default:
				RCUTILS_LOG_ERROR("Unsupported device! Product ID: 0x%s", pid_str);
				rclcpp::shutdown();
				exit(1);
			}
		}
		assert(_realSenseNode);
        _realSenseNode->publishTopics();
		// _realSenseNode->registerDynamicReconfigCb();
	}
    catch(const std::exception& ex)
    {
        RCUTILS_LOG_ERROR("An exception has been thrown: %s", ex.what());
        throw;
    }
    catch(...)
    {
        RCUTILS_LOG_ERROR("Unknown exception has occured!");
        throw;
    }
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
    static const char* severity_var_name = "LRS_LOG_LEVEL";
    auto content = getenv(severity_var_name);

    if (content)
    {
        std::string content_str(content);
        std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

        for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
        {
            auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
            std::transform(current.begin(), current.end(), current.begin(), ::toupper);
            if (content_str == current)
            {
                severity = (rs2_log_severity)i;
                break;
            }
        }
    }
}
