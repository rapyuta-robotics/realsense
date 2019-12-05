// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

bool match_accel_vectors(const rs2_vector& a, const rs2_vector& b, float error = 1.0f)
{
    return (abs(a.x - b.x) <= error && abs(a.y - b.y) <= error && abs(a.z - b.z) <= error);
}

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory()
{
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

	auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
	tryGetLogSeverity(severity);
	if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	rs2::log_to_console(severity);

	_tf_listener = std::make_unique<tf2_ros::TransformListener>(_tf_buffer);
}

void RealSenseNodeFactory::closeDevice()
{
    for(rs2::sensor sensor : _device.query_sensors())
	{
		sensor.stop();
		sensor.close();
	}
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
	closeDevice();
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
	if (!_device)
	{
		if (0 == list.size())
		{
			ROS_WARN("No RealSense devices were found!");
		}
		else
		{
			bool found = false;

			if (_accel_orientation.empty()) // generate accel orientation vector from camera quaternion
			{
				try {
					geometry_msgs::TransformStamped transformStamped = 
									_tf_buffer.lookupTransform(_tf_camera_link_name, _tf_reference_link_name, ros::Time(0));

					tf2::Quaternion q_gravity_base, q_camera_rot, q_gravity_camera;
					// universal gravity vector
					q_gravity_base.setValue(0.0, -9.81, 0.0);
					tf2::convert(transformStamped.transform.rotation , q_camera_rot);
					// Calculate the new orientation
					q_gravity_camera = q_camera_rot.inverse() * q_gravity_base * q_camera_rot;

					std::stringstream ss;
					ss << q_gravity_camera.x() << " " << q_gravity_camera.y() << " " << q_gravity_camera.z();
					_accel_orientation = ss.str();

					ROS_DEBUG_STREAM("Computing from tf2 quaternion for " << _camera_name << " rotation: " << transformStamped.transform.rotation
									<< ", accel: " << ss.str() << " " << q_gravity_camera.w());
				} catch (tf2::TransformException e) {
					ROS_ERROR_STREAM(e.what());
					return;
				}
			}

			// correctly decode and fill rs2_vector for expected median accelerometer orientation vector
			rs2_vector accel_orientation_vec;
			sscanf (_accel_orientation.c_str(), "%f%f%f", &accel_orientation_vec.x, &accel_orientation_vec.y, &accel_orientation_vec.z);
			ROS_DEBUG_STREAM("Looking for " << _camera_name << " device with orientation " << accel_orientation_vec);

			for (auto&& dev : list)
			{
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				ROS_DEBUG_STREAM("Device with serial number " << sn << " was found.");

				if (_serial_no.empty() && !_accel_orientation.empty()) // auto determine correct realsense orientation
				{
					rs2::pipeline pipe;
					rs2::config cfg;
					cfg.enable_device(sn);
					cfg.enable_stream(RS2_STREAM_ACCEL);

					// attempt to create pipe stream for upto max attempts
					const int MAX_ATTEMPTS = 5;
					bool success = false;
					int no_attempts = 0;
					while (no_attempts < MAX_ATTEMPTS)
					{
						try
						{
							pipe.start(cfg);
							success = true;
							break;
						} catch (...)
						{
							no_attempts ++;
							ROS_DEBUG_STREAM("Pipe failed to create on device serial no " << sn << ". Device busy. Waiting..." << no_attempts);
							ros::Duration(2).sleep();
						}
					}

					if (!success)
					{
						// all max attempts exhausted for the current device
						// this is not serious and is expected and will happen for atleast one realsense ros node.
						// when the first node successfully inits one sensor the second node will fail for that sensor
						// if this fails for both nodes that means atleast one sensor is unavailable
						// and that will anyways throw error (!found) further down the code
						// therefore this is INFO only
						ROS_INFO_STREAM("Pipe failed to create on device serial no " << sn << " after " << no_attempts << " attempts");
						continue;
					}

					// attempt to read accel data frame for upto max attempts
					no_attempts = 0;
					while (no_attempts < MAX_ATTEMPTS)
					{
						rs2::frameset frameset = pipe.wait_for_frames();
						no_attempts ++;

						// fetch accelerometer orientation data from pipe stream
						rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL);
						if (accel_frame)
						{
							rs2_vector accel_sample = accel_frame.get_motion_data();
							ROS_DEBUG_STREAM("Received orientation " << accel_sample << " from device serial no " << sn);
							if (match_accel_vectors(accel_sample, accel_orientation_vec, 2.0f))
							{
								_device = dev;
								_serial_no = sn;
								found = true;
								ROS_INFO_STREAM("Device serial no " << sn << " with orientation " << accel_sample << " assigned to " << _camera_name);
								break;
							}
						}
					}

					if (success) {
						pipe.stop(); // stop pipe only if pipe start succeeded earlier
					}

					if (found) // found one matching device in correct orientation, no need to look for more
					{
						break;
					}
				} else {
					_device = dev;
					_serial_no = sn;
					found = true;
					break;
				}
			}
			if (!found)
			{
				if (!_serial_no.empty())
				{
					// T265 could be caught by another node.
					ROS_ERROR_STREAM("The requested device with serial number " << _serial_no << " is NOT found. Will Try again.");
				} else if (!_accel_orientation.empty())
				{
					// error: wrong orientation after 5 attempts
					ROS_ERROR_STREAM("The requested device with orientation " << _accel_orientation << " is NOT found. Will Try again.");
				} else
				{
					// could not assign any available device
					ROS_ERROR_STREAM("No available devices found.");
				}
			}
		}
	}

	bool remove_tm2_handle(_device && RS_T265_PID != std::stoi(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	if (_device && _initial_reset)
	{
		_initial_reset = false;
		try
		{
			ROS_INFO("Resetting device...");
			_device.hardware_reset();
			_device = rs2::device();
			
		}
		catch(const std::exception& ex)
		{
			ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
		}
	}
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
	if (info.was_removed(_device))
	{
		ROS_ERROR("The device has been disconnected!");
		_realSenseNode.reset(nullptr);
		_device = rs2::device();
	}
	if (!_device)
	{
		rs2::device_list new_devices = info.get_new_devices();
		if (new_devices.size() > 0)
		{
			ROS_INFO("Checking new devices...");
			getDevice(new_devices);
			if (_device)
			{
				StartDevice();
			}
		}
	}
}

void RealSenseNodeFactory::onInit()
{
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();

		/* change: sumandeepb: determine correct serial no for realsense_top and realsense_bottom */
		privateNh.param("camera", _camera_name, std::string(""));
		privateNh.param("reference_frame_id", _tf_reference_link_name, std::string(""));
		// base_frame_id selected as the accel_frame_id is unavailable pre initialization
		// both have same orientation (hence identical rotation quaternion, which is what we use)
		privateNh.param("base_frame_id", _tf_camera_link_name, std::string(""));
		// optional argument to force selection of specific sensor to RS ros node by serial no
		privateNh.param("serial_no", _serial_no, std::string(""));
		// optional argument to overide TF URDF specs and select according to manually specified orentation
		// to be used for testing experimental arrangements
		privateNh.param("accel_orientation", _accel_orientation, std::string(""));

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));
		if (!rosbag_filename.empty())
		{
			{
				ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
				auto pipe = std::make_shared<rs2::pipeline>();
				rs2::config cfg;
				cfg.enable_device_from_file(rosbag_filename.c_str(), false);
				cfg.enable_all_streams();
				pipe->start(cfg); //File will be opened in read mode at this point
				_device = pipe->get_active_profile().get_device();
				_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
				_realSenseNode->publishTopics();
				_realSenseNode->registerDynamicReconfigCb(nh);
			}
			if (_device)
			{
				StartDevice();
			}
		}
		else
		{
			privateNh.param("initial_reset", _initial_reset, false);

			_query_thread = std::thread([=]()
						{
							std::chrono::milliseconds timespan(6000);
							while (!_device)
							{
								// _ctx.init_tracking_module(); // Unavailable function.
								getDevice(_ctx.query_devices());
								if (_device)
								{
									std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
									_ctx.set_devices_changed_callback(change_device_callback_function);
									StartDevice();
								}
								else
								{
									std::this_thread::sleep_for(timespan);
								}
							}
						});
		}
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		throw;
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		throw;
	}
}

void RealSenseNodeFactory::StartDevice()
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();
	// TODO
	std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch(pid)
	{
	case SR300_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS_USB2_PID:
		_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	case RS_T265_PID:
		_realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
		break;
	default:
		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
		ros::shutdown();
		exit(1);
	}
	assert(_realSenseNode);
	_realSenseNode->publishTopics();
	_realSenseNode->registerDynamicReconfigCb(nh);
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
