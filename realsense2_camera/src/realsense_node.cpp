#include <realsense2_camera/realsense_node.h>
#include <realsense2_camera/param_manager.h>
#include <boost/interprocess/sync/named_mutex.hpp>

using namespace realsense2_camera;

constexpr stream_index_pair RealSenseNode::COLOR;
constexpr stream_index_pair RealSenseNode::DEPTH;
constexpr stream_index_pair RealSenseNode::INFRA1;
constexpr stream_index_pair RealSenseNode::INFRA2;
constexpr stream_index_pair RealSenseNode::FISHEYE;
constexpr stream_index_pair RealSenseNode::GYRO;
constexpr stream_index_pair RealSenseNode::ACCEL;

// Mutex used to prevent asynchronous_syncer (runs in a separate thread)
// from accessing node's data in _frame_callback
// when the node is resetting during resetNode() call
std::mutex syncer_mutex;

std::string RealSenseNode::getNamespaceStr()
{
    auto ns = ros::this_node::getNamespace();
    ns.erase(std::remove(ns.begin(), ns.end(), '/'), ns.end());
    return ns;
}

RealSenseNode::RealSenseNode(const ros::NodeHandle &nodeHandle,
                                     const ros::NodeHandle &privateNodeHandle) :
    _node_handle(nodeHandle),
    _pnh(privateNodeHandle),
    _json_file_path(""),
    _base_frame_id(""),
    _intialize_time_base(false),
    _namespace(getNamespaceStr())
{
    // Types for depth stream
    _is_frame_arrived[DEPTH] = false;
    _format[DEPTH] = RS2_FORMAT_Z16;   // libRS type
    _image_format[DEPTH] = CV_16UC1;    // CVBridge type
    _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    _unit_step_size[DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    _stream_name[DEPTH] = "depth";
    _depth_aligned_encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Infrared stream - Left
    _is_frame_arrived[INFRA1] = false;
    _format[INFRA1] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA1] = CV_8UC1;    // CVBridge type
    _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA1] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA1] = "infra1";
    _depth_aligned_encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Infrared stream - Right
    _is_frame_arrived[INFRA2] = false;
    _format[INFRA2] = RS2_FORMAT_Y8;   // libRS type
    _image_format[INFRA2] = CV_8UC1;    // CVBridge type
    _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[INFRA2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[INFRA2] = "infra2";
    _depth_aligned_encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for color stream
    _is_frame_arrived[COLOR] = false;
    _format[COLOR] = RS2_FORMAT_RGB8;   // libRS type
    _image_format[COLOR] = CV_8UC3;    // CVBridge type
    _encoding[COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
    _unit_step_size[COLOR] = 3; // sensor_msgs::ImagePtr row step size
    _stream_name[COLOR] = "color";
    _depth_aligned_encoding[COLOR] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for fisheye stream
    _is_frame_arrived[FISHEYE] = false;
    _format[FISHEYE] = RS2_FORMAT_RAW8;   // libRS type
    _image_format[FISHEYE] = CV_8UC1;    // CVBridge type
    _encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[FISHEYE] = "fisheye";
    _depth_aligned_encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_16UC1;

    // Types for Motion-Module streams
    _is_frame_arrived[GYRO] = false;
    _format[GYRO] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[GYRO] = CV_8UC1;    // CVBridge type
    _encoding[GYRO] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[GYRO] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[GYRO] = "gyro";

    _is_frame_arrived[ACCEL] = false;
    _format[ACCEL] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
    _image_format[ACCEL] = CV_8UC1;    // CVBridge type
    _encoding[ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    _unit_step_size[ACCEL] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
    _stream_name[ACCEL] = "accel";

    getParameters();
    getDevice();

    // TODO: Improve the pipeline to accept decimation filter
    // TODO: Provide disparity map if requested
    filters.emplace_back("Depth_to_Disparity", depth_to_disparity);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    filters.emplace_back("Disparity_to_Depth", disparity_to_depth);

    filters[0].is_enabled = false;
    filters[1].is_enabled = false;
    filters[2].is_enabled = false;
    filters[3].is_enabled = false;

    _prev_camera_time_stamp = 0;

    setHealthTimers();

    if (_dev)
    {
      createParamsManager();
      publishTopics();
    }
}

void RealSenseNode::createParamsManager() {
    auto pid_str = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    uint16_t pid;
    std::stringstream ss;
    ss << std::hex << pid_str;
    ss >> pid;
    try
    {
      _params = param_makers.at(pid)();
    }
    catch (...)
    {
       ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
       ros::Duration(20).sleep();
       resetNode();
    }
}

void RealSenseNode::resetNode() {
    ROS_WARN("Resetting node: %s", _namespace.c_str());

    ROS_INFO("Stop sensors: %s", _namespace.c_str());
    for (auto& streams : IMAGE_STREAMS) {
        auto& sens = _sensors[streams.front()];
        if (sens) {
           try {
                sens.stop();
                sens.close();
            } catch (const rs2::error& e) {
                ROS_ERROR("Unable to stop sensor: %s", _namespace.c_str());
            }
        }
    }

    std::lock_guard<std::mutex> syncer_mutex_guard(syncer_mutex);

    auto nh = _node_handle;
    auto pnh = _pnh;

    ROS_INFO("Destroy node: %s", _namespace.c_str());
    this->~RealSenseNode();

    ROS_INFO("Construct new node: %s", _namespace.c_str());
    new (this) RealSenseNode(nh, pnh);

    ROS_INFO("Node reset finished: %s", _namespace.c_str());
}

void RealSenseNode::getDevice() {

    if (!_rosbag_filename.empty())
    {
        ROS_INFO_STREAM("publish topics from rosbag file: " << _rosbag_filename.c_str());
        auto pipe = std::make_shared<rs2::pipeline>();
        rs2::config cfg;
        cfg.enable_device_from_file(_rosbag_filename.c_str(), false);
        cfg.enable_all_streams();
        pipe->start(cfg); //File will be opened in read mode at this point
        auto _device = pipe->get_active_profile().get_device();
        //_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, serial_no));
    }
    else
    {
        namespace bi = boost::interprocess;
        bi::named_mutex usb_mutex{bi::open_or_create, "usb_mutex"};
        usb_mutex.lock();
        auto list = _ctx.query_devices();
        usb_mutex.unlock();
        if (0 == list.size())
        {
            ROS_ERROR("No RealSense devices were found!.");
            return;
        }

        bool found = false;
        for (auto&& dev : list)
        {
            auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            ROS_DEBUG_STREAM("Device with serial number " << sn << " was found.");
            if (_serial_no.empty())
            {
                _dev = dev;
                _serial_no = sn;
                found = true;
                break;
            }
            else if (sn == _serial_no)
            {
                _dev = dev;
                found = true;
                break;
            }
        }

        if (!found)
        {
            ROS_ERROR_STREAM("The requested device with serial number " << _serial_no << " is NOT found!");
            return;
        }

        _ctx.set_devices_changed_callback([this](rs2::event_information& info)
        {
            if (info.was_removed(_dev))
            {
                ROS_ERROR("The device has been disconnected!");
            }
        });
    }
}

void RealSenseNode::publishTopics()
{
    setupDevice();
    setupPublishers();
    setupServices();
    setupStreams();
    publishStaticTransforms();
    if (_params)
    {
      _params->registerDynamicReconfigCb(this);
    }
    ROS_INFO_STREAM("RealSense Node Is Up!");
}


bool RealSenseNode::enableStreams(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    res.success = true;
    for (auto& streams : IMAGE_STREAMS)
    {
        auto& sens = _sensors[streams.front()];
        if (sens)
        {
            if (req.data)
            {
                try
                {
                    if (_sync_frames)
                    {
                        sens.start(_syncer);
                    }
                    else
                    {
                        sens.start(_frame_callback);
                    }
                }
                catch (const rs2::error& e)
                {
                    res.message += std::string("Failed to start stream:  ") + e.what() + '\n';
                    res.success = false;
                }
                depth_callback_timer_.start();
            }
            else
            {
                try
                {
                    sens.stop();
                }
                catch (const rs2::error& e)
                {
                    res.message += std::string("Failed to stop stream:  ") + e.what() + '\n';
                    res.success = false;
                }
                depth_callback_timer_.stop();
            }
        }
    }

    return true;
}

void RealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    _pnh.param("align_depth", _align_depth, ALIGN_DEPTH);
    _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
    _pnh.param("enable_sync", _sync_frames, SYNC_FRAMES);
    _pnh.param("enable_ros_time", _use_ros_time, USE_ROS_TIME);
    if (_pointcloud || _align_depth)
        _sync_frames = true;
    if (_sync_frames)
      _use_ros_time = true;

    _pnh.param("json_file_path", _json_file_path, std::string(""));

    _pnh.param("depth_width", _width[DEPTH], DEPTH_WIDTH);
    _pnh.param("depth_height", _height[DEPTH], DEPTH_HEIGHT);
    _pnh.param("depth_fps", _fps[DEPTH], DEPTH_FPS);

    _pnh.param("enable_depth", _enable[DEPTH], ENABLE_DEPTH);
    _aligned_depth_images[DEPTH].resize(_width[DEPTH] * _height[DEPTH] * _unit_step_size[DEPTH]);

    _pnh.param("infra1_width", _width[INFRA1], INFRA1_WIDTH);
    _pnh.param("infra1_height", _height[INFRA1], INFRA1_HEIGHT);
    _pnh.param("infra1_fps", _fps[INFRA1], INFRA1_FPS);
    _pnh.param("enable_infra1", _enable[INFRA1], ENABLE_INFRA1);
    _aligned_depth_images[INFRA1].resize(_width[DEPTH] * _height[DEPTH] * _unit_step_size[DEPTH]);

    _pnh.param("infra2_width", _width[INFRA2], INFRA2_WIDTH);
    _pnh.param("infra2_height", _height[INFRA2], INFRA2_HEIGHT);
    _pnh.param("infra2_fps", _fps[INFRA2], INFRA2_FPS);
    _pnh.param("enable_infra2", _enable[INFRA2], ENABLE_INFRA2);
    _aligned_depth_images[INFRA2].resize(_width[DEPTH] * _height[DEPTH] * _unit_step_size[DEPTH]);

    _pnh.param("color_width", _width[COLOR], COLOR_WIDTH);
    _pnh.param("color_height", _height[COLOR], COLOR_HEIGHT);
    _pnh.param("color_fps", _fps[COLOR], COLOR_FPS);
    _pnh.param("enable_color", _enable[COLOR], ENABLE_COLOR);
    _aligned_depth_images[COLOR].resize(_width[DEPTH] * _height[DEPTH] * _unit_step_size[DEPTH]);

    _pnh.param("fisheye_width", _width[FISHEYE], FISHEYE_WIDTH);
    _pnh.param("fisheye_height", _height[FISHEYE], FISHEYE_HEIGHT);
    _pnh.param("fisheye_fps", _fps[FISHEYE], FISHEYE_FPS);
    _pnh.param("enable_fisheye", _enable[FISHEYE], ENABLE_FISHEYE);
    _aligned_depth_images[FISHEYE].resize(_width[DEPTH] * _height[DEPTH] * _unit_step_size[DEPTH]);

    _pnh.param("gyro_fps", _fps[GYRO], GYRO_FPS);
    _pnh.param("accel_fps", _fps[ACCEL], ACCEL_FPS);
    _pnh.param("enable_imu", _enable[GYRO], ENABLE_IMU);
    _pnh.param("enable_imu", _enable[ACCEL], ENABLE_IMU);

    _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
    _pnh.param("depth_frame_id", _frame_id[DEPTH], DEFAULT_DEPTH_FRAME_ID);
    _pnh.param("infra1_frame_id", _frame_id[INFRA1], DEFAULT_INFRA1_FRAME_ID);
    _pnh.param("infra2_frame_id", _frame_id[INFRA2], DEFAULT_INFRA2_FRAME_ID);
    _pnh.param("color_frame_id", _frame_id[COLOR], DEFAULT_COLOR_FRAME_ID);
    _pnh.param("fisheye_frame_id", _frame_id[FISHEYE], DEFAULT_FISHEYE_FRAME_ID);
    _pnh.param("imu_gyro_frame_id", _frame_id[GYRO], DEFAULT_IMU_FRAME_ID);
    _pnh.param("imu_accel_frame_id", _frame_id[ACCEL], DEFAULT_IMU_FRAME_ID);

    _pnh.param("depth_optical_frame_id", _optical_frame_id[DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    _pnh.param("infra1_optical_frame_id", _optical_frame_id[INFRA1], DEFAULT_INFRA1_OPTICAL_FRAME_ID);
    _pnh.param("infra2_optical_frame_id", _optical_frame_id[INFRA2], DEFAULT_INFRA2_OPTICAL_FRAME_ID);
    _pnh.param("color_optical_frame_id", _optical_frame_id[COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    _pnh.param("fisheye_optical_frame_id", _optical_frame_id[FISHEYE], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
    _pnh.param("gyro_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_GYRO_OPTICAL_FRAME_ID);
    _pnh.param("accel_optical_frame_id", _optical_frame_id[ACCEL], DEFAULT_ACCEL_OPTICAL_FRAME_ID);

    _pnh.param("aligned_depth_to_color_frame_id",   _depth_aligned_frame_id[COLOR],   DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID);
    _pnh.param("aligned_depth_to_infra1_frame_id",  _depth_aligned_frame_id[INFRA1],  DEFAULT_ALIGNED_DEPTH_TO_INFRA1_FRAME_ID);
    _pnh.param("aligned_depth_to_infra2_frame_id",  _depth_aligned_frame_id[INFRA2],  DEFAULT_ALIGNED_DEPTH_TO_INFRA2_FRAME_ID);
    _pnh.param("aligned_depth_to_fisheye_frame_id", _depth_aligned_frame_id[FISHEYE], DEFAULT_ALIGNED_DEPTH_TO_FISHEYE_FRAME_ID);

    _pnh.param("publish_every_nth_frameset", _publish_every_nth_frameset, PUBLISH_EVERY_NTH_FRAMESET);

    _pnh.param("rosbag_filename", _rosbag_filename, _rosbag_filename);

    double depth_callback_timeout = 30; // seconds
    _pnh.param("depth_callback_timeout", depth_callback_timeout, depth_callback_timeout);
    depth_callback_timeout_ = ros::Duration(depth_callback_timeout);

    _pnh.param("use_depth_rate_monitor", _use_depth_rate_monitor, DEFAULT_USE_DEPTH_RATE_MONITOR);
    _depth_rate_monitor_num_of_new_frames = 0;
    _depth_rate_monitor_num_of_new_aligned_frames = 0;
    _depth_rate_monitor_num_of_new_color_frames = 0;
    _pnh.param("depth_rate_monitor_initial_cycles_number", _depth_rate_monitor_initial_cycles_number, DEFAULT_DEPTH_RATE_MONITOR_INITIAL_CYCLES_NUMBER);
    _depth_rate_monitor_initial_cycles_counter = 0;
    _pnh.param("depth_rate_monitor_min_frames_number_per_cycle", _depth_rate_monitor_min_frames_number_per_cycle, DEFAULT_DEPTH_RATE_MONITOR_MIN_FRAMES_NUMBER_PER_CYCLE);
    _pnh.param("depth_rate_monitor_consequent_low_rate_frames_limit", _depth_rate_monitor_consequent_low_rate_frames_limit, DEFAULT_DEPTH_RATE_MONITOR_CONSEQUENT_LOW_RATE_FRAMES_LIMIT);
    _depth_rate_monitor_consequent_low_rate_frames_counter = 0;
    _depth_rate_monitor_consequent_low_rate_aligned_frames_counter = 0;
    _depth_rate_monitor_consequent_low_rate_color_frames_counter = 0;

    _pnh.param("serial_no", _serial_no, _serial_no);

    _publish_skip_counter = 0;
}

void RealSenseNode::setupDevice()
{
    ROS_INFO("setupDevice...");
    try{
        if (!_json_file_path.empty())
        {
            if (_dev.is<rs400::advanced_mode>())
            {
                std::stringstream ss;
                std::ifstream in(_json_file_path);
                if (in.is_open())
                {
                    ss << in.rdbuf();
                    std::string json_file_content = ss.str();

                    auto adv = _dev.as<rs400::advanced_mode>();
                    adv.load_json(json_file_content);
                    ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
                }
                else
                    ROS_WARN_STREAM("JSON file provided doesn't exist! (" << _json_file_path << ")");
            }
            else
                ROS_WARN("Device does not support advanced settings!");
        }
        else
            ROS_INFO("JSON file is not provided");

        ROS_INFO_STREAM("ROS Node Namespace: " << _namespace);

        auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
        ROS_INFO_STREAM("Device Name: " << camera_name);

        ROS_INFO_STREAM("Device Serial No: " << _serial_no);

        auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        ROS_INFO_STREAM("Device FW version: " << fw_ver);

        auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        ROS_INFO_STREAM("Device Product ID: 0x" << pid);

        ROS_INFO_STREAM("Enable PointCloud: " << ((_pointcloud)?"On":"Off"));
        ROS_INFO_STREAM("Align Depth: " << ((_align_depth)?"On":"Off"));
        ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

        auto dev_sensors = _dev.query_sensors();

        ROS_INFO_STREAM("Device Sensors: ");
        for(auto&& elem : dev_sensors)
        {
            std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
            if ("Stereo Module" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
                _sensors[INFRA2] = elem;

            }
            else if ("Coded-Light Depth Sensor" == module_name)
            {
                _sensors[DEPTH] = elem;
                _sensors[INFRA1] = elem;
            }
            else if ("RGB Camera" == module_name)
            {
                _sensors[COLOR] = elem;
            }
            else if ("Wide FOV Camera" == module_name)
            {
                _sensors[FISHEYE] = elem;
            }
            else if ("Motion Module" == module_name)
            {
                _sensors[GYRO] = elem;
                _sensors[ACCEL] = elem;
            }
            else
            {
                ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense! Terminating RealSense Node...");
                ros::shutdown();
                exit(1);
            }
            ROS_INFO_STREAM(std::string(elem.get_info(RS2_CAMERA_INFO_NAME)) << " was found.");
        }


        // Update "enable" map
        std::vector<std::vector<stream_index_pair>> streams(IMAGE_STREAMS);
        streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
        for (auto& elem : streams)
        {
            for (auto& stream_index : elem)
            {
                if (_enable[stream_index] && _sensors.find(stream_index) == _sensors.end()) // check if device supports the enabled stream
                {
                    ROS_INFO_STREAM("(" << rs2_stream_to_string(stream_index.first) << ", " << stream_index.second << ") sensor isn't supported by current device! -- Skipping...");
                    _enable[stream_index] = false;
                }
            }
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

void RealSenseNode::TemperatureUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
 
    try 
    {
        auto dbg = _dev.as<rs2::debug_protocol>();
        std::vector<uint8_t> cmd = { 0x14, 0, 0xab, 0xcd, 0x2a, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
     
        auto res = dbg.send_and_receive_raw_data(cmd);
        temperature_ = res[4];
                
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  
        stat.add("Projector Temperature", temperature_);
        if (temperature_ > 50) 
        {
            stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Temperature is Higher than 50 Degree Celsius");
        }
    } 
    catch (const rs2::error& e) 
    {
        ROS_ERROR_STREAM_THROTTLE(3, "Can not check device temperature " << e.what());
    }

}

void RealSenseNode::setupServices()
{
  ROS_INFO("setupServices...");
  _enable_streams_service = _pnh.advertiseService("enable_streams", &RealSenseNode::enableStreams, this);
}

void RealSenseNode::setupPublishers()
{
    ROS_INFO("setupPublishers...");
    image_transport::ImageTransport image_transport(_node_handle);

    std::vector<stream_index_pair> image_stream_types;
    for (auto& stream_vec : IMAGE_STREAMS)
    {
        for (auto& stream : stream_vec)
        {
            image_stream_types.push_back(stream);
        }
    }

    temp_diagnostic_updater_.setHardwareIDf("D435_temperature");
    temp_diagnostic_updater_.add("Temperature", this, &RealSenseNode::TemperatureUpdate);

    temp_update_timer_ = _node_handle.createTimer(ros::Duration(0.1), boost::bind(&diagnostic_updater::Updater::update, &temp_diagnostic_updater_));

    for (auto& stream : image_stream_types)
    {
        if (_enable[stream])
        {
            std::stringstream image_raw, camera_info;
            bool rectified_image = false;
            if (stream == DEPTH || stream == INFRA1 || stream == INFRA2)
                rectified_image = true;

            image_raw << _stream_name[stream] << "/image_" << ((rectified_image)?"rect_":"") << "raw";
            camera_info << _stream_name[stream] << "/camera_info";

             
           

            std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], _stream_name[stream], _serial_no));
            _image_publishers[stream] = {image_transport.advertise(image_raw.str(), 1), frequency_diagnostics};
            _info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(camera_info.str(), 1);

            if (_align_depth && (stream != DEPTH))
            {
                std::stringstream aligned_image_raw, aligned_camera_info;
                aligned_image_raw << "aligned_depth_to_" << _stream_name[stream] << "/image_raw";
                aligned_camera_info << "aligned_depth_to_" << _stream_name[stream] << "/camera_info";

                std::string aligned_stream_name = "aligned_depth_to_" + _stream_name[stream];
                std::shared_ptr<FrequencyDiagnostics> frequency_diagnostics(new FrequencyDiagnostics(_fps[stream], aligned_stream_name, _serial_no));
                _depth_aligned_image_publishers[stream] = {image_transport.advertise(aligned_image_raw.str(), 1), frequency_diagnostics};
                _depth_aligned_info_publisher[stream] = _node_handle.advertise<sensor_msgs::CameraInfo>(aligned_camera_info.str(), 1);
            }

            if (stream == DEPTH && _pointcloud)
            {
                _pointcloud_xyz_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/points", 1);
                _pointcloud_xyzrgb_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
            }
        }
    }

    if (_enable[FISHEYE] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[FISHEYE] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_fisheye", 1, true);
    }

    if (_enable[COLOR] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[COLOR] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_color", 1, true);
    }

    if (_enable[INFRA1] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA1] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra1", 1, true);
    }

    if (_enable[INFRA2] &&
        _enable[DEPTH])
    {
        _depth_to_other_extrinsics_publishers[INFRA2] = _node_handle.advertise<Extrinsics>("extrinsics/depth_to_infra2", 1, true);
    }

    if (_enable[GYRO])
    {
        _imu_publishers[GYRO] = _node_handle.advertise<sensor_msgs::Imu>("gyro/sample", 100);
        _info_publisher[GYRO] = _node_handle.advertise<IMUInfo>("gyro/imu_info", 1, true);
    }

    if (_enable[ACCEL])
    {
        _imu_publishers[ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("accel/sample", 100);
        _info_publisher[ACCEL] = _node_handle.advertise<IMUInfo>("accel/imu_info", 1, true);
    }
}

void RealSenseNode::alignFrame(const rs2_intrinsics& from_intrin,
                                   const rs2_intrinsics& other_intrin,
                                   rs2::frame from_image,
                                   uint32_t output_image_bytes_per_pixel,
                                   const rs2_extrinsics& from_to_other,
                                   std::vector<uint8_t>& out_vec)
{
    static const auto meter_to_mm = 0.001f;
    uint8_t* p_out_frame = out_vec.data();

    static const auto blank_color = 0x00;
    memset(p_out_frame, blank_color, other_intrin.height * other_intrin.width * output_image_bytes_per_pixel);

    auto p_from_frame = reinterpret_cast<const uint8_t*>(from_image.get_data());
    auto from_stream_type = from_image.get_profile().stream_type();
    float depth_units = ((from_stream_type == RS2_STREAM_DEPTH)?_depth_scale_meters:1.f);
#pragma omp parallel for schedule(dynamic)
    for (int from_y = 0; from_y < from_intrin.height; ++from_y)
    {
        int from_pixel_index = from_y * from_intrin.width;
        for (int from_x = 0; from_x < from_intrin.width; ++from_x, ++from_pixel_index)
        {
            // Skip over depth pixels with the value of zero
            float depth = (from_stream_type == RS2_STREAM_DEPTH)?(depth_units * ((const uint16_t*)p_from_frame)[from_pixel_index]): 1.f;
            if (depth)
            {
                // Map the top-left corner of the depth pixel onto the other image
                float from_pixel[2] = { from_x - 0.5f, from_y - 0.5f }, from_point[3], other_point[3], other_pixel[2];
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

                // Map the bottom-right corner of the depth pixel onto the other image
                from_pixel[0] = from_x + 0.5f; from_pixel[1] = from_y + 0.5f;
                rs2_deproject_pixel_to_point(from_point, &from_intrin, from_pixel, depth);
                rs2_transform_point_to_point(other_point, &from_to_other, from_point);
                rs2_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
                const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

                if (other_x0 < 0 || other_y0 < 0 || other_x1 >= other_intrin.width || other_y1 >= other_intrin.height)
                    continue;

                for (int y = other_y0; y <= other_y1; ++y)
                {
                    for (int x = other_x0; x <= other_x1; ++x)
                    {
                        int out_pixel_index = y * other_intrin.width + x;
                        uint16_t* p_from_depth_frame = (uint16_t*)p_from_frame;
                        uint16_t* p_out_depth_frame = (uint16_t*)p_out_frame;
                        p_out_depth_frame[out_pixel_index] = p_from_depth_frame[from_pixel_index] * (depth_units / meter_to_mm);
                    }
                }
            }
        }
    }
}

void RealSenseNode::updateIsFrameArrived(std::map<stream_index_pair, bool>& is_frame_arrived,
                                             rs2_stream stream_type, int stream_index)
{
    try
    {
        is_frame_arrived.at({stream_type, stream_index}) = true;
    }
    catch (std::out_of_range)
    {
        ROS_ERROR_STREAM("Stream type is not supported! (" << stream_type << ", " << stream_index << ")");
    }
}

void RealSenseNode::publishAlignedDepthToOthers(rs2::frame depth_frame, const std::vector<rs2::frame>& frames, const ros::Time& t)
{
    for (auto&& other_frame : frames)
    {
        auto stream_type = other_frame.get_profile().stream_type();
        if (RS2_STREAM_DEPTH == stream_type)
            continue;

        auto stream_index = other_frame.get_profile().stream_index();
        stream_index_pair sip{stream_type, stream_index};
        auto& info_publisher = _depth_aligned_info_publisher.at(sip);
        auto& image_publisher = _depth_aligned_image_publishers.at(sip);
        if(0 != info_publisher.getNumSubscribers() ||
           0 != image_publisher.first.getNumSubscribers())
        {
            auto from_image_frame = depth_frame.as<rs2::video_frame>();
            auto& out_vec = _aligned_depth_images[sip];
            alignFrame(_stream_intrinsics[DEPTH], _stream_intrinsics[sip],
                       depth_frame, from_image_frame.get_bytes_per_pixel(),
                       _depth_to_other_extrinsics[sip], out_vec);

            auto& from_image = _depth_aligned_image[sip];
            from_image.data = out_vec.data();

            publishFrame(depth_frame, t, sip,
                         _depth_aligned_image,
                         _depth_aligned_info_publisher,
                         _depth_aligned_image_publishers, _depth_aligned_seq,
                         _depth_aligned_camera_info, _optical_frame_id,
                         _depth_aligned_encoding, false);
        }
    }
}

void RealSenseNode::filterFrame(rs2::frame& frame)
{
    for (auto&& filter : filters)
    {
        if (filter.is_enabled)
        {
            frame = filter.filter.process(frame);
        }
    }
}

void RealSenseNode::enable_devices()
{
	for (auto& streams : IMAGE_STREAMS)
	{
		for (auto& elem : streams)
		{
			if (_enable[elem])
			{
				auto& sens = _sensors[elem];
				auto profiles = sens.get_stream_profiles();
				for (auto& profile : profiles)
				{
					auto video_profile = profile.as<rs2::video_stream_profile>();
					ROS_DEBUG_STREAM("Sensor profile: " <<
									 "Format: " << video_profile.format() <<
									 ", Width: " << video_profile.width() <<
									 ", Height: " << video_profile.height() <<
									 ", FPS: " << video_profile.fps());

					if (video_profile.format() == _format[elem] &&
						(_width[elem] == 0 || video_profile.width() == _width[elem]) &&
						(_height[elem] == 0 || video_profile.height() == _height[elem]) &&
						(_fps[elem] == 0 || video_profile.fps() == _fps[elem]) &&
						video_profile.stream_index() == elem.second)
					{
						_width[elem] = video_profile.width();
						_height[elem] = video_profile.height();
						_fps[elem] = video_profile.fps();

						_enabled_profiles[elem].push_back(profile);

						_image[elem] = cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));

						ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
						break;
					}
				}
				if (_enabled_profiles.find(elem) == _enabled_profiles.end())
				{
					ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
						" Stream: " << rs2_stream_to_string(elem.first) <<
						", Stream Index: " << elem.second <<
						", Format: " << _format[elem] <<
						", Width: " << _width[elem] <<
						", Height: " << _height[elem] <<
						", FPS: " << _fps[elem]);
					_enable[elem] = false;
				}
			}
		}
	}
	if (_align_depth)
	{
		for (auto& profiles : _enabled_profiles)
		{
			_depth_aligned_image[profiles.first] = cv::Mat(_width[DEPTH], _height[DEPTH], _image_format[DEPTH], cv::Scalar(0, 0, 0));
		}
	}
}

void RealSenseNode::setupStreams()
{
	ROS_INFO("setupStreams...");
	enable_devices();
    try{
		// Publish image stream info
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                updateStreamCalibData(video_profile);
            }
        }

        _frame_callback = [this](rs2::frame frame)
        {
            std::lock_guard<std::mutex> syncer_mutex_guard(syncer_mutex);
            try{
                depth_callback_timer_.setPeriod(depth_callback_timeout_, true);
                // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
                // and the incremental timestamp from the camera.
                // In sync mode the timestamp is based on ROS time
                if ((false == _intialize_time_base) || (_prev_camera_time_stamp > frame.get_timestamp()))
                {
                    if (RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME == frame.get_frame_timestamp_domain())
                        ROS_WARN("Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)");

                    _intialize_time_base = true;
                    _ros_time_base = ros::Time::now();
                    _camera_time_base = frame.get_timestamp();
                }
                _prev_camera_time_stamp = frame.get_timestamp();

                ros::Time t;
                if (_use_ros_time)
                    t = ros::Time::now();
                else
                    t = ros::Time(_ros_time_base.toSec()+ (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000);


                std::map<stream_index_pair, bool> is_frame_arrived(_is_frame_arrived);
                std::vector<rs2::frame> frames;
                if (frame.is<rs2::frameset>())
                {
                    ROS_DEBUG("Frameset arrived.");
                    bool is_depth_arrived = false;
                    rs2::frame depth_frame;
                    auto frameset = frame.as<rs2::frameset>();

                    const bool publish_this_frameset = (_publish_every_nth_frameset == 0) || ((++_publish_skip_counter) % _publish_every_nth_frameset == 0);

                    if (publish_this_frameset)
                    {
                        for (auto it = frameset.begin(); it != frameset.end(); ++it)
                        {
                            auto f = (*it);
                            auto stream_type = f.get_profile().stream_type();
                            auto stream_index = f.get_profile().stream_index();
                            updateIsFrameArrived(is_frame_arrived, stream_type, stream_index);

                            ROS_DEBUG("Frameset contain (%s, %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                                    rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame.get_timestamp(), t.toNSec());

                            if (stream_type == RS2_STREAM_DEPTH)
                            {
                                filterFrame(f);
                                if (_use_depth_rate_monitor)
                                {
                                    _depth_rate_monitor_num_of_new_frames++;
                                }
                            }

                            if (stream_type == RS2_STREAM_COLOR)
                            {
                                if (_use_depth_rate_monitor)
                                {
                                    _depth_rate_monitor_num_of_new_color_frames++;
                                }
                            }

                            stream_index_pair sip{stream_type,stream_index};
                            publishFrame(f, t,
                                        sip,
                                        _image,
                                        _info_publisher,
                                        _image_publishers, _seq,
                                        _camera_info, _optical_frame_id,
                                        _encoding);
                            if (_align_depth && stream_type != RS2_STREAM_DEPTH)
                            {
                                frames.push_back(f);
                            }
                            else
                            {
                                depth_frame = f;
                                is_depth_arrived = true;
                            }
                        }

                        if (_align_depth && is_depth_arrived)
                        {
                            //auto color_it = std::find_if(frames.begin(), frames.end(), [](const rs2::frame& fr){ return fr.get_profile().stream_type() == RS2_STREAM_COLOR; });
                            //if (color_it != frames.end())
                            //{
                            //    ROS_WARN("%s: ALIGNED: DEPTH: %f; COLOR: %f", _namespace.c_str(), depth_frame.get_timestamp() / 1000.0, (*color_it).get_timestamp() / 1000.0);
                            //}
                            //else
                            //{
                            //    ROS_WARN("%s: ALIGNED: DEPTH: %f; COLOR: NO COLOR FRAME!", _namespace.c_str(), depth_frame.get_timestamp() / 1000.0);
                            //}
                            ROS_DEBUG("publishAlignedDepthToOthers(...)");
                            publishAlignedDepthToOthers(depth_frame, frames, t);
                            if (_use_depth_rate_monitor)
                            {
                                _depth_rate_monitor_num_of_new_aligned_frames++;
                            }
                        }
                    }
                }
                else
                {
                    auto stream_type = frame.get_profile().stream_type();
                    auto stream_index = frame.get_profile().stream_index();
                    updateIsFrameArrived(is_frame_arrived, stream_type, stream_index);
                    ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                              rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame.get_timestamp(), t.toNSec());

                    if (stream_type == RS2_STREAM_DEPTH)
                    {
                        filterFrame(frame);
                        if (_use_depth_rate_monitor)
                        {
                            _depth_rate_monitor_num_of_new_frames++;
                        }
                    }

                    stream_index_pair sip{stream_type,stream_index};
                    publishFrame(frame, t,
                                 sip,
                                 _image,
                                 _info_publisher,
                                 _image_publishers, _seq,
                                 _camera_info, _optical_frame_id,
                                 _encoding);
                }

                if(_pointcloud && (0 != _pointcloud_xyzrgb_publisher.getNumSubscribers()))
                {
                    ROS_DEBUG("publishRgbToDepthPCTopic(...)");
                    publishRgbToDepthPCTopic(t, is_frame_arrived);
                }
                if(_pointcloud && (0 != _pointcloud_xyz_publisher.getNumSubscribers()))
                {
                    ROS_DEBUG("publishDepthPCTopic(...)");
                    publishDepthPCTopic(t, is_frame_arrived);
                }
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An error has occurred during frame callback: " << ex.what());
            }
        }; // _frame_callback

        // Streaming IMAGES
        for (auto& streams : IMAGE_STREAMS)
        {
            std::vector<rs2::stream_profile> profiles;
            for (auto& elem : streams)
            {
                if (!_enabled_profiles[elem].empty())
                {
                    profiles.insert(profiles.begin(),
                                    _enabled_profiles[elem].begin(),
                                    _enabled_profiles[elem].end());
                }
            }

            if (!profiles.empty())
            {
                auto stream = streams.front();
                auto& sens = _sensors[stream];
                sens.open(profiles);

                if (DEPTH == stream)
                {
                    auto depth_sensor = sens.as<rs2::depth_sensor>();
                    _depth_scale_meters = depth_sensor.get_depth_scale();
                }

                if (_sync_frames)
                {
                    sens.start(_syncer);
                }
                else
                {
                    sens.start(_frame_callback);
                }
                depth_callback_timer_.start();
            }
        }//end for

        if (_sync_frames)
        {
            _syncer.start(_frame_callback);
        }

        // Streaming HID
        for (const auto streams : HID_STREAMS)
        {
            for (auto& elem : streams)
            {
                if (_enable[elem])
                {
                    auto& sens = _sensors[elem];
                    auto profiles = sens.get_stream_profiles();
                    for (rs2::stream_profile& profile : profiles)
                    {
                        if (profile.fps() == _fps[elem] &&
                            profile.format() == _format[elem])
                        {
                            _enabled_profiles[elem].push_back(profile);
                            break;
                        }
                    }
                }
            }
        }

        auto gyro_profile = _enabled_profiles.find(GYRO);
        auto accel_profile = _enabled_profiles.find(ACCEL);

        if (gyro_profile != _enabled_profiles.end() &&
            accel_profile != _enabled_profiles.end())
        {
            std::vector<rs2::stream_profile> profiles;
            profiles.insert(profiles.begin(), gyro_profile->second.begin(), gyro_profile->second.end());
            profiles.insert(profiles.begin(), accel_profile->second.begin(), accel_profile->second.end());
            auto& sens = _sensors[GYRO];
            sens.open(profiles);

            sens.start([this](rs2::frame frame){
                auto stream = frame.get_profile().stream_type();
                if (false == _intialize_time_base)
                    return;

                ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                          rs2_stream_to_string(frame.get_profile().stream_type()),
                          frame.get_profile().stream_index(),
                          rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

                auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
                if (0 != _info_publisher[stream_index].getNumSubscribers() ||
                    0 != _imu_publishers[stream_index].getNumSubscribers())
                {
                    double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                    ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                    auto imu_msg = sensor_msgs::Imu();
                    imu_msg.header.frame_id = _optical_frame_id[stream_index];
                    imu_msg.orientation.x = 0.0;
                    imu_msg.orientation.y = 0.0;
                    imu_msg.orientation.z = 0.0;
                    imu_msg.orientation.w = 0.0;
                    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    auto axes = *(reinterpret_cast<const float3*>(frame.get_data()));
                    if (GYRO == stream_index)
                    {
                        imu_msg.angular_velocity.x = axes.x;
                        imu_msg.angular_velocity.y = axes.y;
                        imu_msg.angular_velocity.z = axes.z;
                    }
                    else if (ACCEL == stream_index)
                    {
                        imu_msg.linear_acceleration.x = axes.x;
                        imu_msg.linear_acceleration.y = axes.y;
                        imu_msg.linear_acceleration.z = axes.z;
                    }
                    _seq[stream_index] += 1;
                    imu_msg.header.seq = _seq[stream_index];
                    imu_msg.header.stamp = t;
                    _imu_publishers[stream_index].publish(imu_msg);
                    ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
                }
            });

            if (_enable[GYRO])
            {
                ROS_INFO_STREAM(_stream_name[GYRO] << " stream is enabled - " << "fps: " << _fps[GYRO]);
                auto gyroInfo = getImuInfo(GYRO);
                _info_publisher[GYRO].publish(gyroInfo);
            }

            if (_enable[ACCEL])
            {
                ROS_INFO_STREAM(_stream_name[ACCEL] << " stream is enabled - " << "fps: " << _fps[ACCEL]);
                auto accelInfo = getImuInfo(ACCEL);
                _info_publisher[ACCEL].publish(accelInfo);
            }
        }

        if (_enable[DEPTH] &&
            _enable[FISHEYE])
        {
            static const char* frame_id = "depth_to_fisheye_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, FISHEYE);
            _depth_to_other_extrinsics[FISHEYE] = ex;
            _depth_to_other_extrinsics_publishers[FISHEYE].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[COLOR])
        {
            static const char* frame_id = "depth_to_color_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, COLOR);
            _depth_to_other_extrinsics[COLOR] = ex;
            _depth_to_other_extrinsics_publishers[COLOR].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[INFRA1])
        {
            static const char* frame_id = "depth_to_infra1_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, INFRA1);
            _depth_to_other_extrinsics[INFRA1] = ex;
            _depth_to_other_extrinsics_publishers[INFRA1].publish(rsExtrinsicsToMsg(ex, frame_id));
        }

        if (_enable[DEPTH] &&
            _enable[INFRA2])
        {
            static const char* frame_id = "depth_to_infra2_extrinsics";
            auto ex = getRsExtrinsics(DEPTH, INFRA2);
            _depth_to_other_extrinsics[INFRA2] = ex;
            _depth_to_other_extrinsics_publishers[INFRA2].publish(rsExtrinsicsToMsg(ex, frame_id));
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

void RealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
    auto intrinsic = video_profile.get_intrinsics();
    _stream_intrinsics[stream_index] = intrinsic;
    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

    _camera_info[stream_index].K.at(0) = intrinsic.fx;
    _camera_info[stream_index].K.at(2) = intrinsic.ppx;
    _camera_info[stream_index].K.at(4) = intrinsic.fy;
    _camera_info[stream_index].K.at(5) = intrinsic.ppy;
    _camera_info[stream_index].K.at(8) = 1;

    _camera_info[stream_index].P.at(0) = _camera_info[stream_index].K.at(0);
    _camera_info[stream_index].P.at(1) = 0;
    _camera_info[stream_index].P.at(2) = _camera_info[stream_index].K.at(2);
    _camera_info[stream_index].P.at(3) = 0;
    _camera_info[stream_index].P.at(4) = 0;
    _camera_info[stream_index].P.at(5) = _camera_info[stream_index].K.at(4);
    _camera_info[stream_index].P.at(6) = _camera_info[stream_index].K.at(5);
    _camera_info[stream_index].P.at(7) = 0;
    _camera_info[stream_index].P.at(8) = 0;
    _camera_info[stream_index].P.at(9) = 0;
    _camera_info[stream_index].P.at(10) = 1;
    _camera_info[stream_index].P.at(11) = 0;

    rs2::stream_profile depth_profile;
    if (!getEnabledProfile(DEPTH, depth_profile))
    {
        ROS_ERROR_STREAM("Given depth profile is not supported by current device!");
        ros::shutdown();
        exit(1);
    }


    _camera_info[stream_index].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].R.at(0) = 1.0;
    _camera_info[stream_index].R.at(1) = 0.0;
    _camera_info[stream_index].R.at(2) = 0.0;
    _camera_info[stream_index].R.at(3) = 0.0;
    _camera_info[stream_index].R.at(4) = 1.0;
    _camera_info[stream_index].R.at(5) = 0.0;
    _camera_info[stream_index].R.at(6) = 0.0;
    _camera_info[stream_index].R.at(7) = 0.0;
    _camera_info[stream_index].R.at(8) = 1.0;

    for (int i = 0; i < 5; i++)
    {
        _camera_info[stream_index].D.push_back(intrinsic.coeffs[i]);
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].P.at(3) = 0;     // Tx
        _camera_info[stream_index].P.at(7) = 0;     // Ty
    }

    if (_align_depth)
    {
        for (auto& profiles : _enabled_profiles)
        {
            for (auto& profile : profiles.second)
            {
                auto video_profile = profile.as<rs2::video_stream_profile>();
                stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
                _depth_aligned_camera_info[stream_index] = _camera_info[stream_index];
            }
        }
    }
}

tf::Quaternion RealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void RealSenseNode::publish_static_tf(const ros::Time& t,
                                          const float3& trans,
                                          const quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    msg.transform.rotation.w = q.w;
    _static_tf_broadcaster.sendTransform(msg);
}

void RealSenseNode::publishStaticTransforms()
{
    ROS_INFO("publishStaticTransforms...");
    // Publish static transforms
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);

    // Get the current timestamp for all static transforms
    ros::Time transform_ts_ = ros::Time::now();

    // The depth frame is used as the base link.
    // Hence no additional transformation is done from base link to depth frame.
    // Transform base link to depth frame
    float3 zero_trans{0, 0, 0};
    publish_static_tf(transform_ts_, zero_trans, quaternion{0, 0, 0, 1}, _base_frame_id, _frame_id[DEPTH]);

    // Transform depth frame to depth optical frame
    quaternion q{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
    publish_static_tf(transform_ts_, zero_trans, q, _frame_id[DEPTH], _optical_frame_id[DEPTH]);

    rs2::stream_profile depth_profile;
    if (!getEnabledProfile(DEPTH, depth_profile))
    {
        ROS_ERROR_STREAM("Given depth profile is not supported by current device!");
        ros::shutdown();
        exit(1);
    }


    if (_enable[COLOR])
    {
        // Transform base to color
        const auto& ex = getRsExtrinsics(COLOR, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[COLOR]);

        // Transform color frame to color optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[COLOR], _optical_frame_id[COLOR]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[COLOR]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[COLOR], _optical_frame_id[COLOR]);
        }
    }

    if (_enable[INFRA1])
    {
        const auto& ex = getRsExtrinsics(INFRA1, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra1
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[INFRA1]);

        // Transform infra1 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[INFRA1], _optical_frame_id[INFRA1]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[INFRA1]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[INFRA1], _optical_frame_id[INFRA1]);
        }
    }

    if (_enable[INFRA2])
    {
        const auto& ex = getRsExtrinsics(INFRA2, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra2
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[INFRA2]);

        // Transform infra2 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[INFRA2], _optical_frame_id[INFRA2]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[INFRA2]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[INFRA2], _optical_frame_id[INFRA2]);
        }
    }

    if (_enable[FISHEYE])
    {
        const auto& ex = getRsExtrinsics(FISHEYE, DEPTH);
        auto Q = rotationMatrixToQuaternion(ex.rotation);
        Q = quaternion_optical * Q * quaternion_optical.inverse();

        // Transform base to infra2
        float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
        quaternion q1{Q.getX(), Q.getY(), Q.getZ(), Q.getW()};
        publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _frame_id[FISHEYE]);

        // Transform infra2 frame to infra1 optical frame
        quaternion q2{quaternion_optical.getX(), quaternion_optical.getY(), quaternion_optical.getZ(), quaternion_optical.getW()};
        publish_static_tf(transform_ts_, zero_trans, q2, _frame_id[FISHEYE], _optical_frame_id[FISHEYE]);

        if (_align_depth)
        {
            publish_static_tf(transform_ts_, trans, q1, _base_frame_id, _depth_aligned_frame_id[FISHEYE]);
            publish_static_tf(transform_ts_, zero_trans, q2, _depth_aligned_frame_id[FISHEYE], _optical_frame_id[FISHEYE]);
        }
    }
}

void RealSenseNode::publishDepthPCTopic(const ros::Time& t, const std::map<stream_index_pair, bool>& is_frame_arrived)
{
    try
    {
        if (!is_frame_arrived.at(DEPTH))
        {
            ROS_DEBUG("Skipping publish PC topic! Depth frame didn't arrive.");
            return;
        }
    }
    catch (std::out_of_range)
    {
        ROS_DEBUG("Skipping publish PC topic! Depth frame didn't configure.");
        return;
    }


    auto image_depth16 = reinterpret_cast<const uint16_t*>(_image[DEPTH].data);
    auto depth_intrinsics = _stream_intrinsics[DEPTH];
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

    modifier.setPointCloud2Fields(3,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

    float depth_point[3], scaled_depth;


    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y)
    {
        for (int x = 0; x < depth_intrinsics.width; ++x)
        {
            scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
            float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
            rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);

            if (depth_point[2] <= 0.f)
            {
                depth_point[0] = 0.f;
                depth_point[1] = 0.f;
                depth_point[2] = 0.f;
            }

            *iter_x = depth_point[0];
            *iter_y = depth_point[1];
            *iter_z = depth_point[2];

            ++image_depth16;
            ++iter_x; ++iter_y; ++iter_z;
        }
    }

    _pointcloud_xyz_publisher.publish(msg_pointcloud);
}

void RealSenseNode::publishRgbToDepthPCTopic(const ros::Time& t, const std::map<stream_index_pair, bool>& is_frame_arrived)
{
    try
    {
        if (!is_frame_arrived.at(COLOR) || !is_frame_arrived.at(DEPTH))
        {
            ROS_DEBUG("Skipping publish PC topic! Color or Depth frame didn't arrive.");
            return;
        }
    }
    catch (std::out_of_range)
    {
        ROS_DEBUG("Skipping publish PC topic! Color or Depth frame didn't configure.");
        return;
    }

    auto& depth2color_extrinsics = _depth_to_other_extrinsics[COLOR];
    auto color_intrinsics = _stream_intrinsics[COLOR];
    auto image_depth16 = reinterpret_cast<const uint16_t*>(_image[DEPTH].data);
    auto depth_intrinsics = _stream_intrinsics[DEPTH];
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
    msg_pointcloud.width = depth_intrinsics.width;
    msg_pointcloud.height = depth_intrinsics.height;
    msg_pointcloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");

    float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
    unsigned char* color_data = _image[COLOR].data;

    // Fill the PointCloud2 fields
    for (int y = 0; y < depth_intrinsics.height; ++y)
    {
        for (int x = 0; x < depth_intrinsics.width; ++x)
        {
            scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
            float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
            rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);

            if (depth_point[2] <= 0.f || depth_point[2] > 5.f)
            {
                depth_point[0] = 0.f;
                depth_point[1] = 0.f;
                depth_point[2] = 0.f;
            }

            *iter_x = depth_point[0];
            *iter_y = depth_point[1];
            *iter_z = depth_point[2];

            rs2_transform_point_to_point(color_point, &depth2color_extrinsics, depth_point);
            rs2_project_point_to_pixel(color_pixel, &color_intrinsics, color_point);

            if (color_pixel[1] < 0.f || color_pixel[1] >= color_intrinsics.height
                || color_pixel[0] < 0.f || color_pixel[0] >= color_intrinsics.width)
            {
                // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
                // This color value is same as the librealsense out of bounds color value.
                *iter_r = static_cast<uint8_t>(96);
                *iter_g = static_cast<uint8_t>(157);
                *iter_b = static_cast<uint8_t>(198);
            }
            else
            {
                auto i = static_cast<int>(color_pixel[0]);
                auto j = static_cast<int>(color_pixel[1]);

                auto offset = i * 3 + j * color_intrinsics.width * 3;
                *iter_r = static_cast<uint8_t>(color_data[offset]);
                *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
                *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
            }

            ++image_depth16;
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
    }

    _pointcloud_xyzrgb_publisher.publish(msg_pointcloud);
}

Extrinsics RealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics, const std::string& frame_id) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }

    extrinsicsMsg.header.frame_id = frame_id;
    return extrinsicsMsg;
}

rs2_extrinsics RealSenseNode::getRsExtrinsics(const stream_index_pair& from_stream, const stream_index_pair& to_stream)
{
    auto& from = _enabled_profiles[from_stream].front();
    auto& to = _enabled_profiles[to_stream].front();
    return from.get_extrinsics_to(to);
}

IMUInfo RealSenseNode::getImuInfo(const stream_index_pair& stream_index)
{
    IMUInfo info{};
    auto sp = _enabled_profiles[stream_index].front().as<rs2::motion_stream_profile>();
    auto imuIntrinsics = sp.get_motion_intrinsics();
    if (GYRO == stream_index)
    {
        info.header.frame_id = "imu_gyro";
    }
    else if (ACCEL == stream_index)
    {
        info.header.frame_id = "imu_accel";
    }

    auto index = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            info.data[index] = imuIntrinsics.data[i][j];
            ++index;
        }
        info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
        info.bias_variances[i] = imuIntrinsics.bias_variances[i];
    }
    return info;
}

void RealSenseNode::publishFrame(rs2::frame f, const ros::Time& t,
                                     const stream_index_pair& stream,
                                     std::map<stream_index_pair, cv::Mat>& images,
                                     const std::map<stream_index_pair, ros::Publisher>& info_publishers,
                                     const std::map<stream_index_pair, ImagePublisherWithFrequencyDiagnostics>& image_publishers,
                                     std::map<stream_index_pair, int>& seq,
                                     std::map<stream_index_pair, sensor_msgs::CameraInfo>& camera_info,
                                     const std::map<stream_index_pair, std::string>& optical_frame_id,
                                     const std::map<stream_index_pair, std::string>& encoding,
                                     bool copy_data_from_frame)
{
    ROS_DEBUG("publishFrame(...)");
    auto& image = images[stream];

    if (copy_data_from_frame) {
        if (stream != DEPTH) {
            image.data = (uint8_t*)f.get_data();
        } else {
            auto depth_data = (uint16_t*)f.get_data();
            auto& image_data = reinterpret_cast<uint16_t*&>(image.data);
            image_data = depth_data;
        }
    }

    ++(seq[stream]);
    auto& info_publisher = info_publishers.at(stream);
    auto& image_publisher = image_publishers.at(stream);
    if(0 != info_publisher.getNumSubscribers() ||
       0 != image_publisher.first.getNumSubscribers())
    {
        auto width = 0;
        auto height = 0;
        auto bpp = 1;
        if (f.is<rs2::video_frame>())
        {
            auto image = f.as<rs2::video_frame>();
            width = image.get_width();
            height = image.get_height();
            bpp = image.get_bytes_per_pixel();
        }

        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), encoding.at(stream), image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = width * bpp;
        img->header.frame_id = optical_frame_id.at(stream);
        img->header.stamp = t;
        img->header.seq = seq[stream];

        auto& cam_info = camera_info.at(stream);
        cam_info.header.stamp = t;
        cam_info.header.seq = seq[stream];
        info_publisher.publish(cam_info);

        image_publisher.first.publish(img);
        image_publisher.second->update();
        ROS_DEBUG("%s stream published", rs2_stream_to_string(f.get_profile().stream_type()));
    }
}

void RealSenseNode::setHealthTimers() {
  auto reset_this = [this](const ros::TimerEvent&) -> void
  {
    ROS_WARN_STREAM("RealSense " << _serial_no << " driver timeout! Resetting");
    resetNode();
  };
  depth_callback_timer_ = _node_handle.createTimer(depth_callback_timeout_, reset_this, false, !_dev);

  if (_use_depth_rate_monitor) {
    auto depth_rate_monitor_callback = [this](const ros::TimerEvent&) -> void
    {
        if (_depth_rate_monitor_initial_cycles_counter < _depth_rate_monitor_initial_cycles_number) {
            _depth_rate_monitor_initial_cycles_counter++;
        } else {
            // Check depth
            if (_depth_rate_monitor_num_of_new_frames < _depth_rate_monitor_min_frames_number_per_cycle) {
                ROS_WARN(
                    "Number of new depth frames is lower than the minimum: %d < %d: %s",
                    _depth_rate_monitor_num_of_new_frames,
                    _depth_rate_monitor_min_frames_number_per_cycle,
                    _namespace.c_str()
                );
                _depth_rate_monitor_consequent_low_rate_frames_counter++;
                if (_depth_rate_monitor_consequent_low_rate_frames_counter > _depth_rate_monitor_consequent_low_rate_frames_limit) {
                    ROS_WARN(
                        "Inconsistent depth rate detected: received %d concequent low rates. Resetting the node: %s",
                        _depth_rate_monitor_consequent_low_rate_frames_counter,
                        _namespace.c_str()
                    );
                    resetNode();
                }
            } else {
                _depth_rate_monitor_consequent_low_rate_frames_counter = 0;
            }

            // Check aligned depth
            if (_depth_rate_monitor_num_of_new_aligned_frames < _depth_rate_monitor_min_frames_number_per_cycle) {
                ROS_WARN(
                    "Number of new aligned depth frames is lower than the minimum: %d < %d: %s",
                    _depth_rate_monitor_num_of_new_aligned_frames,
                    _depth_rate_monitor_min_frames_number_per_cycle,
                    _namespace.c_str()
                );
                _depth_rate_monitor_consequent_low_rate_aligned_frames_counter++;
                if (_depth_rate_monitor_consequent_low_rate_aligned_frames_counter > _depth_rate_monitor_consequent_low_rate_frames_limit) {
                    ROS_WARN(
                        "Inconsistent aligned depth rate detected: received %d concequent low rates. Resetting the node: %s",
                        _depth_rate_monitor_consequent_low_rate_aligned_frames_counter,
                        _namespace.c_str()
                    );
                    resetNode();
                }
            } else {
                _depth_rate_monitor_consequent_low_rate_aligned_frames_counter = 0;
            }

            // Check color
            if (_depth_rate_monitor_num_of_new_color_frames < _depth_rate_monitor_min_frames_number_per_cycle) {
                ROS_WARN(
                    "Number of new color frames is lower than the minimum: %d < %d: %s",
                    _depth_rate_monitor_num_of_new_color_frames,
                    _depth_rate_monitor_min_frames_number_per_cycle,
                    _namespace.c_str()
                );
                _depth_rate_monitor_consequent_low_rate_color_frames_counter++;
                if (_depth_rate_monitor_consequent_low_rate_color_frames_counter > _depth_rate_monitor_consequent_low_rate_frames_limit) {
                    ROS_WARN(
                        "Inconsistent color rate detected: received %d concequent low rates. Resetting the node: %s",
                        _depth_rate_monitor_consequent_low_rate_color_frames_counter,
                        _namespace.c_str()
                    );
                    resetNode();
                }
            } else {
                _depth_rate_monitor_consequent_low_rate_color_frames_counter = 0;
            }
        }

        _depth_rate_monitor_num_of_new_frames = 0;
        _depth_rate_monitor_num_of_new_aligned_frames = 0;
        _depth_rate_monitor_num_of_new_color_frames = 0;
    };
    _depth_rate_monitor_timer = _node_handle.createTimer(ros::Duration(1.0), depth_rate_monitor_callback);
  }
}

bool RealSenseNode::getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile)
    {
        // Assuming that all D400 SKUs have depth sensor
        auto profiles = _enabled_profiles[stream_index];
        auto it = std::find_if(profiles.begin(), profiles.end(),
                               [&](const rs2::stream_profile& profile)
                               { return (profile.stream_type() == stream_index.first); });
        if (it == profiles.end())
            return false;

        profile =  *it;
        return true;
    }




/**
Constructor for filter_options, takes a name and a filter.
*/
filter_options::filter_options(const std::string name, rs2::process_interface& filter) :
    filter_name(name),
    filter(filter),
    is_enabled(true) {}

filter_options::filter_options(filter_options&& other) :
    filter_name(std::move(other.filter_name)),
    filter(other.filter),
    is_enabled(other.is_enabled.load()) {}
