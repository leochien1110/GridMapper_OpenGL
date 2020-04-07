#include "rs_camera.h"
#include <unistd.h>

RS_Camera::RS_Camera()
{
    printf("RS_Camera called!\n");

    camera_stream = false;

}
void RS_Camera::init()
{
    std::cout << "**********************" << std::endl;
    std::cout << "RS_Camera Initializing" << std::endl;
    std::cout << "**********************" << std::endl;

    // Check avaible cameras
    devices = ctx.query_devices();

    std::cout << "Found " << devices.size() << " devices " << std::endl;
    
    if (devices.size() != 2) {
        std::cout << "[WARNING]Please unplug USB and try again." << std::endl;
        exit (EXIT_FAILURE);
    }
    
    // Setup Pipeline for each device
	for (auto&& dev : devices)
	{
        const char* dev_name = dev.get_info(RS2_CAMERA_INFO_NAME);
		printf("dev_name:%s\n",dev_name);
		const char* dev_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		rs2::pipeline pipe(ctx);

        // Assign Sensor
		auto advanced_sensors = dev.query_sensors();

        std::cout << "Sensor List:" << std::endl;
		for (auto&& sensor : advanced_sensors) {
			std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
			std::cout << module_name << std::endl;

			if (module_name == "Stereo Module") {
				depth_sensor = sensor;
			} else if (module_name == "RGB Camera") {
				color_sensor = sensor;
			} else if (module_name == "Tracking Module") {
				pose_sensor = sensor;
			}
		}

		// Setup configuration
		rs2::config cfg;
		cfg.enable_device(dev_num);
        // D435 & D435i
		if (strcmp("Intel RealSense D435", dev_name) == 0 || strcmp("Intel RealSense D435I", dev_name) == 0) {
			std::cout << "Camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected\n" << std::endl;
			// Stream Enable
            cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, framerate);
			cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, framerate);
            // Sensor Function Enable
            depth_sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
		}
		// T265
		if (strcmp("Intel RealSense T265", dev_name) == 0) {
            // Stream Enable
            cfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 1);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 2);
			std::cout << "Camera: " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected\n" << std::endl;
			// Sensor Function Enable
            pose_sensor.set_option(RS2_OPTION_ENABLE_MAPPING, 1.f);
            pose_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 1.f);
            pose_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 1.f);
		}
        pipe.start(cfg);
		pipelines.emplace_back(pipe);

	}


    std::cout << "Setting Filters..." << std::endl;
    std::cout << std::endl;
    // Filter config
    deci_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	thhd_filter.set_option(RS2_OPTION_MAX_DISTANCE, 10); //max depth range
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
}

void RS_Camera::start()
{
    if(!camera_stream){
        std::cout << "Starting RS_Camera Streaming Thread" << std::endl;
        streamThread = std::thread(&RS_Camera::stream, this);
    }
}

void RS_Camera::stream()
{
    if(camera_stream)
    {
        std::cout << "stream thread is running" << std::endl;
    }
    else
    {
        camera_stream = true;
        
        while(camera_stream)
        {
            read_pose();
            read_depth();
            //std::cout << camera_pose[0] << " "  \
                << camera_pose[1] << " "        \
                << camera_pose[2] << " "        \
                << " " << std::endl;
        }
    }
}

void RS_Camera::read_depth()
{
    // Depth data
    rs2::frameset data = pipelines[0].wait_for_frames();
    rs2::frame color_frame = data.get_color_frame();
    rs2::frame depth = data.get_depth_frame();	//Visualize 2D depth image
    depth = thhd_filter.process(depth);
    // Point cloud
    pc.map_to(color_frame);
    points = pc.calculate(depth);   // heavy function :(
    auto vertices = points.get_vertices();
    depth = depth.apply_filter(color_map);
    points_size =  points.size();
    for (int i = 0; i < points.size(); i++)
    {
        pc_vertices[i].x = vertices[i].x;
        pc_vertices[i].y = vertices[i].y;
        pc_vertices[i].z = vertices[i].z;
    }
    fov(depth);
}

void RS_Camera::read_pose()
{
    auto frames = pipelines[1].wait_for_frames();
    if( rs2::pose_frame f = frames.first_or_default(RS2_STREAM_POSE) ) {
        auto pose_data = f.get_pose_data();	// Get states data(x,y,z,qw,qx,qy,qz)
        // Quaternion(w,x,y,z)
        q.w = pose_data.rotation.w;
        q.x = -pose_data.rotation.z;
        q.y = pose_data.rotation.x;
        q.z = -pose_data.rotation.y;
        // Conver quaternion to euler angle
        float3 angle = Quat2Euler();
        // Local Position
        camera_state[0] = pose_data.translation.x;	//X
        camera_state[1] = -pose_data.translation.y;	//Y
        camera_state[2] = -pose_data.translation.z;	//Z
        //convert to y-z-x order
        camera_state[3] = angle.y;					//camera_pose[3]
        camera_state[4] = angle.z;					//camera_pose[4]
        camera_state[5] = angle.x;					//camera_pose[5]
    }
    mutex.lock();
    camera_pose[0] = init_camera_global_pos[0] + camera_state[0];
    camera_pose[1] = init_camera_global_pos[1] + camera_state[1];
    camera_pose[2] = init_camera_global_pos[2] + camera_state[2];
    camera_pose[3] = 0 + camera_state[3];
    camera_pose[4] = 0 + camera_state[4];
    camera_pose[5] = 0 + camera_state[5];
    camera_pose[6] = camera_state[0];
    camera_pose[7] = camera_state[1];
    camera_pose[8] = camera_state[2];
    mutex.unlock();
    get_rotate_matrix();
}
void RS_Camera::fov(rs2::frame &depth)
{
    intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_fov(&intr, FOV);
    half_FOVxz = FOV[0] * 0.00872664625997;	// (pi/180)/2 =  0.00872664625997
    half_FOVyz = FOV[1] * 0.00872664625997;
    cm_fovxz = 1 / tan(half_FOVxz);
}

void RS_Camera::get_rotate_matrix()
{
    float determinant = 0;
    float C[3][3] = { {cos(camera_pose[4])*cos(camera_pose[5]), cos(camera_pose[4])*sin(camera_pose[5]), -sin(camera_pose[4])},
    {(-cos(camera_pose[3])*sin(camera_pose[5]) + sin(camera_pose[3])*sin(camera_pose[4])*cos(camera_pose[5])), cos(camera_pose[3])*cos(camera_pose[5]) + sin(camera_pose[3])*sin(camera_pose[4])*sin(camera_pose[5]), sin(camera_pose[3])*cos(camera_pose[4])},
    {sin(camera_pose[3])*sin(camera_pose[5]) + cos(camera_pose[3])*sin(camera_pose[4])*cos(camera_pose[5]), (-sin(camera_pose[3])*cos(camera_pose[5]) + cos(camera_pose[3])*sin(camera_pose[4])*sin(camera_pose[5])), cos(camera_pose[3])*cos(camera_pose[4])} };

    // Inverse Rotation matrix
    for (int i = 0; i < 3; i++)
        determinant = determinant + (C[0][i] * (C[1][(i + 1) % 3] * C[2][(i + 2) % 3] - C[1][(i + 2) % 3] * C[2][(i + 1) % 3]));
    for (int mi = 0; mi < 3; mi++) {
        for (int mj = 0; mj < 3; mj++)
            inv_C[mi][mj] = ((C[(mj + 1) % 3][(mi + 1) % 3] * C[(mj + 2) % 3][(mi + 2) % 3]) - (C[(mj + 1) % 3][(mi + 2) % 3] * C[(mj + 2) % 3][(mi + 1) % 3])) / determinant;
    }
    //std::cout << "inv_C[1][1]:" << inv_C[1][1] << std::endl;
}

void RS_Camera::stop()
{
    std::cout << "Stop RS_Camera Streaming" << std::endl;
    camera_stream = false;
    if(streamThread.joinable()){
        streamThread.join();
        std::cout << "rs_camera streamThread released" << std::endl;
    }
    //delete pc_vertices;
}

float3 RS_Camera::Quat2Euler()
{
    float3 angle;
	//3-2-1
	double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	angle.x = atan2(sinr_cosp, cosr_cosp);	//yaw
	double sinp = 2.0 * (q.w * q.y - q.z * q.x);
	angle.y = asin(sinp);	//pitch
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	angle.z = atan2(siny_cosp, cosy_cosp);	//roll
    return angle;
}