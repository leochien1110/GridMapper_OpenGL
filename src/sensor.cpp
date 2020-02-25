#include "sensor.hpp"

Sensor::Sensor(int w, int h, int fps)
{
    printf("Sensor called!\n");
    width = w;
    height = h;
    resolution = w * h;
    framerate = fps;
}

void Sensor::init()
{
    filter();
    pipline();
}

void Sensor::capture()
{
    // D435 Stream
    data = pipelines[0].wait_for_frames();	//Wait for next set of frames from the camera
    color_frame = data.get_color_frame();		//Get color frame
    depth = data.get_depth_frame();	//Visualize 2D depth image
    depth = thhd_filter.process(depth);
    get_fov();
    //point cloud
    pc.map_to(color_frame);
    points = pc.calculate(depth);						// heavy function :(
    auto vertices = points.get_vertices();              // get vertices
    //auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
    //rs2::frame d = data.get_depth_frame().apply_filter(color_map);
    depth = depth.apply_filter(color_map);

    /* T265 Data */
    
    auto frames = pipelines[1].wait_for_frames();
    if( rs2::pose_frame f = frames.first_or_default(RS2_STREAM_POSE) ) {
        //auto pose_data = f.get_pose_data();	// Get states data(x,y,z,qw,qx,qy,qz)

        //auto f = frames.first_or_default(RS2_STREAM_POSE);
        //auto pose_data = f.as<rs2::pose_frame>().get_pose_data();	// Get states data(x,y,z,qw,qx,qy,qz)
        auto pose_data = f.get_pose_data();	// Get states data(x,y,z,qw,qx,qy,qz)
        // Quaternion(w,x,y,z)
        quat.w = pose_data.rotation.w;
        quat.x = -pose_data.rotation.z;
        quat.y = pose_data.rotation.x;
        quat.z = -pose_data.rotation.y;

        // Conver quaternion to euler angle
        angle = Quat2Euler(quat);
        // Local Position
        camera_state[0] = pose_data.translation.x;	//X
        camera_state[1] = -pose_data.translation.y;	//Y
        camera_state[2] = -pose_data.translation.z;	//Z
        //convert to y-z-x order
        camera_state[3] = angle.y;					//camera_pose[3]
        camera_state[4] = angle.z;					//camera_pose[4]
        camera_state[5] = angle.x;					//camera_pose[5]
        //x-z-y
        //camera_state[3] = angle.x;					//camera_pose[3]
        //camera_state[4] = angle.z;					//camera_pose[4]
        //camera_state[5] = angle.y;					//camera_pose[5]
        //y-x-z
        //camera_state[3] = angle.y;					//camera_pose[3]
        //camera_state[4] = angle.x;					//camera_pose[4]
        //camera_state[5] = angle.z;					//camera_pose[5]
        camera_pose[0] = init_camera_global_pos[0] + camera_state[0] + map_shift[0];
        camera_pose[1] = init_camera_global_pos[1] + camera_state[1] + map_shift[1];
        camera_pose[2] = init_camera_global_pos[2] + camera_state[2] + map_shift[2];
        camera_pose[3] = 0 + camera_state[3];
        camera_pose[4] = 0 + camera_state[4];
        camera_pose[5] = 0 + camera_state[5];
        camera_pose[6] = camera_state[0];
        camera_pose[7] = camera_state[1];
        camera_pose[8] = camera_state[2];
    }
}

void Sensor::pipline()
{
    //std::vector<rs2::pipeline>  pipelines;

    // Check avaible cameras
	rs2::device_list devices = ctx.query_devices();
//	Sleep(1000);	//if couldn't connect to T265
	std::cout << "Found " << devices.size() << " devices " << std::endl;
	if (devices.size() != 2) {
		std::cout << "[WARNING]There's only one camera connected.\
		Please unplug USB and try again." << std::endl;
		exit;
	}

	// Setup Pipeline for each device
	for (auto&& dev : devices)
	{
		const char* dev_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

		rs2::pipeline pipe(ctx);

		// Setup configuration
		rs2::config cfg;
		cfg.enable_device(dev_num);

		// D435 & D435i
		if (strcmp("843112070567", dev_num) == 0 || strcmp("827112071112", dev_num) == 0) {
			std::cout << "Camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected" << std::endl;
			cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, framerate);
			cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, framerate);
		}
		// T265
		if (strcmp("905312110443", dev_num) == 0) {
            cfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 1);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 2);
			std::cout << "Camera: " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected" << std::endl;
		}

		pipe.start(cfg);
		pipelines.emplace_back(pipe);
	}
}

void Sensor::filter()
{
	// Configure filter parameters
	deci_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	thhd_filter.set_option(RS2_OPTION_MAX_DISTANCE, 10); //max depth range
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
}

void Sensor::get_fov()
{
    rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_fov(&intr, FOV);
    half_FOVxz = FOV[0] * 0.00872664625997;	// (pi/180)/2 =  0.00872664625997
    half_FOVyz = FOV[1] * 0.00872664625997;
    float cm_fovxz = 1 / tan(half_FOVxz);
}

Sensor::float4 Sensor::Quat2Euler(float4 q)
{
    //float Cq[3][3];
	//Cq[0][0] = pow(q.w, 2) + pow(q.x, 2) - pow(q.y, 2) - pow(q.z, 2);	Cq[0][1] = 2 * (q.x* q.y + q.w * q.z);								Cq[0][2] = 2 * (q.x* q.z - q.w * q.y);
	//Cq[1][0] = 2 * (q.w * q.z - q.x* q.y);								Cq[1][1] = -pow(q.w, 2) + pow(q.x, 2) - pow(q.y, 2) + pow(q.z, 2);	Cq[1][2] = 2 * (q.y* q.z + q.w * q.x);
	//Cq[2][0] = 2 * (q.x* q.z + q.w * q.y);								Cq[2][1] = 2 * (q.y* q.z - q.w * q.x);								Cq[2][2] = 1 - 2 * (pow(q.y, 2) + pow(q.z, 2))/*pow(q.w, 2) - pow(q.x, 2) - pow(q.y, 2) + pow(q.z, 2)*/;
	//2-3-1
	//angle.x = atan2(Cq[1][0], Cq[1][1]);	//camera_pose[3] attitude pitch
	//angle.y = asin(Cq[1][2]);				//camera_pose[4] heading yaw
	//angle.y = atan2(Cq[0][2], Cq[2][2]);	//camera_pose[5] bank roll
	//2-3-1
	//angle.x = asin(Cq[0][1]);	//camera_pose[3] attitude pitch
	//angle.y = atan2(-Cq[0][2], Cq[2][2]);	//camera_pose[4] heading yaw
	//angle.z = atan2(-Cq[1][0], 1 - 2 * (pow(q.x, 2) - pow(q.z, 2)));	//camera_pose[5] bank roll
	//3-2-1
	double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	angle.x = atan2(sinr_cosp, cosr_cosp);	//yaw
	double sinp = 2.0 * (q.w * q.y - q.z * q.x);
	angle.y = asin(sinp);	//pitch
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	angle.z = atan2(siny_cosp, cosy_cosp);	//roll

	//--------Reference----------
	//---------------------------
	//3-2-1
	//float camera_pose[3] = atan2(2 * (q.w*q.z + q.y * q.x), q.z ^ 2 - q.w ^ 2 - q.x ^ 2 + q.y ^ 2);
	//float camera_pose[4] = asin(2 * q.x*q.z - q.w * q.y);
	//float camera_pose[5] = atan2(2.*(q.w*q.x + q.y*q.z), (q.z^2+q.w^2-q.x^2-q.y^2));
	//1-2-3
	//float camera_pose[3] = atan2(2 * (q.y*q.z - q.w * q.x), q.z ^ 2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2);	//atan2(Cq[2][1],Cq[2][2])
	//float camera_pose[4] = asin(2 * (q.x*q.z + q.w * q.y));										//asin(Cq[2][0])
	//float camera_pose[5] = atan2(2.*(q.w*q.z - q.x * q.y), (q.z ^ 2 - q.w ^ 2 - q.x ^ 2 + q.y ^ 2));	//atan2(-Cq[1][0],-Cq[0][0])
	//2-3-1
	//float camera_pose[3] = atan2(2 * (q.w * q.z - q.y * q.x), q.z ^ 2 - q.w ^ 2 + q.x ^ 2 - q.y ^ 2);	//atan2(-Cq[1][0],-Cq[1][1])
	//float camera_pose[4] = asin(2 * (q.w * q.x + q.y * q.z));									//asin(Cq[1][2])
	//float camera_pose[5] = atan2(2*(q.x * q.z - q.w * q.y), (q.z^2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2));	//atan2(Cq[0][2],Cq[2][2])
    
    return angle;
}

void Sensor::get_rotation_matrix()
{
    float C[3][3] = { {cos(camera_pose[4])*cos(camera_pose[5]), cos(camera_pose[4])*sin(camera_pose[5]), -sin(camera_pose[4])},
    {(-cos(camera_pose[3])*sin(camera_pose[5]) + sin(camera_pose[3])*sin(camera_pose[4])*cos(camera_pose[5])), cos(camera_pose[3])*cos(camera_pose[5]) + sin(camera_pose[3])*sin(camera_pose[4])*sin(camera_pose[5]), sin(camera_pose[3])*cos(camera_pose[4])},
    {sin(camera_pose[3])*sin(camera_pose[5]) + cos(camera_pose[3])*sin(camera_pose[4])*cos(camera_pose[5]), (-sin(camera_pose[3])*cos(camera_pose[5]) + cos(camera_pose[3])*sin(camera_pose[4])*sin(camera_pose[5])), cos(camera_pose[3])*cos(camera_pose[4])} };
}

void Sensor::calc_transform(rs2_pose& pose_data, float mat[16])
{
	auto q = pose_data.rotation;
	auto t = pose_data.translation;
	// Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
	mat[0] = -(1 - 2 * q.y*q.y - 2 * q.z*q.z); mat[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat[8] = -(2 * q.x*q.z + 2 * q.y*q.w);      mat[12] = t.x;
	mat[1] = -(2 * q.x*q.y + 2 * q.z*q.w);     mat[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat[9] = -(2 * q.y*q.z - 2 * q.x*q.w);      mat[13] = t.y;
	mat[2] = -(2 * q.x*q.z - 2 * q.y*q.w);     mat[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat[10] = -(1 - 2 * q.x*q.x - 2 * q.y*q.y); mat[14] = t.z;
	mat[3] = 0.0f;                             mat[7] = 0.0f;                          mat[11] = 0.0f;                             mat[15] = 1.0f;
}