#include "rs_camera.h"

RS_Camera::RS_Camera()
{
    int width = 640, height = 480, framerate = 30;
    
    pc_vertices = new float3[1000000];

    stream_status = false;

}
void RS_Camera::init(int w, int h, int fps)
{
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
		const char* dev_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

		rs2::pipeline pipe(ctx);

		// Setup configuration
		rs2::config cfg;
		cfg.enable_device(dev_num);

		// D435 & D435i
		if (strcmp("843112070567", dev_num) == 0 || strcmp("827112071112", dev_num) == 0) {
			std::cout << "Camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected" << std::endl;
			cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, framerate);
			cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, framerate);
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
    
    // Filter config
    deci_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	thhd_filter.set_option(RS2_OPTION_MAX_DISTANCE, 10); //max depth range
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
}

void RS_Camera::start()
{
    if(!stream_status){
        std::cout << "Starting RealSense Streaming Thread" << std::endl;
        streamThread = std::thread(&RS_Camera::stream, this);
    }
}

void RS_Camera::stream()
{
    if(stream_status)
    {
        std::cout << "stream thread is running" << std::endl;
    }
    else
    {
        stream_status = true;
        
        while(stream_status)
        {
            read_depth();
            read_pose();
            std::cout << "Shifted Postion(X,Y,Z): " << std::setprecision(5) << std::fixed <<
			camera_pose[0] << " " << camera_pose[1] << " " << camera_pose[2] << " " << std::endl;
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
    camera_pose[0] = init_camera_global_pos[0] + camera_state[0];
    camera_pose[1] = init_camera_global_pos[1] + camera_state[1];
    camera_pose[2] = init_camera_global_pos[2] + camera_state[2];
    camera_pose[3] = 0 + camera_state[3];
    camera_pose[4] = 0 + camera_state[4];
    camera_pose[5] = 0 + camera_state[5];
    camera_pose[6] = camera_state[0];
    camera_pose[7] = camera_state[1];
    camera_pose[8] = camera_state[2];
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
}
void RS_Camera::stop()
{
    std::cout << "Stop Streaming" << std::endl;
    if(streamThread.joinable()){
        streamThread.join();
        std::cout << "rs_camera streamThread released" << std::endl;
    }
    delete pc_vertices;
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