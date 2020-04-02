#ifndef RS_CAMERA_H
#define RS_CAMERA_H

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include <math.h>
#include <thread>
#include <mutex>
#include "data.h"   //strcut float3 & float4

class RS_Camera
{
public:
    RS_Camera();

    ~RS_Camera(){
        stop();
    }

    void init();
    void start();   // read data
    void stream();
    void read_depth();
    void read_pose();
    void fov(rs2::frame &);
    void get_rotate_matrix();
    void stop();
    float3 Quat2Euler();

    float3 *pc_vertices;
    rs2::points points;
    int points_size;
    //******************
    // Camera Parameters
    //******************
    // Initialize camera pose
    // global   :x,y,z
    // attitude :phi,theat,psi
    // local    :X,Y,Z
    // shift_map:sx,sy,sz
    float camera_pose[12] = { 0 };
    
    float init_camera_global_pos[3] = { 10, 3, 10};
    float specific_point[3]; //for FOV
    rs2_intrinsics intr;
    float FOV[2];
    float half_FOVxz;
    float half_FOVyz;
    float cm_fovxz;

    // rotation matrix
    float inv_C[3][3];

private:

    // int
    int width, height, framerate;
    
    // float 
    float camera_state[9] = { 0 };

    // pose
    float4 q;
    //float3 angle;

    // Class
    std::vector<rs2::pipeline>  pipelines;  //pipe for D435 and T265
    
    // Sensors
    rs2::sensor depth_sensor;
	rs2::sensor color_sensor;
	rs2::sensor pose_sensor;

    //rs2::config cfg;    // Setup configuration
    rs2::device_list devices;
	rs2::context   ctx;    // realsense context
	rs2::pointcloud pc;
	
	rs2::colorizer color_map;   //depth colorizer
    
	// Declare filters
	rs2::decimation_filter deci_filter;
	rs2::threshold_filter thhd_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;
	rs2::hole_filling_filter hole_filter;
    
    //Threading
    std::mutex mutex;
    std::thread streamThread;
    bool stream_status;
};

#endif //RS_CAMERA_H