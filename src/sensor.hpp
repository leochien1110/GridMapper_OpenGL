#ifndef SENSOR_H
#define SENSOR_H

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API

class Sensor
{
public:
    Sensor(int w, int h, int fps);

    // sturcture
    struct float4 {
        float w, x, y, z;
        float4 operator*(float t);

        float4 operator-(float t);

        void operator*=(float t);

        void operator=(const float4 &other);

        void add(float t1, float t2, float t3, float t4);
    };

    struct float3 {
        float x, y, z;
        float3 operator*(float t);

        float3 operator-(float t);

        void operator*=(float t);

        void operator=(const float3 &other);

        void add(float t1, float t2, float t3);
    };

    //function
    void init();    //setup filter and pipeline
    void capture();
    void get_fov();
    void calc_transform(rs2_pose& pose_data, float mat[16]);
    float4 Quat2Euler(float4 q);
    void get_rotation_matrix();

    // parameter
    float init_camera_global_pos[3] = { 10 , 3 , 10 };
    float camera_state[6] = { 0 };
    float camera_pose[9] = { 0 };
    int map_shift[3] = { 0 };
    float4 quat;
    float4 angle;
    float FOV[2];
    float half_FOVxz, half_FOVyz;
    float C[3][3];  //rotation matrix
    float determinant = 0;

    // RealSense para
    const rs2::vertex vertices;
    rs2::points points;
    float specific_point[3];

    ~Sensor();    

private:
    //func
    void pipline();
    void filter();

    // para
    int width, height, resolution, framerate;

    // Create realsense context for managing devices
	rs2::context ctx;

    // Point cloud
	rs2::pointcloud pc;
	rs2::points points;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
	std::vector<rs2::pipeline>  pipelines;

    // Declare filters
	rs2::decimation_filter deci_filter;
	rs2::threshold_filter thhd_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;
	rs2::hole_filling_filter hole_filter;

    // Declare capture para
    // D435 Stream
    rs2::frameset data;
    rs2::frame color_frame;
    rs2::frame depth;
};



#endif //SENSOR_H