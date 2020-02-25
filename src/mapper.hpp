#ifndef MAPPER_H
#define MAPPER_H

#include "sensor.hpp"

class Mapper
{
public:
    Mapper(int w, int h, int fps);   //call Sensor realsense();
    
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

    // para
    float4 quat;
    float camera_pose[9] = { 0 };
    float camera_scaled_pose[3] = { 0 };
    float inv_C[3][3] = { 0 };

    // func
    void start(bool &mapper_status, char (*map)[30][100]);//while loop
    void ray_tracing(const rs2::vertex vertices[], float specific_point[]);
    void render();
    void reduce_resolution(rs2::points points, const rs2::vertex vertices[]);
    void rotate();
    void show_2d();

    

    // depth

    // pointcloud vertices

    // pose

    //--------------------
    // Voxel Map Dimension
    //--------------------
    //Voxel map size
    const float block_unit_m = 0.2;	//(meter/cell)
    const int unit_length_x = 5;	//(pixel/cell)
    const int unit_length_y = 5;	//(pixel/cell)
    const int unit_length_z = 5;	//(pixel/cell)

    //Map scale in real world (pixel per meter), can use to transfer map from real world to voxel world
    const float mapscale_x = unit_length_x / block_unit_m;	//
    const float mapscale_y = unit_length_y / block_unit_m;
    const float mapscale_z = unit_length_z / block_unit_m;	//
    
    // Reduce resolution
    static const int reduced_ratio = 10;	//1,2,4,8,10,16,20
    static const int reduced_w = 848 / reduced_ratio;
    static const int reduced_h = 480 / reduced_ratio;
    static const int reduced_res = reduced_w * reduced_h;
    static const int reduced_ratio_square = reduced_ratio * reduced_ratio;
    double sub_darray[reduced_w][reduced_h][reduced_ratio_square];
    float3 reduced_vertices[reduced_res];
    int si, sj, sn, ri, rj;

    // render para
    double coarray[3];	//camera coordinate
    
    ~Mapper();
private:
    const int width, height, resolution, framerate;
    unsigned int reduced_ratio = 10;
    int specific_row = 15;
};





#endif //MAPPER_H