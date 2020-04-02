#include "mapper.h"

Mapper::Mapper(float3 * vertices, int *points_size, float * _camera_pose, float * specific_pt, float _inv_C[3][3], int w, int h) 
{    
    std::cout << "Welcome to Mapper :)" << std::endl;
    //std::cout << "points addr: " << points << std::endl;

    // pointer to camera.camera_pose address
    camera_pose = _camera_pose;

    //pc_vertices = *vertices;
    pc_vertices = vertices;
    pc_points = points_size;
    std::cout << "*pc_points:" << *pc_points << std::endl;
    inv_C = _inv_C;
    
    specific_point = specific_pt;    
   
    /*pc_vertices = new float3[1000000];
    for (int i = 0; i < points.size(); i++)
    {
        pc_vertices[i].x = vertices[i].x;
        pc_vertices[i].y = vertices[i].y;
        pc_vertices[i].z = vertices[i].z;
    }*/
    
   
    /*for(int i = 0; i < 3; ++i)
    {
        specific_point[i] = specific_pt[i];
    }*/

    reduced_vertices = new float3[reduced_res];

    /*for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j)
            inv_C[i][j] = _inv_C[i][j];
    }*/

    //***************
    // Initialization
    //***************
    // voxelmap
	for (int i = 0; i < grid_x; i++) {
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				voxelmap[i][j][k] = 127;
				voxelmap_old[i][j][k] = 127;
				initial_voxelmap[i][j][k] = 0;
                //printf("voxelmap[i][j][k]: %u\n",  \
                    voxelmap[i][j][k]);
                
			}
		}
	}
}

Mapper::~Mapper()
{
    stop();
}
void Mapper ::start()
{
    if(!mapper_status){
        mapper_status = true;
        std::cout << "Starting Mapper Streaming Thread" << std::endl;
        streamThread = std::thread(&Mapper::stream, this);
    }
}

void Mapper::stream()
{
    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(2000));
    std::cout << "Start Mapping..." << std::endl;
    while(mapper_status)
    {
        //std::cout << camera_pose[0] << " "  \
            << camera_pose[1] << " "        \
            << camera_pose[2] << " "        \
            << *pc_points                 \
            << " " << std::endl;
        //std::cout << "pc_vertices:" << pc_vertices << std::endl;
        camera_scaled_pose[0] = camera_pose[0] * mapscale_x;
        camera_scaled_pose[1] = camera_pose[1] * mapscale_y;
        camera_scaled_pose[2] = camera_pose[2] * mapscale_z;
        
        //std::cout << camera_pose[0] << " "  \
            << camera_pose[1] << " "        \
            << camera_pose[2] << " "        \
            << camera_scaled_pose[0] << " "        \
            << camera_scaled_pose[1] << " "        \
            << camera_scaled_pose[2] << " "        \
            << *pc_points                 \
            << " " << std::endl;

        reduce_resolution();

        //----------------------------
		// Create a 3D Voxel Map Point
		//----------------------------
        //float v[3];	//voxel map coordinate
        ray_tracing();

        //std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(500));
    }
}

void Mapper::reduce_resolution()
{
    for (int i = 0; i < reduced_res; i++) {
        reduced_vertices[i].x = 10;
        reduced_vertices[i].y = 10;
        reduced_vertices[i].z = 10;
    }
    
    for (int i = 0; i < *pc_points; i++)
    {
        if(pc_vertices[i].z){
            int ri = 0;
            si = i / reduced_ratio % reduced_w;
            sj = i / reduced_ratio / width;
            ri = si + sj * reduced_w; 
            if(pc_vertices[i].z < reduced_vertices[ri].z){
                reduced_vertices[ri].x = pc_vertices[i].x;
                reduced_vertices[ri].y = pc_vertices[i].y;
                reduced_vertices[ri].z = pc_vertices[i].z;
                //printf("reduced_vertices[ri].x:%f\n",reduced_vertices[ri].x);
            }
        }
        
    }
}

void Mapper::ray_tracing()
{
    float v[3];
    //printf("reduced_res:%d\n",reduced_res);
    for (int i = 0; i < reduced_res; i++)
    {
        if (pc_vertices[i].z)
        {
            specific_point[0] = reduced_vertices[i].x;
            specific_point[1] = reduced_vertices[i].y;
            specific_point[2] = reduced_vertices[i].z;

            /* Rotation */
            //position relative to camera, real world(meter)
            coarray[0] = 0;
            coarray[1] = 0;
            coarray[2] = 0;

            //rotation correction [camera frame] (meter), pt1 = invC * pt1', for loop: matrix multiply
            for (int l = 0; l < 3; l++) {
                for (int m = 0; m < 3; m++) {
                    coarray[l] += inv_C[l][m] * specific_point[m];
                }
            }

            /* Conversion */
            //convert position unit [camera frame] (meter → pixel)
            coarray[0] *= mapscale_x;
            coarray[1] *= mapscale_y;
            coarray[2] *= mapscale_z;


            //convert to [voxel frame] (pixel): point clouds position
            v[0] = camera_scaled_pose[0] + coarray[0];
            v[1] = camera_scaled_pose[1] + coarray[1];
            v[2] = camera_scaled_pose[2] + coarray[2];
            
            /*******************************************/
            /**********Occupancy Indicator(rect)********/
            /*******************************************/
            // Ray traversal
            int r[3];						//ray 
            float r_length;					//unit ray_length
            int length_factor;				//use to extend r_length
            float dx, dy, dz;				//unit ray x - vector
            float r_distance = sqrt(pow(coarray[0], 2) + pow(coarray[1], 2) + pow(coarray[2], 2));	//ray_distance

            /* Occupied Cell */
            int obj_cube[3];
            obj_cube[0] = v[0] / unit_length_x;
            obj_cube[1] = v[1] / unit_length_y;
            obj_cube[2] = v[2] / unit_length_z;


            if (obj_cube[0] >= grid_x || obj_cube[1] >= grid_y || obj_cube[2] >= grid_z || obj_cube[0] < 0 || obj_cube[1] < 0 || obj_cube[2] < 0
                || obj_cube[0] - 1 < 0 || obj_cube[1] - 1 < 0 || obj_cube[2] - 1 < 0)
                continue;
            //mapping!!
            render_vmap(true, obj_cube, r_distance, 1, false, false);

            //Ray property
            double c_m_xz = 1;
            double c_m_zy = 1;
            double c_m_xy = 1;
            bool angle45_xz = true;
            bool angle45_zy = true;
            bool angle45_xy = true;

            //Find max coarray axes
            double max = 0;
            int c_index = 0;
            for (int n = 0; n < 3; ++n) {
                if (abs(coarray[n]) > max) {
                    max = abs(coarray[n]);
                    c_index = n;
                }
            }

            //Make sure that the ray direction is correct
            int unit_length_dir = unit_length_x;
            if (coarray[c_index] < 0)
                unit_length_dir *= -1;
            int ray_case = 0;
            if (coarray[0] == 0 && coarray[1] == 0 && coarray[2] != 0) {			//case1 (0,0,1)
                dx = 0;
                dy = 0;
                dz = unit_length_dir;
                ray_case = 1;
            }
            else if (coarray[0] != 0 && coarray[1] == 0 && coarray[2] == 0) {		//case2 (1,0,0)
                dx = unit_length_dir;
                dy = 0;
                dz = 0;
                ray_case = 2;
            }
            else if (coarray[0] == 0 && coarray[1] != 0 && coarray[2] == 0) {		//case3 (0,1,0)
                dx = 0;
                dy = unit_length_dir;
                dz = 0;
                ray_case = 3;
            }
            else if (coarray[0] == 0 && coarray[1] != 0 && coarray[2] != 0) {		//case4 (0,1,1)
                c_m_zy = coarray[1] / coarray[2];
                if (c_index == 1)
                {
                    dx = 0;
                    dy = unit_length_dir;
                    dz = dy / c_m_zy;
                }
                else if (c_index == 2)
                {
                    dz = unit_length_dir;
                    dx = 0;
                    dy = dz * c_m_zy;
                }
                ray_case = 4;
            }
            else if (coarray[0] != 0 && coarray[1] == 0 && coarray[2] != 0) {		//case5 (1,0,1)
                c_m_xz = coarray[2] / coarray[0];
                if (c_index == 0) {
                    dx = unit_length_dir;
                    dz = dx * c_m_xz;
                    dy = 0;
                }
                else if (c_index == 2)
                {
                    dz = unit_length_dir;
                    dx = dz / c_m_xz;
                    dy = 0;
                }
                ray_case = 5;
            }
            else if (coarray[0] != 0 && coarray[1] != 0 && coarray[2] == 0) {		//case6 (1,1,0)
                c_m_xy = coarray[1] / coarray[0];
                if (c_index == 0) {
                    dx = unit_length_dir;
                    dz = 0;
                    dy = dx * c_m_xy;
                }
                else if (c_index == 1) {
                    dy = unit_length_dir;
                    dz = 0;
                    dx = dy / c_m_xy;
                }
                ray_case = 6;
            }
            else																	//Normal Cases(1,1,1)
            {
                c_m_xz = coarray[2] / coarray[0];		//x-z plane, m = dz/dx;
                c_m_zy = coarray[1] / coarray[2];		//z-y plane,
                //c_m_xy = coarray[1] / coarray[0];		//x-z plane

                /* Check if angle > 45 */
                //x-z plane
                if (abs(c_m_xz) >= 1)	angle45_xz = true;
                else if (abs(c_m_xz) < 1)	angle45_xz = false;

                //z-y plane
                if (abs(c_m_zy) >= 1)	angle45_zy = true;
                if (abs(c_m_zy) < 1)	angle45_zy = false;

                //x-y plane
                //if (abs(c_m_xy) >= 1)	angle45_xy = true;
                //if (abs(c_m_xy) < 1)	angle45_xy = false;
                //if (abs(c_m_xy) < 1)	angle45_xy = false;

                //Assign Maximum axis as unit_length
                if (c_index == 0) {
                    dx = unit_length_dir;
                    dz = dx * c_m_xz;
                    dy = dz * c_m_zy;
                }
                else if (c_index == 1) {
                    dy = unit_length_dir;
                    dz = dy / c_m_zy;
                    dx = dz / c_m_xz;
                }
                else if (c_index == 2) {
                    dz = unit_length_dir;
                    dx = dz / c_m_xz;
                    dy = dz * c_m_zy;
                }
            }
            //std::cout << "dx,dy,dz=" << dx << "," << dy << "," << dz << std::endl;

            //compare unit ray vector length and original obj distance
            r_length = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
            length_factor = r_distance / r_length;

            //make sure free cell won't occupy obstacle cell
            //if (dz >= 0 || dy >= 0)
            //	length_factor = length_factor - 1;

            /* Free Cell */
            for (int k = 0; k < length_factor; k++) {
                //voxel map origin(pixel)
                r[0] = camera_scaled_pose[0] + dx * k;
                r[1] = camera_scaled_pose[1] + dy * k;
                r[2] = camera_scaled_pose[2] + dz * k;
                //std::cout << "r[0]:" << r[0] << "  r[1]:" << r[1] << "  r[2]:" << r[2] << std::endl;
                //*3D* find parameter of cube
                int ray_cube[3];
                ray_cube[0] = r[0] / unit_length_x;
                ray_cube[1] = r[1] / unit_length_y;
                ray_cube[2] = r[2] / unit_length_z;

                //check if cell is beyond boundary (for fix boundary only)
                if (ray_cube[0] >= grid_x || ray_cube[1] >= grid_y || ray_cube[2] >= grid_z || ray_cube[0] < 0 || ray_cube[1] < 0 || ray_cube[2] < 0
                    || ray_cube[0] - 1 < 0 || ray_cube[1] - 1 < 0 || ray_cube[2] - 1 < 0)
                    continue;
                
                // Keep ray away from obstacle cells
                if(ray_cube[0] == obj_cube[0] && ray_cube[1] == obj_cube[1] && ray_cube[2] == obj_cube[2])
                    continue;
                //free cell mapping!!
                render_vmap(false, ray_cube, r_length, k, angle45_xz, angle45_zy);
            }
        }
    }
}

void Mapper::render_vmap(bool occupied, int cube[], float r_distance, int k, bool angle45_xz, bool angle45_zy)
{
    float K;
	//std::lock_guard<std::mutex> mlock(mutex_t_m2g);
    //std::cout << cube[0] << " "  \
        << cube[1] << " "        \
        << cube[2] << " "        \
        << std::endl;
    //printf("voxelmap[cube[0]][cube[1]][cube[2]]: %c\n",  \
        voxelmap[cube[0]][cube[1]][cube[2]]);
    // occupied cell
	if (voxelmap[cube[0]][cube[1]][cube[2]] <= 255 && occupied == true) {
		K = 64 * k  * r_distance / (width / 2);

		voxelmap[cube[0]][cube[1]][cube[2]] =
			(voxelmap_old[cube[0]][cube[1]][cube[2]] * (255 - K) + (255) * K) / 255;
		if (voxelmap[cube[0]][cube[1]][cube[2]] > 255)
			voxelmap[cube[0]][cube[1]][cube[2]] = 255;

		initial_voxelmap[cube[0]][cube[1]][cube[2]] = 1;
		voxelmap_old[cube[0]][cube[1]][cube[2]] = voxelmap[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}
	//false = free cell
	else if (voxelmap[cube[0]][cube[1]][cube[2]] > 0 && occupied == false) {
		K = 16 * k  * r_distance / (width / 2);
		voxelmap[cube[0]][cube[1]][cube[2]] =
			(voxelmap_old[cube[0]][cube[1]][cube[2]] * (255 - K) + 1 * K) / 255;

		if (voxelmap[cube[0]][cube[1]][cube[2]] < 0.1)
			voxelmap[cube[0]][cube[1]][cube[2]] = 0;

		initial_voxelmap[cube[0]][cube[1]][cube[2]] = 1;
		voxelmap_old[cube[0]][cube[1]][cube[2]] = voxelmap[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}
}
void Mapper::shift_map(int axis)
{
    int shift_unit = mps * unit_length_x;
	switch (axis)
	{
	case 0:		//x-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
			printf("shift_unit(x) > 0");
			// Shift Maps
			for (int i = grid_x - 1; i > shift_unit - 1; i--) {	//i = 99~5
				for (int j = 0; j < grid_y; j++) {	// j = 29~5
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = voxelmap[i - shift_unit][j][k];
						voxelmap_old[i][j][k] = voxelmap_old[i - shift_unit][j][k];
						initial_voxelmap[i][j][k] = initial_voxelmap[i - shift_unit][j][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = shift_unit - 1; i >= 0; i--) {	//i = 4~0
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}

		}
		// For Upper Limit
		else	// shift_unit <= 0
		{
			printf("shift_unit(x) < 0");
			// For Lower Limit
			// Shift Maps
			for (int i = 0; i < grid_x - abs(shift_unit); i++) {	//i = 0~94
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = voxelmap[i - shift_unit][j][k];
						voxelmap_old[i][j][k] = voxelmap_old[i - shift_unit][j][k];
						initial_voxelmap[i][j][k] = initial_voxelmap[i - shift_unit][j][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = grid_x - abs(shift_unit); i < grid_x; i++) {	// i = 95~99
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}
		}
		break;
	case 1:		//y-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
			printf("shift_unit(y) > 0");
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = grid_y - 1; j > shift_unit - 1; j--) {	// j = 29~5
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = voxelmap[i][j - shift_unit][k];
						voxelmap_old[i][j][k] = voxelmap_old[i][j - shift_unit][k];
						initial_voxelmap[i][j][k] = initial_voxelmap[i][j - shift_unit][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = shift_unit - 1; j >= 0; j--) {	// j = 0~4
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}

		}

		// For Upper Limit
		else	// shift_unit <= 0
		{
//			printf("shift_unit(y) < 0");
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y - abs(shift_unit); j++) {	// j = 0~24
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = voxelmap[i][j - shift_unit][k];
						voxelmap_old[i][j][k] = voxelmap_old[i][j - shift_unit][k];
						initial_voxelmap[i][j][k] = initial_voxelmap[i][j - shift_unit][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = grid_y - abs(shift_unit); j < grid_y; j++) {	// j = 25~29
					for (int k = 0; k < grid_z; k++) {
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}
		}	
		break;
	case 2:		//z-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
//			printf("shift_unit(z) > 0");
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = grid_z - 1; k > shift_unit - 1; k--) {	//k = 99~5
						voxelmap[i][j][k] = voxelmap[i][j][k - shift_unit];
						voxelmap_old[i][j][k] = voxelmap_old[i][j][k - shift_unit];
						initial_voxelmap[i][j][k] = initial_voxelmap[i][j][k - shift_unit];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = shift_unit - 1; k >= 0; k--) {	//k = 4~0
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}

		}

		// For Upper Limit
		else	// shift_unit <= 0
		{
//			printf("shift_unit(z) < 0");
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z - abs(shift_unit); k++) {	//k = 0~94
						voxelmap[i][j][k] = voxelmap[i][j][k - shift_unit];
						voxelmap_old[i][j][k] = voxelmap_old[i][j][k - shift_unit];
						initial_voxelmap[i][j][k] = initial_voxelmap[i][j][k - shift_unit];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j =0; j < grid_y; j++) {
					for (int k = grid_z - abs(shift_unit); k < grid_z; k++) {	//k = 95~99
						voxelmap[i][j][k] = 127;
						voxelmap_old[i][j][k] = 127;
						initial_voxelmap[i][j][k] = 0;
					}
				}
			}
		}
		break;	
	}
}

void Mapper::stop()
{
    std::cout << "Stop Mapper streaming" << std::endl;
    if(streamThread.joinable()){
        streamThread.join();
        std::cout << "mapper streamThread released" << std::endl;
    }
    delete pc_vertices;
    delete reduced_vertices;
}