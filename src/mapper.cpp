#include "mapper.h"

Mapper::Mapper() 
{    
    std::cout << "Welcome to Mapper :)" << std::endl;
    //std::cout << "points addr: " << points << std::endl;

    // pointer to camera.camera_pose address
    //camera_pose = _camera_pose;

    //pc_vertices = *vertices;
    //pc_vertices = vertices;
    //points_size = points_size;
    std::cout << "points_size:" << points_size << std::endl;
    //inv_C = _inv_C;
    
    //pc_localpos = specific_pt;    
   
    /*pc_vertices = new float3[1000000];
    for (int i = 0; i < points.size(); i++)
    {
        pc_vertices[i].x = vertices[i].x;
        pc_vertices[i].y = vertices[i].y;
        pc_vertices[i].z = vertices[i].z;
    }*/
    
   
    /*for(int i = 0; i < 3; ++i)
    {
        pc_localpos[i] = specific_pt[i];
    }*/

    reduced_vertices = new float3[reduced_res];

    /*for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j)
            inv_C[i][j] = _inv_C[i][j];
    }*/

    //***************
    // Initialization
    //***************
    // voxel_map_logodd
	for (int i = 0; i < grid_x; i++) {
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				voxel_map_logodd[i][j][k] = 127;
				voxel_map_logodd_old[i][j][k] = 127;
				initial_voxel_map[i][j][k] = 0;
                //printf("voxel_map_logodd[i][j][k]: %u\n",  \
                    voxel_map_logodd[i][j][k]);
                
			}
		}
	}
}

Mapper::~Mapper()
{
    savelog();
    stop();
}
void Mapper ::start()
{
    if(!mapper_stream){
        mapper_stream = true;
        std::cout << "Starting Mapper Streaming Thread" << std::endl;
        streamThread = std::thread(&Mapper::stream, this);
    }
}

void Mapper::stream()
{
    std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(2000));
    std::cout << "Start Mapping..." << std::endl;
    while(mapper_stream)
    {
        //std::cout << '\r' << std::setiosflags(std::ios::fixed)  \
            << std::setprecision(3)                     \
            << std::setw(2) << camera_pose[0] << " "                    \
            << std::setw(2) << camera_pose[1] << " "                    \
            << std::setw(2) << camera_pose[2] << " "                    \
            << std::setw(2) << points_size << " "                       \
            << std::flush;
        //std::cout << "pc_vertices:" << pc_vertices << std::endl;
        camera_scaled_pose[0] = camera_global_pose[0] * mapscale_x;
        camera_scaled_pose[1] = camera_global_pose[1] * mapscale_y;
        camera_scaled_pose[2] = camera_global_pose[2] * mapscale_z;

        reduce_resolution();

        //----------------------------
		// Create a 3D Voxel Map Point
		//----------------------------
        //float v[3];	//voxel map coordinate
        map_update();

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
    
    for (int i = 0; i < points_size; i++)
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

double Mapper::exp_fast(double x)
{
  if(x<-500){x = 0;    return x;}
  x = 1.0 + x/256;  x *= x;  x *= x;  x *= x;
  x *= x;  x *= x;  x *= x;  x *= x;  x *= x;
  return x;
}

void Mapper::map_update()
{
    camera_pixel_pose[0] = camera_global_pose[0] * mapscale_x;	//(pixel)
    camera_pixel_pose[1] = camera_global_pose[1] * mapscale_y;
    camera_pixel_pose[2] = camera_global_pose[2] * mapscale_z;
    
    //printf("reduced_res:%d\n",reduced_res);
    for (int i = 0; i < reduced_res; i++)
    {
        if (pc_vertices[i].z)
        {
            pc_localpos[0] = reduced_vertices[i].x;
            pc_localpos[1] = reduced_vertices[i].y;
            pc_localpos[2] = reduced_vertices[i].z;

            //pointcloud position relative to camera, real world (meter)
            float pc_position[3];
            pc_position[0] = 0;
            pc_position[1] = 0;
            pc_position[2] = 0;

            /* Rotation */
            //rotation correction [camera frame] (meter), pt1 = invC * pt1', for loop: matrix multiply
            for (int l = 0; l < 3; l++) {
                for (int m = 0; m < 3; m++) {
                    pc_position[l] += inv_C[l][m] * pc_localpos[m];
                }
            }

            /* Unit Conversion */
            //convert pc position to [local frame] (meter → pixel)
            pc_position[0] *= mapscale_x;
            pc_position[1] *= mapscale_y;
            pc_position[2] *= mapscale_z;

            //convert pc position to [global frame] (pixel)
            float v[3];	//point cloud map coordinate
            v[0] = camera_pixel_pose[0] + pc_position[0];
            v[1] = camera_pixel_pose[1] + pc_position[1];
            v[2] = camera_pixel_pose[2] + pc_position[2];
            
            int r[3];						//ray 
            float r_length;					//ray length
            float r_unit_length;			//unit ray length
            int length_factor;				//use to extend r_length
            float dx, dy, dz;				//unit ray x - vector
            float obj_distance = sqrt(pow(pc_position[0], 2) + pow(pc_position[1], 2) + pow(pc_position[2], 2));	//ray_distance

            /* Occupied Cell */
            int obj_cube[3];	// pointcloud/object position
            int raydir[3];		// ray direction vector
            float ray_m[3];		// ray slope
            float ray_m_hypo = sqrt(raydir[0]*raydir[0]+raydir[1]*raydir[1]+raydir[2]*raydir[2]);	// ray hypotenuse

            obj_cube[0] = v[0] / unit_length_x;
            obj_cube[1] = v[1] / unit_length_y;
            obj_cube[2] = v[2] / unit_length_z;
            
            raydir[0] = v[0]-camera_pixel_pose[0];
            raydir[1] = v[1]-camera_pixel_pose[1];
            raydir[2] = v[2]-camera_pixel_pose[2];
            
            ray_m[0] = ray_m_hypo/raydir[0];
            ray_m[1] = ray_m_hypo/raydir[1];
            ray_m[2] = ray_m_hypo/raydir[2];
            
            /**************************/
            /***Occupied Cell Assign***/
            /**************************/
            if (obj_cube[0] >= grid_x || obj_cube[1] >= grid_y || obj_cube[2] >= grid_z || obj_cube[0] < 0 || obj_cube[1] < 0 || obj_cube[2] < 0
                || obj_cube[0] - 1 < 0 || obj_cube[1] - 1 < 0 || obj_cube[2] - 1 < 0)
                continue;

            render_vmap(true, obj_cube, obj_distance, pc_localpos[2], obj_distance);

            /*************************************/
            /***Ray Traversal (Ray Cell Assign)***/
            /*************************************/
#ifdef RT
            // Fast Voxel Ray Traversal Algorithm (Method 1)
            ray_traversal(raydir,ray_m,obj_cube,pc_localpos[2],obj_distance);
#endif //RT
#ifndef RT
            // Line algorithm (Method 2)
            line_drawing(pc_position,obj_distance,obj_cube,pc_localpos[2]);
#endif //RT
        }
    }
}

void Mapper::ray_traversal(int raydir[], float ray_m[], int dist[], float z_m, float obj_distance)
{
	int cam_cube[3];
	cam_cube[0] = camera_pixel_pose[0]/unit_length_x;
	cam_cube[1] = camera_pixel_pose[1]/unit_length_y;
	cam_cube[2] = camera_pixel_pose[2]/unit_length_z;

	// ray: start from camera position
	int ray_cube[3];
	ray_cube[0] = cam_cube[0];
	ray_cube[1] = cam_cube[1];
	ray_cube[2] = cam_cube[2];
	float r_length = 0;

	int cb[2];
	float tmax[3] = {0};
	float delta[3] = {0};
	int stepX, outX = 0;
	int stepY, outY = 0;
	int stepZ, outZ = 0;

	if (raydir[0] > 0)
	{
		stepX = 1;
		outX = grid_x;
		cb[0] = ray_cube[0] + 1;
	}
	else 
	{
		stepX = -1;
		outX = -1;
		cb[0] = ray_cube[0];
	}
	if (raydir[1] > 0.0f)
	{
		stepY = 1;
		outY = grid_y;
		cb[1] = ray_cube[1] + 1; 
	}
	else 
	{
		stepY = -1;
		outY = -1;
		cb[1] = ray_cube[1];
	}
	if (raydir[2] > 0.0f)
	{
		stepZ = 1;
		outZ = grid_z;
		cb[2] = ray_cube[2] + 1;
	}
	else 
	{
		stepZ = -1;
		outZ = -1;
		cb[2] = ray_cube[2];
	}

	if (raydir[0] != 0)
	{
		tmax[0] = (cb[0]*unit_length_x - camera_pixel_pose[0]) * ray_m[0]; 
		delta[0] = unit_length_x * stepX * ray_m[0];
	}
	else tmax[0] = 1000000;
	if (raydir[1] != 0)
	{
		tmax[1] = (cb[1]*unit_length_y - camera_pixel_pose[1]) * ray_m[1]; 
		delta[1] = unit_length_y * stepY * ray_m[1];
	}
	else tmax[1] = 1000000;
	if (raydir[2] != 0)
	{
		tmax[2] = (cb[2]*unit_length_z - camera_pixel_pose[2]) * ray_m[2];
		delta[2] = unit_length_z * stepZ * ray_m[2];
	}
	else tmax[2] = 1000000;
	// ray tracing loop
	while(ray_cube[0] != dist[0] && ray_cube[1] != dist[1] && ray_cube[2] != dist[2])
	{
		if (tmax[0] < tmax[1])
		{
			if (tmax[0] < tmax[2])
			{
				ray_cube[0] = ray_cube[0] + stepX;
				if (ray_cube[0] == outX) return;
				tmax[0] += delta[0];
				r_length = tmax[0];
			}
			else
			{
				ray_cube[2] = ray_cube[2] + stepZ;
				if (ray_cube[2] == outZ) return;
				tmax[2] += delta[2];
				r_length = tmax[2];
			}
		}
		else
		{
			if (tmax[1] < tmax[2])
			{
				ray_cube[1] = ray_cube[1] + stepY;
				if (ray_cube[1] == outY) return;
				tmax[1] += delta[1];
				r_length = tmax[1];
			}
			else
			{
				ray_cube[2] = ray_cube[2] + stepZ;
				if (ray_cube[2] == outZ) return;
				tmax[2] += delta[2];
				r_length = tmax[2];
			}
		}
		//float r_length = sqrt(abs(ray_cube[0] - cam_cube[0])*abs(ray_cube[0] - cam_cube[0])+
		//					  abs(ray_cube[1] - cam_cube[1])*abs(ray_cube[1] - cam_cube[1])+
		//					  abs(ray_cube[2] - cam_cube[2])*abs(ray_cube[2] - cam_cube[2]));
		if(r_length < obj_distance){render_vmap(false, ray_cube, r_length,z_m, obj_distance);}
		else{return;}
	}
}

void Mapper::line_drawing(float pc_position[], float ray_length, int dist[], float z_m)
{
	double c_m_xz = 1;
	double c_m_zy = 1;
	double c_m_xy = 1;
	bool angle45_xz = true;
	bool angle45_zy = true;
	bool angle45_xy = true;
	int r[3];						//ray 
	float r_length;					//ray length
	float r_unit_length;			//unit ray length
	int length_factor;				//use to extend r_length
	float dx, dy, dz;				//unit ray x - vector

	//Find max pc_position axes
	double max = 0;
	int c_index = 0;
	for (int n = 0; n < 3; ++n) {
		if (abs(pc_position[n]) > max) {
			max = abs(pc_position[n]);
			c_index = n;
		}
	}

	//Make sure that the ray direction is correct
	int unit_length_dir = unit_length_x;
	if (pc_position[c_index] < 0)
		unit_length_dir *= -1;
	int ray_case = 0;
	if (pc_position[0] == 0 && pc_position[1] == 0 && pc_position[2] != 0) {			//case1 (0,0,1)
		dx = 0;
		dy = 0;
		dz = unit_length_dir;
		ray_case = 1;
	}
	else if (pc_position[0] != 0 && pc_position[1] == 0 && pc_position[2] == 0) {		//case2 (1,0,0)
		dx = unit_length_dir;
		dy = 0;
		dz = 0;
		ray_case = 2;
	}
	else if (pc_position[0] == 0 && pc_position[1] != 0 && pc_position[2] == 0) {		//case3 (0,1,0)
		dx = 0;
		dy = unit_length_dir;
		dz = 0;
		ray_case = 3;
	}
	else if (pc_position[0] == 0 && pc_position[1] != 0 && pc_position[2] != 0) {		//case4 (0,1,1)
		c_m_zy = pc_position[1] / pc_position[2];
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
	else if (pc_position[0] != 0 && pc_position[1] == 0 && pc_position[2] != 0) {		//case5 (1,0,1)
		c_m_xz = pc_position[2] / pc_position[0];
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
	else if (pc_position[0] != 0 && pc_position[1] != 0 && pc_position[2] == 0) {		//case6 (1,1,0)
		c_m_xy = pc_position[1] / pc_position[0];
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
		c_m_xz = pc_position[2] / pc_position[0];		//x-z plane, m = dz/dx;
		c_m_zy = pc_position[1] / pc_position[2];		//z-y plane,
		//c_m_xy = pc_position[1] / pc_position[0];		//x-z plane

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

	//compare unit ray vector length and original obj distance
	r_unit_length = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
	length_factor = ray_length / r_unit_length;

	/* Free Cell */
	for (int k = 0; k < length_factor; k++) {
		r_length = r_unit_length * k;
		//voxel map origin(pixel)
		r[0] = camera_pixel_pose[0] + dx * k;
		r[1] = camera_pixel_pose[1] + dy * k;
		r[2] = camera_pixel_pose[2] + dz * k;
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
		if(ray_cube[0] == dist[0] && ray_cube[1] == dist[1] && ray_cube[2] == dist[2])
			continue;

		//free cell mapping!!
		render_vmap(false, ray_cube, r_length, z_m, ray_length);
	}
}

void Mapper::render_vmap(bool type, int cube[], float r_distance, float z_m, float obj_distance)
{
	float DW;			// distance weighting
	float logodds;	// depend on pdf
	float P_occ;
	float k = 0.07;			// inverse sensor model weighting
	
	r_distance /= mapscale_x;	//pixel->cube->meter
	obj_distance /= mapscale_x;
	
	if(r_distance > max_distance)
		r_distance = max_distance;
	else if(r_distance < min_distance)
		r_distance = min_distance;

	DW = r_distance/pixel_density;	// Distance Weighting

	float delta_z = z_m * z_m * sensor_para;
	float sigma = delta_z * r_distance / z_m;
	float Ps;

	// Inverse sensor model
	if (r_distance <= obj_distance){
		P_occ = p_min;
		Ps = P_occ + (k/sigma/sqrt(2*PI)+0.5-P_occ)*exp_fast(-0.5*(r_distance-obj_distance)*(r_distance-obj_distance)/(sigma*sigma));
	}
	else{
		P_occ = 0.5;
		Ps = P_occ;
	}
	
	if (Ps >= 1){
		Ps = 0.99999;
	}
	logodds = log(Ps/(1-Ps));
	
	//std::lock_guard<std::mutex> mlock(mutex_t_m2g);
	mutex.lock();
	// true = occupied cell
	if (voxel_map_logodd[cube[0]][cube[1]][cube[2]] < 255 && type == true) {
		if(ceil(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds) >= 255){
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] = 255;
		}
		else{
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] =
				ceil(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds);
		}

		initial_voxel_map[cube[0]][cube[1]][cube[2]] = 1;
		voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] = voxel_map_logodd[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}

	//false = free cell
	else if (voxel_map_logodd[cube[0]][cube[1]][cube[2]] > 0 && type == false) {
		if(floor(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds) <= 0){
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] = 0;
		}
		else{
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] =
				floor(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds);
		}

		initial_voxel_map[cube[0]][cube[1]][cube[2]] = 1;
		voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] = voxel_map_logodd[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}
	mutex.unlock();
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i - shift_unit][j][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i - shift_unit][j][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i - shift_unit][j][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = shift_unit - 1; i >= 0; i--) {	//i = 4~0
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i - shift_unit][j][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i - shift_unit][j][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i - shift_unit][j][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = grid_x - abs(shift_unit); i < grid_x; i++) {	// i = 95~99
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j - shift_unit][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j - shift_unit][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j - shift_unit][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = shift_unit - 1; j >= 0; j--) {	// j = 0~4
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j - shift_unit][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j - shift_unit][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j - shift_unit][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = grid_y - abs(shift_unit); j < grid_y; j++) {	// j = 25~29
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j][k - shift_unit];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j][k - shift_unit];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j][k - shift_unit];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = shift_unit - 1; k >= 0; k--) {	//k = 4~0
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j][k - shift_unit];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j][k - shift_unit];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j][k - shift_unit];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j =0; j < grid_y; j++) {
					for (int k = grid_z - abs(shift_unit); k < grid_z; k++) {	//k = 95~99
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
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
    //delete pc_vertices;
    delete reduced_vertices;
}

void Mapper::savelog()
{
    FILE *fp;
    fp = fopen(map_dir, "w");
    fprintf(fp, "");
    for (int k = 0; k < grid_z; k++) {
        for (int j = 0; j < grid_y; j++) {
            for (int i = 0; i < grid_x; i++) {
                fprintf(fp, "%i,", voxel_map_logodd[i][j][k]);
            }
        }
    }
    printf("Mapper closing... \nMap saved!\n");
}