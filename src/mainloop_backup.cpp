for (int j = 0; j < height; ++j) {
			//#pragma omp for 
			for (int i = 0; i < width; ++i)
			{
				darray[i][j] = d.get_distance(i, j);
				if (darray[i][j] > max_distance) {
					darray[i][j] = 0;
				}
				//sub_darray assign
				si = i / reduced_ratio;
				sj = j / reduced_ratio;
				sn = i % reduced_ratio + (j % reduced_ratio) * reduced_ratio;
				sub_darray[si][sj][sn] = darray[i][j];
			}
		}




//
//#pragma omp parallel for private(coarray,specific_point) num_threads(6)
		for (int j = 0; j < reduced_h; ++j) {
			//#pragma omp parallel for					//enable when reduce_w is big
			for (int i = 0; i < reduced_w; ++i) {
				//derive the min from each sub_darray
				double min = 10;
				int rn = 0;
				for (int n = 0; n < reduced_ratio_square; ++n) {
					if (sub_darray[i][j][n] < min && sub_darray[i][j][n] != 0) {
						min = sub_darray[i][j][n];
						rn = n;
					}
				}
				if (min >= 10) {
					min = 0;
					continue;
				}

				//std::cout << "min(" << i << "," << j << ")=" << min << std::endl;
				ri = (rn % reduced_ratio) + i * reduced_ratio;	//
				rj = (rn / reduced_ratio) + j * reduced_ratio;	//

				//organize 3 dimension coordinate in real world X Y Z, [camera frame] (meter)
				float specific_point[3];
				const float specific_pixel[2] = { (float)ri, (float)rj };
				rs2_deproject_pixel_to_point(specific_point, &intr, specific_pixel, min);
				//leave if there's no depth data
				if (specific_point[2] == 0)
					continue;

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
				v[0] = camera_global_pose[0] + coarray[0];
				v[1] = camera_global_pose[1] + coarray[1];
				v[2] = camera_global_pose[2] + coarray[2];

				/*******************************************/
				/**********Occupancy Indicator(rect)********/
				/*******************************************/

				/* Free Cell */
				int r[3];						//ray 
				float r_length;					//unit ray_length
				int length_factor;				//use to extend r_length
				float dx, dy, dz;				//unit ray x - vector
				float r_distance = sqrt(pow(coarray[0], 2) + pow(coarray[1], 2) + pow(coarray[2], 2));	//ray_distance

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
				if (dz >= 0 || dy >= 0)
					length_factor = length_factor - 1;

				//draw occupancy rectangle
				for (int k = 0; k < length_factor; k++) {
					//voxel map origin(pixel)
					r[0] = camera_global_pose[0] + dx * k;
					r[1] = camera_global_pose[1] + dy * k;
					r[2] = camera_global_pose[2] + dz * k;
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

					//ray detection
					//logodd mapping!!
					render_vmap(false, ray_cube, update_data, r_length, k, angle45_xz, angle45_zy);
				}

				//3D object detect
				int obj_cube[3];
				obj_cube[0] = v[0] / unit_length_x;
				obj_cube[1] = v[1] / unit_length_y;
				obj_cube[2] = v[2] / unit_length_z;
				if (obj_cube[0] >= grid_x || obj_cube[1] >= grid_y || obj_cube[2] >= grid_z || obj_cube[0] < 0 || obj_cube[1] < 0 || obj_cube[2] < 0
					|| obj_cube[0] - 1 < 0 || obj_cube[1] - 1 < 0 || obj_cube[2] - 1 < 0)
					continue;
				//logodd mapping!!
				render_vmap(true, obj_cube, update_data, r_distance, 1, false, false);
			}
		}
