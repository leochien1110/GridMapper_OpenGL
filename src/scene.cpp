#define STB_IMAGE_IMPLEMENTATION 
#include "scene.h"

glm::vec3 Scene::cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
glm::vec3 Scene::cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);

bool Scene::firstMouse = true;
float Scene::yaw = -90.0f; // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float Scene::pitch = -30.0f;
float Scene::lastX = SCR_WIDTH / 2.0;
float Scene::lastY = SCR_HEIGHT / 2.0;
float Scene::fov = 45.0f;
bool Scene::rotation_started = false;
bool Scene::viewpoint = true;
int Scene::start_x = 0;
int Scene::start_y = 0;

Scene::Scene()
{
    std::cout << "Scene called" << std::endl;

	// glfw: initialize and configure(version 3.3)
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Voxel 3D", NULL, NULL);
    if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit;
	}
	glInit(window);
	scene_stream = false;
	// Filed of view
	f1_2 = (unit_length_x / block_unit_m) * sqrt(pow(1, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) + 1));
	f1_0 = tan(half_FOVxz)*f1_2;
	f1_1 = tan(half_FOVyz)*f1_2;
	float f1[3] = { f1_0, f1_1, f1_2 };
	printf("Scene done!\n");
}

Scene::~Scene()
{
	stop();
	//delete window;
}

void Scene::processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	// moving mode
	float cameraSpeed = 1.0f; // camera moving speed
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS){
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
		printf("m pressed, Map saved!\n");
	}
	// Change Texture
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS){
		texture_index = 0;
	}
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS){
		texture_index = 1;
	}
	lightPos = cameraPos + glm::vec3(2.0f, 0.0f, -0.5f);
}

void Scene::mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
{
#ifdef GUI
	ImGuiIO &io = ImGui::GetIO();
    (void)io;
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !io.WantCaptureMouse) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
#else
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
#endif //GUI

	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		//std::cout << "Right Key Pressed!" << std::endl;
		//move viewpoint to realsense position
		if (viewpoint == false) {
			viewpoint = true;
		}
		else if (viewpoint == true) {
			viewpoint = false;
			cameraPos = cameraPos + glm::vec3(0.0f, 0.0f, 2.0f);
			fov = 60;
		}
	}
	else if (button == GLFW_RELEASE) {
		//std::cout << "Left Key Released!" << std::endl;
		rotation_started = false;
		firstMouse = true;
	}
}

void Scene::mouse_position_callback(GLFWwindow * window, double xpos, double ypos)
{
	if (rotation_started) {
		if (firstMouse)	// this bool variable is initially set to true
		{
			start_x = xpos;
			start_y = ypos;
			firstMouse = false;
		}

		if (rotation_started) {
			float xoffset = start_x - xpos;
			float yoffset = ypos - start_y;	// reversed since y-coordinates range from bottom to top
			start_x = xpos;
			start_y = ypos;

			float sensitivity = 0.1;
			xoffset *= sensitivity;
			yoffset *= sensitivity;

			yaw += xoffset;
			pitch += yoffset;
		}
		if (pitch > 89.0f)
			pitch = 89.0f;
		if (pitch < -89.0f)
			pitch = -89.0f;
	}

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);
}

void Scene::scroll_callback(GLFWwindow * window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 70.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 70.0f)
		fov = 70.0f;
}

void Scene::mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos)
{
	rotation_started = true;
	start_x = xpos;
	start_y = ypos;
}

void Scene::mouseLeftReleaseEvent()
{
	rotation_started = false;

}

void Scene::mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos)
{
	if (rotation_started)
	{
		float xoffset = (ypos - start_y);
		float yoffset = (xpos - start_x);

		float sensitivity = 0.05;
		xoffset *= sensitivity;
		yoffset *= sensitivity;

		yaw += xoffset;
		pitch += yoffset;

		if (pitch > 89.0f)
			pitch = 89.0f;
		if (pitch < -89.0f)
			pitch = -89.0f;

		glm::vec3 front;
		front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
		front.y = sin(glm::radians(pitch));
		front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
		cameraFront = glm::normalize(front);
	}
}

void Scene::glInit(GLFWwindow * window)
{
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// glfw mouse input
	// ---------------gl_Mouse();
	glfwSetCursorPosCallback(window, mouse_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	// glad: load all OpenGL function pointers
	// ---------------------------------------glad_Init();
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_FRONT);
	//glFrontFace(GL_CW);
}

void Scene::start()
{
	//update();
	streamThread = std::thread(&Scene::update,this);
}

void Scene::update()
{
	//map = _map;
	//camera_global_pose = _camera_global_pose;
	//camera_scaled_pose = _camera_scaled_pose;
	//int map_shift[3] = {0};
	scene_stream = true;
	map_shift[0] = camera_global_pose[9];
	map_shift[1] = camera_global_pose[10];
	map_shift[2] = camera_global_pose[11];
	// build and compile shader program
	Shader lightingShader("../res/texture02.vs", "../res/texture02.fs");
	Shader planeshader("../res/grid.vs", "../res/grid.fs");
	// -----------
	// Voxel Cells
	// -----------
	// first, configure the cube's VAO (and VBO)
	unsigned int VBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glBindVertexArray(cubeVAO);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	// texture attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
	unsigned int lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);
	// we only need to bind to the VBO (to link it with glVertexAttribPointer), no need to fill it; the VBO's data already contains all we need (it's already bound, but we do it again for educational purposes)
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// Dear Imgui
#ifdef GUI
	clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);
	const char* glsl_version = "#version 130";
	// Setup Dear ImGui context
	ImGui::CreateContext();
    
    ImGui::StyleColorsDark();

	 // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

	show_2d_map = true;
    show_demo_window = true;
	show_another_window = false;
	//ImVec4 clear_color = ImVec4(0.3f, 0.5f, 0.60f, 1.00f);
	
 	window_size_2d = ImVec2(500,500); //2d map size
#endif //GUI
	
	// load textures (we now use a utility function to keep the code more organized)
	// -----------------------------------------------------------------------------
	//Texture
	unsigned int wallTexture = loadTexture("../res/wall_b.jpg");			//index = 0
	unsigned int creeperTexture  = loadTexture("../res/creeper.jpg");	//index = 1

	// -----
	// Plane
	// -----
	const int gridlines = 202;
	float planeVertices[(gridlines + 2) * 2 * 6];
	int gridlinecount = 0;
	for (int k = 0; k < gridlines; k += 2) {
		//-----------------
		planeVertices[6 * k + 0] = 0.0f;
		planeVertices[6 * k + 1] = 0.0f;
		planeVertices[6 * k + 2] = -0.5*k;
		planeVertices[6 * k + 3] = 1.0f;
		planeVertices[6 * k + 4] = 1.0f;
		planeVertices[6 * k + 5] = 1.0f;

		planeVertices[6 * (k + 1) + 0] = 100.0;
		planeVertices[6 * (k + 1) + 1] = 0.0f;
		planeVertices[6 * (k + 1) + 2] = -0.5*k;
		planeVertices[6 * (k + 1) + 3] = 1.0f;
		planeVertices[6 * (k + 1) + 4] = 1.0f;
		planeVertices[6 * (k + 1) + 5] = 1.0f;	//last 131
		//std::cout << planeVertices[6 * k + 0] << "," << planeVertices[6 * k + 2] << ";" << planeVertices[6 * (k + 1) + 0] << "," << planeVertices[6 * (k + 1) + 2] << std::endl;
		gridlinecount++;
		//||||||||||||||||||||
		planeVertices[6 * k + 0 + 6 * (gridlines)] = 0.5*k;	//1st 132
		planeVertices[6 * k + 1 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * k + 2 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * k + 3 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * k + 4 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * k + 5 + 6 * (gridlines)] = 1.0f;

		planeVertices[6 * (k + 1) + 0 + 6 * (gridlines)] = 0.5*k;
		planeVertices[6 * (k + 1) + 1 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -100.0;
		planeVertices[6 * (k + 1) + 3 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * (k + 1) + 4 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * (k + 1) + 5 + 6 * (gridlines)] = 1.0f;	//263
		//std::cout << planeVertices[6 * k + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * k + 2 + 6 * (gridlines + 2)] << ";" << planeVertices[6 * (k + 1) + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * (k + 1) + 2 + 6 * (gridlines + 2)] << std::endl;
		gridlinecount++;
	}
	unsigned int planeVAO, planeVBO;
	glGenVertexArrays(1, &planeVAO);
	glGenBuffers(1, &planeVBO);
	glBindVertexArray(planeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);	//(x,y,z,R,G,B),each float occupy 4 byte
	glEnableVertexAttribArray(0);
	// color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));	//last variable is the offset of the color data
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);
	
	// ----
	// Axis
	// ----	
	unsigned int axisVAO, axisVBO;
	glGenVertexArrays(1, &axisVAO);
	glGenBuffers(1, &axisVBO);
	glBindVertexArray(axisVAO);
	glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);

	// -------------
	// Field of View
	// -------------
	unsigned int fovVAO, fovVBO;
	glGenVertexArrays(1, &fovVAO);
	glGenBuffers(1, &fovVBO);
	glBindVertexArray(fovVAO);
	glBindBuffer(GL_ARRAY_BUFFER, fovVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(fovVertices), fovVertices, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);
	
	while(!glfwWindowShouldClose(window))
	{
		processInput(window);	//mouse and keyboard input

		// OpenCV 2D Map
		//--------------
		Mat grid2dmap(v_x, v_z, CV_8UC3, Scalar(255, 255, 255));	//xz
		// Draw grid
		for (int i = 1; i < grid_z; i++) {
			line(grid2dmap, Point(i * unit_length_x, 0), Point(i * unit_length_x, v_x), Scalar(0, 0, 0), 1);
		}
		for (int i = 1; i < grid_x; i++) {
			line(grid2dmap, Point(0, i * unit_length_z), Point(v_z, i * unit_length_z), Scalar(0, 0, 0), 1);
		}

		for (int k = 0; k < grid_z; k++) {
			for (int i = 0; i < grid_x; i++) {
				//std::cout << left;
				//std::cout << setprecision(3) << std::setw(5) << voxel_map_logodd[i][specific_row][k];		//show log_odd map in cmd window
				if (voxel_map_logodd[i][specific_row][k] != 127) {
					rectangle(grid2dmap, Point(i * unit_length_x, k * unit_length_z), Point((i + 1) * unit_length_x, (k + 1) * unit_length_z),
						Scalar(voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k]), -1);	//BGR
					//new statement
					//initial_voxel_map[i][specific_row][k] = 0;					
				}
				//std::cout << omp_get_num_threads() << std::endl;
			}
			//std::cout << std::endl;
		}

		/* Roatation */
		// Rotation matrix & Inverse rotation matrix
		float determinant = 0;
		float C[3][3] = { {cos(camera_global_pose[4])*cos(camera_global_pose[5]), cos(camera_global_pose[4])*sin(camera_global_pose[5]), -sin(camera_global_pose[4])},
		{(-cos(camera_global_pose[3])*sin(camera_global_pose[5]) + sin(camera_global_pose[3])*sin(camera_global_pose[4])*cos(camera_global_pose[5])), cos(camera_global_pose[3])*cos(camera_global_pose[5]) + sin(camera_global_pose[3])*sin(camera_global_pose[4])*sin(camera_global_pose[5]), sin(camera_global_pose[3])*cos(camera_global_pose[4])},
		{sin(camera_global_pose[3])*sin(camera_global_pose[5]) + cos(camera_global_pose[3])*sin(camera_global_pose[4])*cos(camera_global_pose[5]), (-sin(camera_global_pose[3])*cos(camera_global_pose[5]) + cos(camera_global_pose[3])*sin(camera_global_pose[4])*sin(camera_global_pose[5])), cos(camera_global_pose[3])*cos(camera_global_pose[4])} };

		// Inverse Rotation matrix
		float inv_C[3][3];
		for (int i = 0; i < 3; i++)
			determinant = determinant + (C[0][i] * (C[1][(i + 1) % 3] * C[2][(i + 2) % 3] - C[1][(i + 2) % 3] * C[2][(i + 1) % 3]));
		for (int mi = 0; mi < 3; mi++) {
			for (int mj = 0; mj < 3; mj++)
				inv_C[mi][mj] = ((C[(mj + 1) % 3][(mi + 1) % 3] * C[(mj + 2) % 3][(mi + 2) % 3]) - (C[(mj + 1) % 3][(mi + 2) % 3] * C[(mj + 2) % 3][(mi + 1) % 3])) / determinant;
		}

		//Right(+-+)
		f1R[0] = f1[0];
		f1R[1] = -f1[1];
		f1R[2] = f1[2];
		//Left(--+)
		f1L[0] = -f1[0];
		f1L[1] = -f1[1];
		f1L[2] = f1[2];

		// FOV conversion
		float f1L[3], f1R[3], f2L[3], f2R[3];
		float f1Rc[3] = { 0,0,0 };
		float f1Lc[3] = { 0,0,0 };
		float f2Rc[3] = { 0,0,0 };
		float f2Lc[3] = { 0,0,0 };

		//f1' = Cc/c' * f1
		for (int l = 0; l < 3; l++) {
			for (int m = 0; m < 3; m++) {
				f1Rc[l] += inv_C[l][m] * f1R[m];
				f1Lc[l] += inv_C[l][m] * f1L[m];
			}
		}

		for (int i = 0; i < 3; i++) {
			f1Rc[i] += camera_scaled_pose[i];
			f1Lc[i] += camera_scaled_pose[i];
		}

		/* Draw the FOV(x-z plane) */

		Point FOV_R(f1Rc[0], f1Rc[2]);	//12m is the max range
		Point FOV_L(f1Lc[0], f1Lc[2]);	//12m is the max range

		line(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), FOV_R, Scalar(0, 255, 0), 2);
		line(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), FOV_L, Scalar(0, 0, 255), 2);
		circle(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), 3, Scalar(0, 0, 0), 2);
		// Create OpenCV matrix of size (w,h) from the colorized depth data

		//flip img
		flip(grid2dmap, grid2dmap, 0);

		// OpenGL 3D Map
		//--------------
		//opengl camera view
		if (viewpoint == true) {
			fov = 75;
			glm::vec3 front;
			front.x = -cos(camera_global_pose[3]) * sin(-camera_global_pose[4]);
			front.y = sin(camera_global_pose[3]);
			front.z = -cos(camera_global_pose[3]) * cos(-camera_global_pose[4]);
			cameraFront = glm::normalize(front);
			cameraPos = glm::vec3(camera_pixel_pose[0] / unit_length_x - map_shift[0] / block_unit_m,
				-camera_pixel_pose[1] / unit_length_y + map_shift[1] / block_unit_m,
				-camera_pixel_pose[2] / unit_length_x + map_shift[2] / block_unit_m + 0.1);
			cameraPos -= front;
		}
		
		//Plane
		const int gridlines = 2*grid_x+2;
		float planeVertices[(gridlines + 2) * 2 * 6];
		int gridlinecount = 0;
		for (int k = 0; k < gridlines; k += 2) {
			//-----------------
			planeVertices[6 * k + 0] = 0.0f;
			planeVertices[6 * k + 1] = -specific_row;
			planeVertices[6 * k + 2] = -0.5*k;
			planeVertices[6 * k + 3] = 1.0f;
			planeVertices[6 * k + 4] = 1.0f;
			planeVertices[6 * k + 5] = 1.0f;

			planeVertices[6 * (k + 1) + 0] = grid_x;
			planeVertices[6 * (k + 1) + 1] = -specific_row;
			planeVertices[6 * (k + 1) + 2] = -0.5*k;
			planeVertices[6 * (k + 1) + 3] = 1.0f;
			planeVertices[6 * (k + 1) + 4] = 1.0f;
			planeVertices[6 * (k + 1) + 5] = 1.0f;	//last 131
			//std::cout << planeVertices[6 * k + 0] << "," << planeVertices[6 * k + 2] << ";" << planeVertices[6 * (k + 1) + 0] << "," << planeVertices[6 * (k + 1) + 2] << std::endl;
			gridlinecount++;
			//||||||||||||||||||||
			planeVertices[6 * k + 0 + 6 * (gridlines)] = 0.5*k;	//1st 132
			planeVertices[6 * k + 1 + 6 * (gridlines)] = -specific_row;
			planeVertices[6 * k + 2 + 6 * (gridlines)] = 0.0f;
			planeVertices[6 * k + 3 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * k + 4 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * k + 5 + 6 * (gridlines)] = 1.0f;

			planeVertices[6 * (k + 1) + 0 + 6 * (gridlines)] = 0.5*k;
			planeVertices[6 * (k + 1) + 1 + 6 * (gridlines)] = -specific_row;
			planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -grid_x;
			planeVertices[6 * (k + 1) + 3 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * (k + 1) + 4 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * (k + 1) + 5 + 6 * (gridlines)] = 1.0f;	//263
			//std::cout << planeVertices[6 * k + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * k + 2 + 6 * (gridlines + 2)] << ";" << planeVertices[6 * (k + 1) + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * (k + 1) + 2 + 6 * (gridlines + 2)] << std::endl;
			gridlinecount++;
		}
		unsigned int planeVAO, planeVBO;
		glGenVertexArrays(1, &planeVAO);
		glGenBuffers(1, &planeVBO);
		glBindVertexArray(planeVAO);
		glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);	//(x,y,z,R,G,B),each float occupy 4 byte
		glEnableVertexAttribArray(0);
		// color attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));	//last variable is the offset of the color data
		glEnableVertexAttribArray(1);
		glBindVertexArray(0);

		// render
		// ------glbgRender(color);
		glClearColor(0.3f, 0.5f, 0.6f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// activate shader
		planeshader.use();

		// view/projection transformations
		glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 300.0f);
		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		planeshader.setMat4("projection", projection);
		planeshader.setMat4("view", view);

		// render floor--------------floor_render();
		glLineWidth(1.0f);
		glm::mat4 plane = glm::mat4(1.0f);
		glBindVertexArray(planeVAO);
		plane = glm::translate(plane, glm::vec3(- map_shift[0]*5, map_shift[1]*5, map_shift[2]*5));
		planeshader.setMat4("model", plane);
		glDrawArrays(GL_LINES, 0, gridlinecount * 2);
		glBindVertexArray(0);

		// render axes-------------axis_render();
		glLineWidth(5.0f);
		glm::mat4 axis = glm::mat4(1.0f);
		glBindVertexArray(axisVAO);
		planeshader.setMat4("model", axis);
		glDrawArrays(GL_LINES, 0, 6);
		glBindVertexArray(0);

		// traj VAO : it has to be close to render part and locked to keep the vector size
		mutex.lock();
		unsigned int trajVAO, trajVBO;
		glGenVertexArrays(1, &trajVAO);
		glGenBuffers(1, &trajVBO);
		glBindVertexArray(trajVAO);
		glBindBuffer(GL_ARRAY_BUFFER, trajVBO);
		glBufferData(GL_ARRAY_BUFFER, scale_trajectory.size() * sizeof(float), &scale_trajectory[0], GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glBindVertexArray(0);

		// render traj------------traj_render();
		glLineWidth(3.0f);
		glm::mat4 gltraj = glm::mat4(1.0f);
		glBindVertexArray(trajVAO);
		planeshader.setMat4("model",gltraj);
		glDrawArrays(GL_LINE_STRIP,0,scale_trajectory.size()/6);
		glBindVertexArray(0);
		mutex.unlock();
		

		// render fov-------------fov_render();
		glLineWidth(2.0f);
		glm::mat4 glfov = glm::mat4(1.0f);
		glBindVertexArray(fovVAO);
		glfov = glm::translate(glfov, glm::vec3(camera_scaled_pose[0] / unit_length_x - map_shift[0]*5, 
			-camera_scaled_pose[1]/ unit_length_x  + map_shift[1]*5, -camera_scaled_pose[2] / unit_length_x + map_shift[2]*5));
		//312
		glfov = glm::rotate(glfov, camera_global_pose[4], glm::vec3(0, -1, 0));	//2
		glfov = glm::rotate(glfov, camera_global_pose[5], glm::vec3(0, 0, -1));	//3
		glfov = glm::rotate(glfov, camera_global_pose[3], glm::vec3(1, 0, 0));	//1

		planeshader.setMat4("model", glfov);
		glDrawArrays(GL_LINES, 0, 16);
		glBindVertexArray(0);
		
		// activate shader for textured boxes
		// be sure to activate shader when setting uniforms/drawing objects
		// shader configuration
		// --------------------
		lightingShader.use();
		lightingShader.setInt("material.diffuse", 0);
		lightingShader.setVec3("light.position", lightPos);
		lightingShader.setVec3("viewPos", cameraPos);

		// light properties
		lightingShader.setVec3("light.ambient", 0.4f, 0.4f, 0.4f);
		lightingShader.setVec3("light.diffuse", 0.5f, 0.5f, 0.5f);
		lightingShader.setVec3("light.specular", 0.2f, 0.2f, 0.2f);

		// material propertiesspecific_row
		lightingShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
		lightingShader.setFloat("material.shininess", 8.0f);

		lightingShader.setMat4("projection", projection);
		lightingShader.setMat4("view", view);

		// setup texture index
		if(texture_index == 0){
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, wallTexture);
		}
		else if(texture_index == 1){
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, creeperTexture);
		}

		// render boxes
		glBindVertexArray(cubeVAO);
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				for (int i = 0; i < grid_x; i++) {
					if (voxel_map_logodd[i][j][k] != 127) {
						if (voxel_map_logodd[i][j][k] >= 180) {

							// calculate the model matrix for each object and pass it to shader bedore drawing
							glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
							model = glm::translate(model,
								glm::vec3((i + 0.5) - map_shift[0]*5, -(j + 0.5)+ map_shift[1]*5, -(k + 0.5)+ map_shift[2]*5));
							//glm::vec3 scale = { probability, probability, probability};
							//model = glm::scale(model, scale);
							lightingShader.setMat4("model", model);
							glDrawArrays(GL_TRIANGLES, 0, 36);
						}
					}
				}
			}
		}

		glBindVertexArray(0);

#ifdef GUI
		ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		unsigned int texture_2d = matToTexture(grid2dmap,GL_LINEAR,GL_LINEAR,GL_CLAMP_TO_EDGE);
		// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        //if (show_demo_window)
        //   ImGui::ShowDemoWindow(&show_demo_window);
		// 1. Show Connect Infomation
//#ifndef READFILE
		{
			ImGui::Begin("Connect Info");
			ImGui::Text("Connection		");	ImGui::SameLine();
			if (connect_stream)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("CAMERA STATE	  ");	ImGui::SameLine();
			if (camera_stream)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("Mapper	 	   ");	ImGui::SameLine();
			if (mapper_status)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			ImGui::End();
		}
		
        // 2. Show Map Infomation
        {
            static float f = 0.0f;
            static int counter = 0;
			
            ImGui::Begin("Map Info");                          // Create a window called "Hello, world!" and append into it.
			ImGui::Text("Refresh rate: average %.2f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Checkbox("2D Map", &show_2d_map);
			if(show_2d_map){
				ImGui::Text("2D Map Size (%i, %i)", (int)window_size_2d.x, (int)window_size_2d.y);
				ImGui::SliderInt("Layer",&specific_row,0,grid_y);
				ImGui::Image((void*)(intptr_t)texture_2d, window_size_2d);
			}
            ImGui::Text("This is some useless text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Click me plz~"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::End();
        }
		
        // 3. Show another simple window.
        if (show_another_window)
        {
            ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }
		
		ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        //glViewport(0, 0, display_w, display_h);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glDeleteTextures(1, &texture_2d);
#endif //GUI

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	scene_stream = false;
#ifdef GUI
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
#endif
}

void Scene::stop()
{
	if(streamThread.joinable()){
        streamThread.join();
        std::cout << "scene streamThread released" << std::endl;
    }
	std::cout << "scene stoped" << std::endl;
}

void Scene::glSetup(unsigned int VAO, unsigned int VBO, float *Vertices, unsigned int AttribSize1, unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset)
{
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
	// position attribute
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, AttribSize1, GL_FLOAT, GL_FALSE, Stride * sizeof(float), (void*)0);
	// texture coord attribute
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, AttribSize2, GL_FLOAT, GL_FALSE, Stride * sizeof(float), (void*)(VertexOffset * sizeof(float)));
	glBindVertexArray(0);	// unbind
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void Scene::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int Scene::loadTexture(char const * path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
	stbi_set_flip_vertically_on_load(true); 
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}

//Mat to GL texture
unsigned int Scene::matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    unsigned int textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
        magFilter == GL_LINEAR_MIPMAP_NEAREST ||
        magFilter == GL_NEAREST_MIPMAP_LINEAR ||
        magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
        magFilter = GL_LINEAR;
    }

    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_RG;
    }

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
        0,                 // Pyramid level (for mip-mapping) - 0 is the top level
        GL_RGB,            // Internal colour format to convert to
        mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
        mat.rows,          // Image height i.e. 480 for Kinect in standard mode
        0,                 // Border width in pixels (can either be 1 or 0)
        inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
        GL_UNSIGNED_BYTE,  // Image data type
        mat.ptr());        // The actual image data itself

    // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
    if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
        minFilter == GL_LINEAR_MIPMAP_NEAREST ||
        minFilter == GL_NEAREST_MIPMAP_LINEAR ||
        minFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        glGenerateMipmap(GL_TEXTURE_2D);
    }
	glBindTexture(GL_TEXTURE_2D, 0);
    return textureID;
}