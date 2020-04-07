#define STB_IMAGE_IMPLEMENTATION 
#include "scene.h"

glm::vec3 Scene::cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
glm::vec3 Scene::cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);

bool Scene::firstMouse = true;
float Scene::yaw = -90.0f; // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float Scene::pitch = -30.0f;
float Scene::lastX = 800.0f / 2.0;
float Scene::lastY = 600.0 / 2.0;
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

	// Filed of view
	f1_2 = (unit_length_x / block_unit_m) * sqrt(pow(1, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) + 1));
	f1_0 = tan(half_FOVxz)*f1_2;
	f1_1 = tan(half_FOVyz)*f1_2;
	float f1[3] = { f1_0, f1_1, f1_2 };
	printf("Scene done!\n");
}

Scene::~Scene()
{
	//delete window;
}

void Scene::processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	// moving mode
	float cameraSpeed = 1.0f; // adjust accordingly
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/creeper.jpg");
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/wall.jpg");
	lightPos = cameraPos + glm::vec3(2.0f, 0.0f, -0.5f);
}

void Scene::mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		//std::cout << "Right Key Pressed!" << std::endl;
		//move viewpoint to realsense position
		if (viewpoint == false) {
			viewpoint = true;
		}
		else if (viewpoint == true) {
			viewpoint = false;
			cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
			fov = 45;
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
void Scene::update()
{
	//map = _map;
	//camera_pose = _camera_pose;
	//camera_scaled_pose = _camera_scaled_pose;
	int map_shift[3] = {0};

	map_shift[0] = camera_pose[9];
	map_shift[1] = camera_pose[10];
	map_shift[2] = camera_pose[11];
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

	// load textures (we now use a utility function to keep the code more organized)
	// -----------------------------------------------------------------------------
	unsigned int diffuseMap = loadTexture("../res/wall.jpg");

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

		//opengl camera view
		if (viewpoint == true) {
			fov = 75;
			glm::vec3 front;
			front.x = -cos(camera_pose[3]) * sin(-camera_pose[4]);
			front.y = sin(camera_pose[3]);
			front.z = -cos(camera_pose[3]) * cos(-camera_pose[4]);
			cameraFront = glm::normalize(front);
			cameraPos = glm::vec3(camera_scaled_pose[0] / unit_length_x - map_shift[0]*5,
				-camera_scaled_pose[1] / unit_length_x + map_shift[1]*5,
				-camera_scaled_pose[2] / unit_length_x  + map_shift[2]*5 + 0.1);
			cameraPos -= front;
		}

		//Plane
		const int gridlines = 202;
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

			planeVertices[6 * (k + 1) + 0] = 100.0;
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
		glm::mat4 plane = glm::mat4(1.0f);
		glBindVertexArray(planeVAO);
		plane = glm::translate(plane, glm::vec3(- map_shift[0]*5, map_shift[1]*5, map_shift[2]*5));
		planeshader.setMat4("model", plane);
		glDrawArrays(GL_LINES, 0, gridlinecount * 2);
		glBindVertexArray(0);

		// render axes-------------axis_render();
		glm::mat4 axis = glm::mat4(1.0f);
		glBindVertexArray(axisVAO);
		planeshader.setMat4("model", axis);
		glDrawArrays(GL_LINES, 0, 6);
		glBindVertexArray(0);

		// render fov-------------fov_render();
		glm::mat4 glfov = glm::mat4(1.0f);
		glBindVertexArray(fovVAO);
		glfov = glm::translate(glfov, glm::vec3(camera_scaled_pose[0] / unit_length_x - map_shift[0]*5, 
			-camera_scaled_pose[1]/ unit_length_x  + map_shift[1]*5, -camera_scaled_pose[2] / unit_length_x + map_shift[2]*5));
		//312
		glfov = glm::rotate(glfov, camera_pose[4], glm::vec3(0, -1, 0));	//2
		glfov = glm::rotate(glfov, camera_pose[5], glm::vec3(0, 0, -1));	//3
		glfov = glm::rotate(glfov, camera_pose[3], glm::vec3(1, 0, 0));	//1

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

		// render boxes
		glBindVertexArray(cubeVAO);
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				for (int i = 0; i < grid_x; i++) {
					if (voxelmap[i][j][k] != 127) {
						if (voxelmap[i][j][k] >= 180) {

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
		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
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