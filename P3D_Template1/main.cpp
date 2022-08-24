///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2019 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>
#include <limits>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "grid.h"
#include "maths.h"
#include "sampler.h"
#include "project_options.h"

//Enable OpenGL drawing.
bool drawModeEnabled = DRAW_MODE_ACTIVE;

//Draw Mode: 0 - point by point; 1 - line by line; 2 - full frame at once
int draw_mode = 1;

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16]; //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene *scene = NULL;
Grid grid;
int RES_X, RES_Y;

int WindowHandle = 0;

int global_ray_id = 0;

Color rayTracing(Scene *scene, Ray ray, int depth, float ior_1) //index of refraction of medium 1 where the ray is travelling
{
	float closest_dist = std::numeric_limits<float>::max();
	Object *closest_obj = NULL;
	int ix, iy, iz;

	if (!GRID_ACTIVE) // test all objects in the scene
	{
		for (int i = 0; i < scene->getNumObjects(); i++)
		{
			Object *o = scene->getObject(i);
			float dist;

			if (o->intercepts(ray, dist) && dist < closest_dist)
			{
				closest_obj = o;
				closest_dist = dist;
			}
		}
	}
	else //use grid
	{
		global_ray_id++;
		grid.ray_Traverse(ray, closest_dist, closest_obj, global_ray_id);
	}

	if (closest_obj == NULL) // ray does not intercept any object
		return scene->GetSkyboxColor(ray);
		//return scene->GetBackgroundColor();

	Vector hit_point = ray.origin + ray.direction * closest_dist;
	Vector hit_normal = closest_obj->getNormal(hit_point);

	Color final_color = Color();

	//SHADOWS
	if (!SS_ACTIVE) // NO SOFT SHADOWS
	{
		for (int i = 0; i < scene->getNumLights(); i++)
		{
			Light *l = scene->getLight(i);
			Vector l_pos = (l->position - hit_point);

			float dist_to_l = l_pos.length();

			if (hit_normal * l_pos.normalize() > 0.0)
			{
				//shadow feelers
				Ray r = Ray(hit_point + hit_normal * EPSILON, l_pos.normalize());
				bool in_Shadow = false;

				// check if shadow feeler is obstructed by any object
				if (!GRID_ACTIVE)
				{
					for (int i = 0; i < scene->getNumObjects(); i++)
					{
						Object *o = scene->getObject(i);
						float dist;

						if (o->intercepts(r, dist) && dist < dist_to_l)
						{
							in_Shadow = true;
							break;
						}
					}
				}
				else
				{
					if (grid.bbox.isInside(l->position))
						in_Shadow = grid.shadow_Traverse(r, dist_to_l, global_ray_id);
					else
						in_Shadow = grid.safe_shadow_Traverse(r, global_ray_id);
				}

				if (!in_Shadow) //if not in shadow calculate local color
				{
					Vector li = l_pos.normalize();
					Vector v = (ray.origin - hit_point).normalize();
					Vector h = (li + v).normalize();

					//DIFUSE
					final_color = final_color + l->color * closest_obj->GetMaterial()->GetDiffColor() * closest_obj->GetMaterial()->GetDiffuse() * max(0.0, (hit_normal * li)) * 0.7;

					//SPECLAR
					final_color = final_color + l->color * closest_obj->GetMaterial()->GetSpecColor() * closest_obj->GetMaterial()->GetSpecular() * pow(max(0.0, h * hit_normal), closest_obj->GetMaterial()->GetShine());
				}
			}
		}
	}
	else // SOFT SHADOWS ACTIVE
	{
		if (AA_ACTIVE) /*SS WITH AA*/
		{
			for (int i = 0; i < scene->getNumLights(); i++)
			{
				Light *l = scene->getLight(i);
				Vector light_offset = l->position + sample_horizontal_square((float)SS_LIGHT_SIZE);
				Vector l_pos = (light_offset - hit_point);

				float dist_to_l = l_pos.length();

				if (hit_normal * l_pos.normalize() > 0.0)
				{
					//shadow feelers
					Ray r = Ray(hit_point + hit_normal * EPSILON, l_pos.normalize());
					bool in_Shadow = false;

					// check if shadow feeler is obstructed by any object
					// check if shadow feeler is obstructed by any object
					if (!GRID_ACTIVE)
					{
						for (int i = 0; i < scene->getNumObjects(); i++)
						{
							Object *o = scene->getObject(i);
							float dist;

							if (o->intercepts(r, dist) && dist < dist_to_l)
							{
								in_Shadow = true;
								break;
							}
						}
					}
					else
					{
						if (grid.bbox.isInside(l->position))
							in_Shadow = grid.shadow_Traverse(r, dist_to_l, global_ray_id);
						else
							in_Shadow = grid.safe_shadow_Traverse(r, global_ray_id);
					}
					if (!in_Shadow) //if not in shadow calculate local color
					{
						Vector li = l_pos.normalize();
						Vector v = (ray.origin - hit_point).normalize();
						Vector h = (li + v).normalize();

						//DIFUSE
						final_color = final_color + l->color * closest_obj->GetMaterial()->GetDiffColor() * closest_obj->GetMaterial()->GetDiffuse() * max(0.0, (hit_normal * li)) * 0.7;

						//SPECLAR
						final_color = final_color + l->color * closest_obj->GetMaterial()->GetSpecColor() * closest_obj->GetMaterial()->GetSpecular() * pow(max(0.0, h * hit_normal), closest_obj->GetMaterial()->GetShine());
					}
				}
			}
		}
		else /* SS WITHOUT AA */
		{
			float grid_step = (float)SS_LIGHT_SIZE / (float)SS_GRID_SIZE;
			set_rand_seed(time(NULL) * time(NULL));
			float sample_intensity = 1.0f / (SS_GRID_SIZE * SS_GRID_SIZE);
			for (int i = 0; i < scene->getNumLights(); i++)
			{
				Light *l = scene->getLight(i);
				Vector light_corner = l->position - Vector(SS_LIGHT_SIZE / 2, 0, SS_LIGHT_SIZE / 2);

				for (int p = 0; p < SS_GRID_SIZE; p++)
				{
					for (int q = 0; q < SS_GRID_SIZE; q++)
					{
						Vector light_offset;
						float rx = rand_float();
						float rz = rand_float();
						light_offset.x = (p + rx) * grid_step;
						light_offset.z = (q + rz) * grid_step;
						light_offset.y = 0;

						light_offset = light_offset + l->position;

						Vector l_pos = (light_offset - hit_point);

						float dist_to_l = l_pos.length();

						if (hit_normal * l_pos.normalize() > 0.0)
						{
							//shadow feelers
							Ray r = Ray(hit_point + hit_normal * EPSILON, l_pos.normalize());
							bool in_Shadow = false;

							// check if shadow feeler is obstructed by any object
							if (!GRID_ACTIVE)
							{
								for (int i = 0; i < scene->getNumObjects(); i++)
								{
									Object *o = scene->getObject(i);
									float dist;

									if (o->intercepts(r, dist) && dist < dist_to_l)
									{
										in_Shadow = true;
										break;
									}
								}
							}
							else
							{
								if (grid.bbox.isInside(l->position))
									in_Shadow = grid.shadow_Traverse(r, dist_to_l, global_ray_id);
								else
									in_Shadow = grid.safe_shadow_Traverse(r, global_ray_id);
							}
							if (!in_Shadow) //if not in shadow calculate local color
							{
								Vector li = l_pos.normalize();
								Vector v = (ray.origin - hit_point).normalize();
								Vector h = (li + v).normalize();

								//DIFUSE
								final_color = final_color + (l->color * closest_obj->GetMaterial()->GetDiffColor() * closest_obj->GetMaterial()->GetDiffuse() * max(0.0, (hit_normal * li)) * 0.7) * sample_intensity;

								//SPECLAR
								final_color = final_color + (l->color * closest_obj->GetMaterial()->GetSpecColor() * closest_obj->GetMaterial()->GetSpecular() * pow(max(0.0, h * hit_normal), closest_obj->GetMaterial()->GetShine())) * sample_intensity;
							}
						}
					}
				}
			}
		}
	}

	if (depth >= MAX_DEPTH)
		return final_color;

	if (closest_obj->GetMaterial()->GetTransmittance() == 0.0f) // purely reflective object
	{
		//REFLECTIONS
		if (closest_obj->GetMaterial()->GetReflection() > 0) // Object is reflective
		{
			Vector V = (ray.origin - hit_point).normalize();
			Vector Rr = hit_normal * 2 * (V * hit_normal) - V;

			Ray refl_ray = Ray(hit_point + hit_normal * EPSILON, Rr.normalize());
			Color refl_color = rayTracing(scene, refl_ray, depth + 1, ior_1);
			//refl_color *= closest_obj->GetMaterial()->GetSpecular();
			refl_color *= closest_obj->GetMaterial()->GetSpecular();
			final_color += refl_color;
		}
	}
	else // transparent object
	{

		float ior_2;

		if (ior_1 == 1.0f) // on air
			ior_2 = closest_obj->GetMaterial()->GetRefrIndex();
		else // inside object
		{
			ior_2 = 1.0f;
			hit_normal = hit_normal * -1.0f;
		}

		float n = ior_1 / ior_2;
		Vector V = (ray.origin - hit_point).normalize();
		float cosI = hit_normal * V; // as both V and hit_normal are normalized
		float sinI = sqrtf(1 - pow(cosI, 2));
		float sinT = n * sinI;

		float Kr;
		float T;

		//optimization: only calculate square root if needed
		float aux = 1.0f - (sinT * sinT);
		if (aux > 0)
		{
			float cosT = sqrtf(aux);

			//fresnel equations TODO: rs and rp aare wrong
			float Rs = pow((ior_1 * cosI - ior_2 * cosT) / (ior_1 * cosI + ior_2 * cosT), 2);
			float Rp = pow((ior_1 * cosT - ior_2 * cosI) / (ior_1 * cosT + ior_2 * cosI), 2);
			Kr = 0.5f * (Rs + Rp);
			if (Kr > 1.0f)
				Kr = 1.0f;
			if (Kr < 0.0f)
				Kr = 0.0f;
			T = 1 - Kr;

			//simple transparency
			/* T = closest_obj->GetMaterial()->GetTransmittance();
			Kr = 1.0f - T; */

			Vector vt = hit_normal * (V * hit_normal) - V;
			Vector t = (vt * (1.0f / sinI)).normalize();

			Vector refractedDirection = t * sinT - hit_normal * cosT;
			Vector hit_point_refr = hit_point - hit_normal * EPSILON; //offset to deal with acne

			Ray trans_ray = Ray(hit_point_refr, refractedDirection.normalize());

			Color transm_color = rayTracing(scene, trans_ray, depth + 1, ior_2);
			transm_color *= T;
			final_color += transm_color;
		}
		else
		{ // as aux is 0 or negative only reflection occurs
			Kr = closest_obj->GetMaterial()->GetSpecular();
		}

		//REFLECTIONS
		if (closest_obj->GetMaterial()->GetReflection() > 0) // Object is reflective
		{
			Vector V = (ray.origin - hit_point).normalize();
			Vector Rr = hit_normal * 2 * (V * hit_normal) - V;

			Ray refl_ray = Ray(hit_point + hit_normal * EPSILON, Rr.normalize());
			Color refl_color = rayTracing(scene, refl_ray, depth + 1, ior_1);
			refl_color *= Kr;
			final_color += refl_color;
		}
	}

	return final_color;
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError()
{
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR)
	{
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if (isOpenGLError())
	{
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar *VertexShader =
	{
		"#version 430 core\n"

		"in vec2 in_Position;\n"
		"in vec3 in_Color;\n"
		"uniform mat4 Matrix;\n"
		"out vec4 color;\n"

		"void main(void)\n"
		"{\n"
		"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
		"	color = vec4(in_Color, 1.0);\n"
		"	gl_Position = Matrix * position;\n"

		"}\n"};

const GLchar *FragmentShader =
	{
		"#version 430 core\n"

		"in vec4 color;\n"
		"out vec4 out_Color;\n"

		"void main(void)\n"
		"{\n"
		"	out_Color = color;\n"
		"}\n"};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");

	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);

	// unbind the VAO
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);

	if (draw_mode == 0)
		glDrawArrays(GL_POINTS, 0, 1);
	else if (draw_mode == 1)
		glDrawArrays(GL_POINTS, 0, RES_X);
	else
		glDrawArrays(GL_POINTS, 0, RES_X * RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename)
{
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)
		return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (GRID_ACTIVE)
	{
		grid = Grid(scene->objects, scene->getNumObjects());
		grid.Build();
	}

	if (!AA_ACTIVE) // Anti aliasing is disabled
	{
		for (int y = 0; y < RES_Y; y++)
		{
			for (int x = 0; x < RES_X; x++)
			{
				Color color;

				Vector pixel; //viewport coordinates
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);

				color = rayTracing(scene, ray, 1, 1.0);
				//color = scene->GetBackgroundColor(); //just for the template

				img_Data[counter++] = u8fromfloat((float)color.r());
				img_Data[counter++] = u8fromfloat((float)color.g());
				img_Data[counter++] = u8fromfloat((float)color.b());

				if (drawModeEnabled)
				{
					vertices[index_pos++] = (float)x;
					vertices[index_pos++] = (float)y;
					colors[index_col++] = (float)color.r();

					colors[index_col++] = (float)color.g();

					colors[index_col++] = (float)color.b();

					if (draw_mode == 0)
					{ // drawing point by point
						drawPoints();
						index_pos = 0;
						index_col = 0;
					}
				}
			}
			if (draw_mode == 1 && drawModeEnabled)
			{ // drawing line by line
				drawPoints();
				index_pos = 0;
				index_col = 0;
			}
		}
	}
	else // Anti Aliasing is ON
	{
		float grid_step = 1.0f / AA_GRID_SIZE;
		set_rand_seed(time(NULL) * time(NULL));
		for (int y = 0; y < RES_Y; y++)
		{
			for (int x = 0; x < RES_X; x++)
			{
				Color color = Color();
				for (int p = 0; p < AA_GRID_SIZE; p++)
				{
					for (int q = 0; q < AA_GRID_SIZE; q++)
					{
						Vector sub_pixel;
						float rx = rand_float();
						float ry = rand_float();
						sub_pixel.x = x + (p + rx) * grid_step;
						sub_pixel.y = y + (q + ry) * grid_step;

						if (DOF_ACTIVE)
						{
							Vector ls = sample_unit_disk() * scene->GetCamera()->GetAperture();
							Ray ray = scene->GetCamera()->PrimaryRay(ls, sub_pixel);
							color += rayTracing(scene, ray, 1, 1.0);
						}
						else
						{
							Ray ray = scene->GetCamera()->PrimaryRay(sub_pixel);
							color += rayTracing(scene, ray, 1, 1.0);
						}
					}
				}
				color = color * (1.0f / (AA_GRID_SIZE * AA_GRID_SIZE)); // determin the average color

				img_Data[counter++] = u8fromfloat((float)color.r());
				img_Data[counter++] = u8fromfloat((float)color.g());
				img_Data[counter++] = u8fromfloat((float)color.b());

				if (drawModeEnabled)
				{
					vertices[index_pos++] = (float)x;
					vertices[index_pos++] = (float)y;
					colors[index_col++] = (float)color.r();

					colors[index_col++] = (float)color.g();

					colors[index_col++] = (float)color.b();

					if (draw_mode == 0)
					{ // drawing point by point
						drawPoints();
						index_pos = 0;
						index_col = 0;
					}
				}
			}
			if (draw_mode == 1 && drawModeEnabled)
			{ // drawing line by line
				drawPoints();
				index_pos = 0;
				index_col = 0;
			}
		}
	}

	if (draw_mode == 2 && drawModeEnabled) //full frame at once
		drawPoints();

	printf("Drawing finished!\n");

	if (saveImgFile("RT_Output.png") != IL_NO_ERROR)
	{
		printf("Error saving Image file\n");
		exit(0);
	}
	printf("Image file created\n");
	glFlush();
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top,
		   float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key)
	{

	case 27:
		glutLeaveMainLoop();
		break;
	}
}

/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
}

void setupGLEW()
{
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK)
	{
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	}
	GLenum err_code = glGetError();
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char *argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glutInitWindowPosition(640, 100);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if (WindowHandle < 1)
	{
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}

void init(int argc, char *argv[])
{
	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}

void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	while (true)
	{
		cout << "Input the Scene Name: ";
		cin >> input_user;
		strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
		strcat_s(scene_name, sizeof(scene_name), input_user);

		ifstream file(scene_name, ios::in);
		if (file.fail())
		{
			printf("\nError opening P3F file.\n");
		}
		else
			break;
	}

	scene = new Scene();
	scene->load_p3f(scene_name);
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t *)malloc(3 * RES_X * RES_Y * sizeof(uint8_t));
	if (img_Data == NULL)
		exit(1);
}

int main(int argc, char *argv[])
{
	//Initialization of DevIL
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled)
	{

		do
		{
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene(); //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);

			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete (scene);
			free(img_Data);
			ch = _getch();
		} while ((toupper(ch) == 'Y'));
	}

	else
	{ //Use OpenGL to draw image in the screen
		init_scene();
		if (draw_mode == 0)
		{ // draw image point by point
			size_vertices = 2 * sizeof(float);
			size_colors = 3 * sizeof(float);
			printf("DRAWING MODE: POINT BY POINT\n\n");
		}
		else if (draw_mode == 1)
		{ // draw image line by line
			size_vertices = 2 * RES_X * sizeof(float);
			size_colors = 3 * RES_X * sizeof(float);
			printf("DRAWING MODE: LINE BY LINE\n\n");
		}
		else if (draw_mode == 2)
		{ // draw full frame at once
			size_vertices = 2 * RES_X * RES_Y * sizeof(float);
			size_colors = 3 * RES_X * RES_Y * sizeof(float);
			printf("DRAWING MODE: FULL IMAGE\n\n");
		}
		else
		{
			printf("Draw mode not valid \n");
			exit(0);
		}
		vertices = (float *)malloc(size_vertices);
		if (vertices == NULL)
			exit(1);

		colors = (float *)malloc(size_colors);
		if (colors == NULL)
			exit(1);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////