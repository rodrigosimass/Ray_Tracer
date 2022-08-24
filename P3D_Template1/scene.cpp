#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include <iostream>
#include "maths.h"
#include "scene.h"

#define EPS 0.00001

Triangle::Triangle(Vector &P0, Vector &P1, Vector &P2)
{
	points[0] = P0;
	points[1] = P1;
	points[2] = P2;

	/* Calculate the normal */
	Vector v01 = P1 - P0;
	Vector v02 = P2 - P0;
	normal = v01 % v02; // cross product
	normal.normalize();

	//Calculate the Min and Max for bounding box
	Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	//float minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float value = points[i].get(j);
			if (value < Min.get(j)) 
				Min.set(j, value);
			if (value > Max.get(j)) 
				Max.set(j, value);
		}
	}

	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox()
{
	return (AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray &r, float &t)
{
	Plane p = Plane(points[0], points[1], points[2]);

	if (p.intercepts(r, t))
	{
		float a = points[1].x - points[0].x;
		float b = points[2].x - points[0].x;
		float c = -1 * r.direction.x;
		float d = r.origin.x - points[0].x;

		float e = points[1].y - points[0].y;
		float f = points[2].y - points[0].y;
		float g = -1 * r.direction.y;
		float h = r.origin.y - points[0].y;

		float i = points[1].z - points[0].z;
		float j = points[2].z - points[0].z;
		float k = -1 * r.direction.z;
		float l = r.origin.z - points[0].z;

		float denom = a * (f * k - g * j) + b * (g * i - e * k) + c * (e * j - f * i);

		float beta = (d * (f * k - g * j) + b * (g * l - h * k) + c * (h * j - f * l)) / denom;
		if (beta > 1.0 || beta < 0.0)
			return false;

		float gama = (a * (h * k - g * l) + d * (g * i - e * k) + c * (e * l - h * i)) / denom;
		if (gama > 1.0 || gama < 0.0)
			return false;

		if (gama + beta > 1.0f)
			return false;

		t = (a * (f * l - h * j) + b * (h * i - e * l) + d * (e * j - f * i)) / denom;
		if (t < EPS)
			return false;

		return true;
	}
	return (false);
}

Plane::Plane(Vector &a_PN, float a_D)
	: PN(a_PN), D(a_D)
{
}

Plane::Plane(Vector &P0, Vector &P1, Vector &P2)
	: Point0(P0), Point1(P1), Point2(P2)
{
	float l;

	Vector v01 = P1 - P0;
	Vector v02 = P2 - P0;

	PN = v01 % v02; // cross product

	if ((l = PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		PN.normalize();
	}
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts(Ray &r, float &t)
{
	if (PN * r.direction == 0.0f) //parallel
		return false;

	t = ((r.origin - Point0) * PN / (PN * r.direction)) * -1;

	if (t < 0)
		return false; // behind the origin
	else
		return true;
}

Vector Plane::getNormal(Vector point)
{
	return PN;
}

bool Sphere::intercepts(Ray &r, float &t)
{
	float a = 1.0f;
	float b = (center.x - r.origin.x) * r.direction.x + (center.y - r.origin.y) * r.direction.y + (center.z - r.origin.z) * r.direction.z;
	float c = (center.x - r.origin.x) * (center.x - r.origin.x) + (center.y - r.origin.y) * (center.y - r.origin.y) + (center.z - r.origin.z) * (center.z - r.origin.z) - radius * radius;

	if (c > 0.0f)
	{
		if (b <= 0.0f)
			return false;
	}
	float disc = b * b - c;

	if (disc <= 0.0f)
		return false;

	if (c > 0.0f)
	{
		t = b - sqrt(disc);
		return true;
	}
	else
	{
		t = b + sqrt(disc);
		return true;
	}

	return false;
}

Vector Sphere::getNormal(Vector point)
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox()
{
	Vector a_min = center - Vector(radius,radius,radius);
	Vector a_max = center + Vector(radius,radius,radius);
	return (AABB(a_min, a_max));
}

aaBox::aaBox(Vector &minPoint, Vector &maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox()
{
	return (AABB(min, max));
}

bool aaBox::intercepts(Ray &ray, float &t)
{
	AABB box = AABB(min,max);
	return box.intercepts(ray, t, Normal);
}

Vector aaBox::getNormal(Vector point)
{
	return Normal;
}

Scene::Scene()
{
}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

int Scene::getNumObjects()
{
	return objects.size();
}

void Scene::addObject(Object *o)
{
	objects.push_back(o);
}

Object *Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}

int Scene::getNumLights()
{
	return lights.size();
}

void Scene::addLight(Light *l)
{
	lights.push_back(l);
}

Light *Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];

	const char* maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };


	for (int i = 0; i < 6; i++)
	{
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char *)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}

	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++)
	{
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i])) //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray &r)
{
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;

	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y))
	{
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT; //left cubemap at X = +1 and right at X = -1
	}
	else
	{
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma)
	{
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK; //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side)
	{

	case 0: //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1: //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2: //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4: //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 2]);

	return (Color(red, green, blue));
}

////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream &file, char *token, const char *name)
{
	file >> token;
	if (strcmp(token, name))
		cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char *name)
{
	const int lineSize = 1024;
	string cmd;
	char token[256];
	ifstream file(name, ios::in);
	Material *material;

	material = NULL;

	if (file >> cmd)
	{
		while (true)
		{

			if (cmd == "f") //Material
			{
				double Kd, Ks, Shine, T, ior;
				Color cd, cs;

				file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

				material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
			}

			else if (cmd == "s") //Sphere
			{
				Vector center;
				float radius;
				Sphere *sphere;

				file >> center >> radius;
				sphere = new Sphere(center, radius);
				if (material)
					sphere->SetMaterial(material);
				this->addObject((Object *)sphere);
			}

			else if (cmd == "box") //axis aligned box
			{
				Vector minpoint, maxpoint;
				aaBox *box;

				file >> minpoint >> maxpoint;
				box = new aaBox(minpoint, maxpoint);
				if (material)
					box->SetMaterial(material);
				this->addObject((Object *)box);
			}
			else if (cmd == "p") // Polygon: just accepts triangles for now
			{
				Vector P0, P1, P2;
				Triangle *triangle;
				unsigned total_vertices;

				file >> total_vertices;
				if (total_vertices == 3)
				{
					file >> P0 >> P1 >> P2;
					triangle = new Triangle(P0, P1, P2);
					if (material)
						triangle->SetMaterial(material);
					this->addObject((Object *)triangle);
				}
				else
				{
					cerr << "Unsupported number of vertices.\n";
					break;
				}
			}

			else if (cmd == "pl") // General Plane
			{
				Vector P0, P1, P2;
				Plane *plane;

				file >> P0 >> P1 >> P2;
				plane = new Plane(P0, P1, P2);
				if (material)
					plane->SetMaterial(material);
				this->addObject((Object *)plane);
			}

			else if (cmd == "l") // Need to check light color since by default is white
			{
				Vector pos;
				Color color;

				file >> pos >> color;

				this->addLight(new Light(pos, color));
			}
			else if (cmd == "v")
			{
				Vector up, from, at;
				float fov, hither;
				int xres, yres;
				Camera *camera;
				float focal_ratio;	//ratio beteween the focal distance and the viewplane distance
				float aperture_ratio; // number of times to be multiplied by the size of a pixel

				next_token(file, token, "from");
				file >> from;

				next_token(file, token, "at");
				file >> at;

				next_token(file, token, "up");
				file >> up;

				next_token(file, token, "angle");
				file >> fov;

				next_token(file, token, "hither");
				file >> hither;

				next_token(file, token, "resolution");
				file >> xres >> yres;

				next_token(file, token, "aperture");
				file >> aperture_ratio;

				next_token(file, token, "focal");
				file >> focal_ratio;
				// Create Camera
				camera = new Camera(from, at, up, fov, hither, 100.0 * hither, xres, yres, aperture_ratio, focal_ratio);
				this->SetCamera(camera);
			}

			else if (cmd == "bclr") //Background color
			{
				Color bgcolor;
				file >> bgcolor;
				this->SetBackgroundColor(bgcolor);
			}

			else if (cmd == "env")
			{
				file >> token;

				this->LoadSkybox(token);
				this->SetSkyBoxFlg(true);
			}
			else if (cmd[0] == '#')
			{
				file.ignore(lineSize, '\n');
			}
			else
			{
				cerr << "unknown command '" << cmd << "'.\n";
				break;
			}
			if (!(file >> cmd))
				break;
		}
	}

	file.close();
	return true;
};
