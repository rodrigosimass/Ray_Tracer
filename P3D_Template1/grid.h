#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include "scene.h"
#include "project_options.h"

//using namespace std;

class Mail
{
public:
	int ray_id;
    float t;
    bool hit;
    Mail();
};

class Mailbox
{
public:
    Mail* box;

    Mailbox();
    Mailbox(int numObjs);
    bool check(int obj_id, int ray_id, float &t, bool &h);
    void update(int obj_id, int ray_id, float dist, bool h);
};

class Grid
{
public:
	Grid(void);
	Grid(vector<Object*> objs, int num_obj);
	//~Grid(void);

	int getNumObjects();
	Object* getObject(unsigned int index);
	vector<Object*> getCell(int ix, int iy, int iz);

	void Build(void);   // set up grid cells

	//bool Traverse(Ray& ray, Object **hitobject, Vector& hitpoint);  //(const Ray& ray, double& tmin, ShadeRec& sr)
	//bool Traverse(Ray& ray);  //Traverse for shadow ray

	vector<Object *> objects;
	vector<vector<Object*> > cells;
	Mailbox mailbox;

	int N; //Number of objects
	int Nx, Ny, Nz; // number of cells in the x, y, and z directions
	float m; // factor that allows to vary the number of cells

	Vector p0; //min of the grid BB
	Vector p1; //max of the grid BB

	/* bool cellClosestHit(std::vector<Object *> cellObjs, Ray &ray, float &closets_dist, Object *&closest_obj, int r_id);
	bool safeIsObscured(std::vector<Object *> cellObjs, Ray &ray, int r_id);
	bool IsObscured(std::vector<Object *> cellObjs, Ray &ray, float l_dist, int r_id); */

	void ray_Traverse(Ray &ray, float &closest_dist, Object* &closest_obj, int r_id);
	bool shadow_Traverse(Ray &ray, float l_dist, int r_id);
	bool safe_shadow_Traverse(Ray &ray, int r_id);
	AABB bbox;
};
#endif
