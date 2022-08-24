#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void)
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector &v0, const Vector &v1)
{
	min = v0;
	max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB &bbox)
{
	min = bbox.min;
	max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator=(const AABB &rhs)
{
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a grid

bool AABB::isInside(const Vector &p)
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}
bool AABB::intercepts(const Ray& ray, float& t, Vector& Normal, Vector& minT, Vector& maxT, Vector& hitp1, Vector& hitp2)
{
	float ox = ray.origin.x;
	float oy = ray.origin.y;
	float oz = ray.origin.z;

	float dx = ray.direction.x;
	float dy = ray.direction.y;
	float dz = ray.direction.z;


	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;
	
	float a = 1.0 / dx;

	if (a >= 0)
	{
		tx_min = (min.x - ox) * a;
		tx_max = (max.x - ox) * a;
	}
	else
	{
		tx_min = (max.x - ox) * a;
		tx_max = (min.x - ox) * a;
	}

	float b = 1.0 / dy;

	if (b >= 0)
	{
		ty_min = (min.y - oy) * b;
		ty_max = (max.y - oy) * b;
	}
	else
	{
		ty_min = (max.y - oy) * b;
		ty_max = (min.y - oy) * b;
	}

	float c = 1.0 / dz;

	if (c >= 0)
	{
		tz_min = (min.z - oz) * c;
		tz_max = (max.z - oz) * c;
	}
	else
	{
		tz_min = (max.z - oz) * c;
		tz_max = (min.z - oz) * c;
	}

	float t0, t1;			  //entering and exiting points
	Vector face_in, face_out; //normals o the faces

	// find largest entering t value
	if (tx_min > ty_min)
	{
		t0 = tx_min;
		face_in = (a >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}
	else
	{
		t0 = ty_min;
		face_in = (b >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}
	if (tz_min > t0)
	{
		t0 = tz_min;
		face_in = (c >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	// find smallest exiting t value
	if (tx_max < ty_max)
	{
		t1 = tx_max;
		face_out = (a >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}
	else
	{
		t1 = ty_max;
		face_out = (b >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}
	if (tz_max < t1)
	{
		t1 = tz_max;
		face_out = (c >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}

	minT = Vector(tx_min, ty_min, tz_min);
	maxT = Vector(tx_max, ty_max, tz_max);
	

	float tminX = (minT.x - ox) / dx;
	float tminY = (minT.y - oy) / dy;
	float tminZ = (minT.z - oz) / dz;


	float tmaxX = (maxT.x - ox) / dx;
	float tmaxY = (maxT.y - oy) / dy;
	float tmaxZ = (maxT.z - oz) / dz;


	if (tminX > tmaxX) {
		float aux = tminX;
		tminX = tmaxX;
		tmaxX = aux;
	}
	if (tminY > tmaxY) {
		float aux = tminY;
		tminY = tmaxY;
		tmaxY = aux;
	}	if (tminZ > tmaxZ) {
		float aux = tminZ;
		tminZ = tmaxZ;
		tmaxZ = aux;
	}

	hitp1 = Vector(tminX, tminY, tminZ);
	hitp2 = Vector(tmaxX, tmaxY, tmaxZ);

	if (t0 < t1 && t1 > 0) // condition for a hit
	{
		if (t0 > 0)
		{
			t = t0; // ray hits outside surface
			Normal = face_in;
		}
		else
		{
			t = t1; // ray hits inside surface
			Normal = face_out;
		}
		return true;
	}
	else
		return false;

	return true;
}

bool AABB::intercepts(const Ray &ray, float &t, Vector &Normal)
{
	float ox = ray.origin.x;
	float oy = ray.origin.y;
	float oz = ray.origin.z;

	float dx = ray.direction.x;
	float dy = ray.direction.y;
	float dz = ray.direction.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0 / dx;

	if (a >= 0)
	{
		tx_min = (min.x - ox) * a;
		tx_max = (max.x - ox) * a;
	}
	else
	{
		tx_min = (max.x - ox) * a;
		tx_max = (min.x - ox) * a;
	}

	float b = 1.0 / dy;

	if (b >= 0)
	{
		ty_min = (min.y - oy) * b;
		ty_max = (max.y - oy) * b;
	}
	else
	{
		ty_min = (max.y - oy) * b;
		ty_max = (min.y - oy) * b;
	}

	float c = 1.0 / dz;

	if (c >= 0)
	{
		tz_min = (min.z - oz) * c;
		tz_max = (max.z - oz) * c;
	}
	else
	{
		tz_min = (max.z - oz) * c;
		tz_max = (min.z - oz) * c;
	}

	float t0, t1;			  //entering and exiting points
	Vector face_in, face_out; //normals o the faces
	
	// find largest entering t value
	if (tx_min > ty_min)
	{
		t0 = tx_min;
		face_in = (a >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}
	else
	{
		t0 = ty_min;
		face_in = (b >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}
	if (tz_min > t0)
	{
		t0 = tz_min;
		face_in = (c >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	// find smallest exiting t value
	if (tx_max < ty_max)
	{
		t1 = tx_max;
		face_out = (a >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}
	else
	{
		t1 = ty_max;
		face_out = (b >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}
	if (tz_max < t1)
	{
		t1 = tz_max;
		face_out = (c >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}

	if (t0 < t1 && t1 > 0) // condition for a hit
	{ 
		if (t0 > 0)
		{
			t = t0; // ray hits outside surface
			Normal = face_in;
		}
		else
		{
			t = t1; // ray hits inside surface
			Normal = face_out;
		}
		return true;
	}
	else
		return false;

	return true;
}
#endif