#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <stdio.h>
using namespace std;

#include "vector.h"
#include "ray.h"

#define PI 3.141592653589793238462f

class Camera
{

private:
	Vector eye, at, up;
	float fovy, vnear, vfar, plane_dist, focal_ratio, aperture, aperture_ratio;
	float w, h;
	int res_x, res_y;
	Vector u, v, n; //xe, ye, ze

public:
	int GetResX() { return res_x; }
	int GetResY() { return res_y; }
	float GetFov() { return fovy; }
	float GetPlaneDist() { return plane_dist; }
	float GetFar() { return vfar; }
	float GetAperture() { return aperture; }
	float GetApertureRatio() { return aperture_ratio; }

	Camera(Vector from, Vector At, Vector Up, float angle, float hither, float yon, int ResX, int ResY, float Aperture_ratio, float Focal_ratio)
	{
		eye = from;
		at = At;
		up = Up;
		fovy = angle;
		vnear = hither;
		vfar = yon;
		res_x = ResX;
		res_y = ResY;
		focal_ratio = Focal_ratio;
		aperture_ratio = Aperture_ratio;

		// set the camera frame uvn
		n = (eye - at);
		plane_dist = n.length();
		n = n / plane_dist;

		u = up % n;
		u = u / u.length();

		v = n % u;

		//Dimensions of the vis window
		h = 2 * plane_dist * tan((PI * angle / 180) / 2.0f);
		w = ((float)res_x / res_y) * h;

		aperture = Aperture_ratio * (w / res_x); //Lens aperture = aperture_ratio * pixel_size

		printf("\nwidth=%f height=%f fov=%f, viewplane distance=%f, pixel size=%.3f\n", w, h, fovy, plane_dist, w / res_x);
		if (Aperture_ratio != 0)
			printf("\nDepth-Of-Field effect enabled with a lens aperture = %.1f\n", Aperture_ratio);
	}

	Ray PrimaryRay(const Vector &pixel_sample) //  Rays cast from the Eye to a pixel sample which is in Viewport coordinates
	{

		float ux = w * ((pixel_sample.x) / res_x - 0.5);
		Vector v1 = u * ux;

		float vy = h * ((pixel_sample.y) / res_y - 0.5);
		Vector v2 = v * vy;

		Vector v3 = n * (-1 * plane_dist);

		Vector dir_ray = (v1 + v2 + v3).normalize();
		return Ray(eye, dir_ray);
	}

	Ray PrimaryRay(Vector &ls, Vector &ps) // DOF: Rays cast from  a thin lens sample to a pixel sample
	{

		float px = w * (ps.x / res_x - 0.5f) * focal_ratio;
		float py = h * (ps.y / res_y - 0.5f) * focal_ratio;

		/*float pz = -1 * plane_dist * focal_ratio;
		Vector p = Vector(px,py,pz);
		Vector ray_dir = (p-ls).normalize(); */

		Vector v1 = u * (px - ls.x);
		Vector v2 = v * (py - ls.y);
		Vector v3 = n * (-1.0f * focal_ratio * plane_dist);

		Vector ray_dir = (v1 + v2 + v3).normalize();

		Vector eye_offset = eye + u * ls.x + v * ls.y;

		return Ray(eye_offset, ray_dir);
	}
};

#endif