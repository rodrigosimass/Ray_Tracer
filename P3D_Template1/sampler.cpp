#include "vector.h"
#include "maths.h"

// Sampling with rejection method
Vector sample_unit_disk(void) {
	Vector p;
	do {
		p = Vector(rand_float(), rand_float(), 0.0) * 2 - Vector(1.0, 1.0, 0.0);
	} while (p*p >= 1.0);
	return p;
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

Vector sample_horizontal_square(float size) {
	float randx = RandomFloat(-0.5f,0.5f);
	float randz = RandomFloat(-0.5f,0.5f);
	return Vector(randx,0,randz) * size;
}
