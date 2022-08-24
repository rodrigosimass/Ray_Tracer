#ifndef PROJECT_OPTIONS_H
#define PROJECT_OPTIONS_H
#define CAPTION "Distributed Ray Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define DRAW_MODE_ACTIVE true

// REFLECTIONS AND REFRACTIONS PARAM
#define MAX_DEPTH 5

// ANTI-ALIASING PARAMS
#define AA_GRID_SIZE 7 // if this is 3 then a 3x3 grid is used for the jittering
#define AA_ACTIVE true

// DEPTH OF FIELD PARAMS
#define DOF_ACTIVE true

// SOFT SHADOWS PARAMS
#define SS_ACTIVE true
#define SS_LIGHT_SIZE 0.5
#define SS_GRID_SIZE 5

// GRID ACCELARATING STRUCTURE PARAMS
#define GRID_ACTIVE false
#define GRID_M 3

#define MAILBOX_ACTIVE false

#endif