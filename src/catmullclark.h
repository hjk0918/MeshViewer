#ifndef CATMULLCLARK
#define CATMULLCLARK

#include "mesh.h"

void get_face_points(Mesh* mesh, Mesh* previous);

void get_edge_points(Mesh* mesh, Mesh* previous);

void get_vertex_points(Mesh* mesh, Mesh* previous);

void get_faces(Mesh* mesh, Mesh* previous);


#endif