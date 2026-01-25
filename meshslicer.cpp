/*Authors: Rodrigo Minetto (UTFPR). */
/*         Jorge Stolfi (UNICAMP).  */
/*rodrigo.minetto@gmail.com*/

/*
modified by pzp
*/
#include "meshslicer.h"

#include <glm/gtc/matrix_transform.hpp>
#include "glm-aabb/aabb.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <map>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>


/*-----------------------------------------------------------------------*/
struct bounding_box {
	double xMin;
	double xMax;
	double yMin;
	double yMax;
	bool firstPoint;
} ;



bounding_box create_bounding_box() {
	bounding_box bb;
	bb.xMax = std::numeric_limits<double>::min();
	bb.xMin = std::numeric_limits<double>::max();
	bb.yMax = std::numeric_limits<double>::min();
	bb.yMin = std::numeric_limits<double>::max();
	return bb;
}
/*-----------------------------------------------------------------------*/
class Triangle {

public:

	Triangle(glm::f64vec3 v0, glm::f64vec3 v1, glm::f64vec3 v2) {
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
		zMin = +99999999.9;
		zMax = -99999999.9;
		setZMin(v0.z); setZMin(v1.z); setZMin(v2.z);
		setZMax(v0.z); setZMax(v1.z); setZMax(v2.z);
	}

	void setZMin(float z) {
		if (z < zMin) {
			zMin = z;
		}
	}

	void setZMax(float z) {
		if (z > zMax) {
			zMax = z;
		}
	}

	Triangle& operator-=(const glm::vec3& pt) {
		v[0] -= pt;
		v[1] -= pt;
		v[2] -= pt;
		return *this;
	}

	bool operator<(const Triangle& t) {
		return zMin < t.zMin;
	}

	friend std::ostream& operator<<(std::ostream& os, const Triangle& t) {
		os << "V0: (" << t.v[0].z << "); V1: (" << t.v[1].z << "); V2: (" << t.v[2].z << ")";
		return os;
	}

public:

	glm::f64vec3 v[3];
	glm::f64vec3 normal;
	double zMin;
	double zMax;
};


/*Structures*/
struct Mesh_Triangle_Node_t {
	Triangle t;
	Mesh_Triangle_Node_t* next;
	Mesh_Triangle_Node_t* prev;
};

struct Mesh_Triangle_List_t {
	Mesh_Triangle_Node_t* head;
	Mesh_Triangle_Node_t* tail;
};

class LineSegment {

public:

	LineSegment(glm::vec3 p0 = glm::vec3(), glm::vec3 p1 = glm::vec3(), int i = 0) {
		v[0] = p0;
		v[1] = p1;
		index = i;
		vertical = false;
		if ((v[1].x - v[0].x) != 0) {
			a = (v[1].y - v[0].y) / (v[1].x - v[0].x);
			b = (v[0].y - (a * v[0].x));
		}
		else {
			vertical = true;
		}
	}

	bool operator==(const LineSegment& ls) const {
		return ((v[0] == ls.v[0]) && (v[1] == ls.v[1]));
	}

	friend std::ostream& operator<<(std::ostream& os, const LineSegment& ls) {
		os << "V0: (" << ls.v[0].x << " " << ls.v[0].y << " " << ls.v[0].z << "); V1: (" << ls.v[1].x << " " << ls.v[1].y << " " << ls.v[1].z << ")";
		return os;
	}

public:

	glm::vec3 v[2];
	double a;
	double b;
	bool vertical;
	int index;
};

/* Assumes that {P} is a list of {k} strictly increasing {Z} coordinates.
   Returns an integer {p} such that {P[p-1] < zMin < P[p]}. As special cases,
   if {zMin < P[0]} returns 0; if {zMin > P[k-1]} returns {k}. */
int IncrementalSlicing_binary_search(float zMin, std::vector<float> P) {
	int k = P.size();
	assert(k >= 1);
	if (zMin >= P[k - 1]) { return k; }
	/* Binary search: */
	int l = -1; /* Inferior Z index. */
	int r = k;  /* Superior Z index. */
	while (r - l > 1) {
		/* At this point, {zMin} is between {P[l]} and {P[r]}. */
		int m = (l + r) / 2;
		assert((0 <= m) && (m < k));
		if (zMin >= P[m]) {
			l = m;
		}
		else {
			r = m;
		}
	}
	return r;
}

/**************************************************************************
 *                                  HASH                                  *
 **************************************************************************/
template<typename T> inline void hash_combine(size_t& seed, const T& v) {
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct HashV3 {
	size_t operator() (const glm::f64vec3& v) const {
		size_t h = std::hash<float>()(v.x);
		hash_combine(h, v.y);
		//hash_combine(h, v.z);
		return h;
	}
};

typedef std::unordered_map<glm::f64vec3, std::vector<glm::f64vec3>, HashV3> PointMesh;

/*-----------------------------------------------------------------------*/
Mesh_Triangle_List_t* Mesh_Triangle_List_create(void) {
	Mesh_Triangle_List_t* L = (Mesh_Triangle_List_t*)malloc(sizeof(Mesh_Triangle_List_t));
	L->head = NULL;
	L->tail = NULL;
	return L;
}

/*-----------------------------------------------------------------------*/
void Mesh_Triangle_List_insert(Triangle t, Mesh_Triangle_List_t* L) {
	Mesh_Triangle_Node_t* node = (Mesh_Triangle_Node_t*)malloc(sizeof(Mesh_Triangle_Node_t));
	node->t = t;
	node->next = L->head;
	node->prev = NULL;
	if (L->head == NULL) {
		/*New head*/
		L->head = L->tail = node;
	}
	else {
		L->head->prev = node;
		L->head = node;
	}
}

/*-----------------------------------------------------------------------*/
void Mesh_Triangle_List_union(Mesh_Triangle_List_t* L1, Mesh_Triangle_List_t* L2) {
	if ((L1->head != NULL) && (L2->head != NULL)) {
		L1->tail->next = L2->head;
		L2->head->prev = L1->tail;
		L1->tail = L2->tail;;
	}
	else if (L2->head != NULL) {
		L1->head = L2->head;
		L1->tail = L2->tail;
	}
}

/*-----------------------------------------------------------------------*/
Mesh_Triangle_Node_t* Mesh_Triangle_List_remove(Mesh_Triangle_List_t* L, Mesh_Triangle_Node_t* node) {
	if ((node->prev == NULL) && (node->next == NULL)) {
		free(node);
		L->head = NULL;
		L->tail = NULL;
		return NULL;
	}
	else if (node->prev == NULL) {
		node->next->prev = NULL;
		L->head = node->next;
		free(node);
		return L->head;
	}
	else if (node->next == NULL) {
		node->prev->next = NULL;
		L->tail = node->prev;
		free(node);
		return NULL;
	}
	else {
		Mesh_Triangle_Node_t* next = node->next;
		node->next->prev = node->prev;
		node->prev->next = next;
		free(node);
		return next;
	}
}


glm::f64vec3 R3_Mesh_Side_slice(glm::f64vec3 vi, glm::f64vec3 vj, double Z) {
	double dx = vj.x - vi.x;
	double dy = vj.y - vi.y;
	double dz = vj.z - vi.z;
	assert(dz != 0);
	double frac = (Z - vi.z) / dz;
	double xint = (frac * dx + (double)vi.x);
	double yint = (frac * dy + (double)vi.y);
	//return (glm::vec3){ .x = xint, .y = yint, .z = Z };
	return glm::f64vec3(xint, yint, Z);
}

/*-----------------------------------------------------------------------*/
LineSegment R3_Mesh_Triangle_slice(Mesh_Triangle_Node_t* t, double Z) {
	assert((t->t.zMin < Z) && (t->t.zMax > Z));
	int np = 0; /* Number of segment endpoints found */
	LineSegment seg;
	for (int i = 0; i < 3; i++) {
		/* Get side {i} of triangle: */
		int j = (i == 2 ? 0 : i + 1);
		glm::f64vec3 vi = (t->t.v[i]);
		glm::f64vec3 vj = (t->t.v[j]);
		/* Check for intersection of plane with {vi--vj}. */
		/* Must consider segment closed at bottom and open at top in case {Z} goes through a vertex. */
		double vzMin = (vi.z < vj.z ? vi.z : vj.z);
		double vzMax = (vi.z > vj.z ? vi.z : vj.z);
		if ((vzMin <= Z) && (vzMax > Z)) {
			glm::f64vec3 p = R3_Mesh_Side_slice(vi, vj, Z);
			assert(np < 2);
			seg.v[np] = p;
			np++;
		}
	}
	assert(np == 2);
	return seg;
}

/*-----------------------------------------------------------------------*/
LineSegment R3_Mesh_Triangle_slice(Triangle t, double Z) {
	assert((t.zMin < Z) && (t.zMax > Z));
	int np = 0; /* Number of segment endpoints found */
	LineSegment seg;
	for (int i = 0; i < 3; i++) {
		/* Get side {i} of triangle: */
		int j = (i == 2 ? 0 : i + 1);
		glm::f64vec3 vi = (t.v[i]);
		glm::f64vec3 vj = (t.v[j]);
		/* Check for intersection of plane with {vi--vj}. */
		/* Must consider segment closed at bottom and open at top in case {Z} goes through a vertex. */
		double vzMin = (vi.z < vj.z ? vi.z : vj.z);
		double vzMax = (vi.z > vj.z ? vi.z : vj.z);
		if ((vzMin <= Z) && (vzMax > Z)) {
			glm::f64vec3 p = R3_Mesh_Side_slice(vi, vj, Z);
			assert(np < 2);
			seg.v[np] = p;
			np++;
		}
	}
	assert(np == 2);
	return seg;
}


/*----------------------------------------------------------------------*/
/* Assumes that {P[0..k-1]} is a list of {k} strictly increasing {Z}
  coordinates. Returns a vector of {k+1} lists {L[0..k]} such that {L[p]}
  contains all triangles of the {mesh} that have {zMin} between {P[p-1]}
  and {P[p]}, assuming that {P[-1] = -oo} and {P[k] = +oo}. If {delta > 0},
  assumes that {P[p]-P[p-1] = delta} for {p} in {1..k-1}. If {srt} is true,
  assumes that the triangles are already sorted by increasing {zMin}. */
Mesh_Triangle_List_t** IncrementalSlicing_buildLists(bool srt, double delta, const std::vector<glm::vec3>& tri_pts, std::vector<float> P) {

	int k = P.size(); /* Number of planes. */

	Mesh_Triangle_List_t** L = (Mesh_Triangle_List_t**)malloc((k + 1) * sizeof(Mesh_Triangle_List_t*));

	for (size_t p = 0; p <= k; p++) { L[p] = Mesh_Triangle_List_create(); }

	//auto tri_pts = mesh->geometry->getTriangleList();
	std::vector<Triangle> T;// = mesh->getvTriangle();
	for (auto i = 0; i < tri_pts.size(); i += 3)
	{
		//tri_pts[i] = glm::vec3( mesh->model_world_transform* glm::vec4(tri_pts[i], 1.0));
		//tri_pts[i+1] = glm::vec3(mesh->model_world_transform * glm::vec4(tri_pts[i+1], 1.0));
		//tri_pts[i+2] = glm::vec3(mesh->model_world_transform * glm::vec4(tri_pts[i+2], 1.0));

		Triangle tri(tri_pts[i], tri_pts[i + 1], tri_pts[i + 2]);
		T.emplace_back(tri);
	}

	int n = T.size(); /* Number of triangles. */

	if (delta > 0.0) {
		/* Uniform slicing - compute list index: */
		for (auto it = T.begin(), itEnd = T.end(); it != itEnd; ++it) {
			Triangle t = *it;
			int p;
			if (t.zMin < P[0]) {
				p = 0;
			}
			else if (t.zMin > P[k - 1]) {
				p = k;
			}
			else {
				p = floor((t.zMin - P[0]) / delta) + 1;
			}
			Mesh_Triangle_List_insert(t, L[p]);
		}
	}
	else if (srt) {
		/* Slicing of a pre-sorted mesh - merge {zMin}s and {P}: */
		auto it = T.begin();
		auto itEnd = T.end();
		double zprev = -INFINITY;
		for (int p = 0; p <= k; p++) {
			float Zp = (p < k ? P[k] : +INFINITY);
			const Triangle t = *it;
			assert(t.zMin >= zprev);
			while ((it != itEnd) && (t.zMin < Zp)) {
				Mesh_Triangle_List_insert(t, L[p]);
				zprev = t.zMin;
				it++;
			}
		}
	}
	else {
		/* General case: */
		for (auto it = T.begin(), itEnd = T.end(); it != itEnd; ++it) {
			const Triangle t = *it;
			int p = IncrementalSlicing_binary_search(t.zMin, P);
			assert((p >= 0) && (p <= k));
			Mesh_Triangle_List_insert(t, L[p]);
		}
	}
	return L;
}


/*Rounds x to an integer multiple of {eps}. If {mod} > 1,
  the factor will be congruent to {rem} modulo {mod}. */
float xround(float x, double eps, int mod, int rem) {
	double y = round((double)x / (mod * eps));
	double z = (y * mod + rem) * eps;
	return (float)z;
}

/*-----------------------------------------------------------------------*/
/*Rounds {x,y,z} to an even multiple of {eps}. */
glm::vec3 v3_round(double x, double y, double z, double eps) {
	glm::vec3 p;
	p.x = xround(x, eps, 2, 0);
	p.y = xround(y, eps, 2, 0);
	p.z = xround(z, eps, 2, 0);
	return p;
}

/*Compute uniform and adaptive z-plane coordinates!*/
std::vector<float> compute_planes(const glm::AABB& tri_box, float max_thickness, char* adaptive, double eps, float* delta) {

	bool rounding = true; /*To avoid that the Z-coordinates of all planes are distinct from the Z-coordinates of all vertices.*/

	/* Vector to keep the plane coordinates: */
	std::vector<float> Planes;
	//mesh->updateBoundingBox();

	//auto box = mesh->getWorldBounds();

	auto box = tri_box;
	/* Assuming the model as a 3D axis-aligned bounding-box: */
	double model_zmax = box.getMax().z;// std::max(mesh->getUpperRightVertex().z, mesh->meshAABBSize().z);

	double model_zmin = box.getMin().z;// mesh->getBottomLeftVertex().z;

	if (strcmp(adaptive, "false") == 0) { /*Uniform slicing: */

		double spacing = (rounding ? xround(max_thickness, eps, 2, 0) : max_thickness); /*Plane spacing even multiple of {eps}*/

		double P0 = xround(model_zmin - spacing, eps, 2, 1); /*First plane odd multiple of {eps}.*/

		int no_planes = 1 + (int)((model_zmax + spacing - P0) / spacing); /* Number of planes: */

		std::cout << "eps = " << eps << std::endl;
		std::cout << "max thickness = " << max_thickness << std::endl;
		std::cout << "rounded plane spacing spacing = " << spacing << std::endl;
		std::cout << "model zmin = " << model_zmin << ", model zmax = " << model_zmax << ", first plane Z = " << P0 << ", number of planes = " << no_planes << std::endl;

		for (size_t i = 0; i < no_planes; i++) {
			/* Building the vector with the slice z coordinates: */
			float Pi = (float)(P0 + i * spacing);
			if ((Pi > model_zmin) && (Pi < model_zmax)) {
				Planes.push_back((float)(P0 + i * spacing));
			}
		}
		*delta = (float)(spacing);
	}
	else { /*Adaptive slicing z-planes: */

		float zplane = 0.0;
		float min_thickness = 0.016;
		Planes.push_back(model_zmin + zplane);

		while ((model_zmin + zplane) <= model_zmax) {
			double vrandom = min_thickness + (max_thickness - min_thickness) * (rand() / (double)RAND_MAX);
			double coordinate = xround(model_zmin + zplane + vrandom, eps, 2, 1);
			if (coordinate >= model_zmax) { break; }
			Planes.push_back(coordinate);
			zplane += vrandom;
		}
	}
	return Planes;
}



//defines
long intersections = 0;
double slicing_time = 0.0, loopclosure_time = 0.0;

/*-----------------------------------------------------------------------*/
/*Gets an arbitrary segment from {H}, removes it from {H} and returns it as a trivial chain. */
std::vector<glm::f64vec3> IncrementalStartLoop(std::vector<PointMesh>& H) {
	std::vector<glm::f64vec3> P;
	auto it = (H[0]).begin();
	glm::f64vec3 u = (*it).first;
	std::vector<glm::f64vec3> vw = (*it).second;
	glm::f64vec3 v = vw.at(0);
	P.push_back(u);
	P.push_back(v);
	(H[0][u]).erase(std::remove((H[0][u]).begin(), (H[0][u]).end(), v), (H[0][u]).end());
	if (H[0][u].size() == 0) { (H[0]).erase(u); }
	(H[0][v]).erase(std::remove((H[0][v]).begin(), (H[0][v]).end(), u), (H[0][v]).end());
	if (H[0][v].size() == 0) { (H[0]).erase(v); }
	return P;
}

/*-----------------------------------------------------------------------*/
/*Extends the chain {P} wih segments from {H}, removing them, while possible. */
void IncrementalExtendLoop(std::vector<glm::f64vec3>& P, std::vector<PointMesh>& H) {
	int index = 0;
	int n = P.size();
	glm::f64vec3 first = P.front();
	glm::f64vec3 current = P.back();
	glm::f64vec3 last;

	/* Collect other vertices: */
	while (true) {
		auto it = (H[0]).find(current);
		if (it == (H[0]).end()) { /*Vertex {current} is a dead end:*/ break; }
		glm::f64vec3 key1 = (*it).first; assert(key1 == current);  /*Paranoia check.*/

		/*Get {next}, the first unused neighbor of {current}:*/
		std::vector<glm::f64vec3> vw = (*it).second; /*Unused neighbors of {current}.*/
		assert(vw.size() != 0);
		glm::f64vec3 next = vw.at(0); /*First unused neighbor of {current}.*/

		/*Append the segment {(current,next)} to {P} and delete from {H}:*/
		P.push_back(next);

		/*Remove the segment {(current,next)} from {H}:*/
		(H[0][current]).erase(std::remove((H[0][current]).begin(), (H[0][current]).end(), next), (H[0][current]).end());
		if (H[0][current].size() == 0) { (H[0]).erase(current); }
		(H[0][next]).erase(std::remove((H[0][next]).begin(), (H[0][next]).end(), current), (H[0][next]).end());
		if (H[0][next].size() == 0) { (H[0]).erase(next); }

		if (next == first) { /*We closed a loop:*/ break; }

		/*Move on:*/
		current = next;
	}
}

/*Reverses the chain {P}.*/
void IncrementalReverseLoop(std::vector<glm::f64vec3>& P) {
	std::reverse(P.begin(), P.end());
}
/*-----------------------------------------------------------------------
  Groups the segments {segs} into zero or more open or closed polygonal chains.
A chain with {n} segments has {n+1} vertices (of type {glm::vec3}). There may be
repeated vertices.  The chain is closed iff the last vertex is equal to the
first. Vertices have their {x} and {y} coordinates rounded to a multiple of a
certain {eps}. The output will have one open chain for every pair of vertices
of odd degree. The rest of the segments will be grouped into closed chains.
All chains are returned in the {Polygons} list. */
void ContourConstruction(std::vector<LineSegment>& segs, std::vector<std::vector<Contour>>& polygons, int plane,float planeZ) {

	bool verbose = false;

	clock_t contour_begin = clock();

	/*Creating the hash table.*/
	std::vector<PointMesh> H(1);

	/*Rounding vertices and filling the hash table.*/
	double eps = 1 / 128.0;
	for(auto i = segs.begin(); i != segs.end(); i++) {
		LineSegment q = *i;
		q.v[0].x = round(q.v[0].x / eps) * eps;
		q.v[0].y = round(q.v[0].y / eps) * eps;
		q.v[0].z = planeZ;
		q.v[1].x = round(q.v[1].x / eps) * eps;
		q.v[1].y = round(q.v[1].y / eps) * eps;
		q.v[1].z = planeZ;
		if (glm::distance(q.v[0],q.v[1]) > 0.0001) {
			(H[0][q.v[0]]).push_back(q.v[1]);
			(H[0][q.v[1]]).push_back(q.v[0]);
		}
	}

	/* Count vertices by degree: */
	if (verbose) {
		const int degmax = 10;
		int ctdeg[degmax + 1];
		for (int deg = 0; deg <= degmax; deg++) { ctdeg[deg] = 0; }
		for (auto i = (H[0]).begin(); i != (H[0]).end(); i++) {
			std::vector<glm::f64vec3> L = (*i).second;
			int deg = L.size();
			if (deg > degmax) { deg = degmax; }
			ctdeg[deg]++;
		}
		assert(ctdeg[0] == 0);
		bool closedSlice = true;
		for (int deg = 1; deg <= degmax; deg++) {
			if (((deg % 2) != 0) && (ctdeg[deg] > 0)) { closedSlice = false; }
			if ((verbose || (deg != 2)) && (ctdeg[deg] != 0))
			{
				std::cout << "there are " << ctdeg[deg] << " vertices of degree " << deg << " on plane " << plane << std::endl;
			}
		}
		if (!closedSlice) { std::cout << "** contours of plane " << plane << " are not closed" << std::endl; }
	}

	/*Contour construction.*/
	bool maximal = true;
	while(!(H[0]).empty()) {
		if (maximal) {
			std::vector<glm::f64vec3> P = IncrementalStartLoop(H);
			IncrementalExtendLoop(P, H);
			if (P.front() != P.back()) { //Chain {P} is open
				IncrementalReverseLoop(P);
				IncrementalExtendLoop(P, H);
			}
			polygons[plane].push_back({ false, false, P });
		}
		else {
			std::vector<glm::f64vec3> P = IncrementalStartLoop(H);
			IncrementalExtendLoop(P, H);
			polygons[plane].push_back({ false, false, P });
		}
	}
	clock_t contour_end = clock();
	loopclosure_time += double(contour_end - contour_begin) / CLOCKS_PER_SEC;
}

/*-----------------------------------------------------------------------*/
bool is_inside(LineSegment line, glm::vec3 point) {
	double maxX = (line.v[0].x > line.v[1].x) ? line.v[0].x : line.v[1].x;
	double minX = (line.v[0].x < line.v[1].x) ? line.v[0].x : line.v[1].x;
	double maxY = (line.v[0].y > line.v[1].y) ? line.v[0].y : line.v[1].y;
	double minY = (line.v[0].y < line.v[1].y) ? line.v[0].y : line.v[1].y;
	if ((point.x >= minX && point.x <= maxX) && (point.y >= minY && point.y <= maxY)) {
		return true;
	}
	return false;
}


/*-----------------------------------------------------------------------*/
bool ray_intersect(LineSegment ray, LineSegment side) {
	glm::vec3 intersectPoint;
	/* If both vectors aren't from the kind of x=1 lines then go into: */
	if (!ray.vertical && !side.vertical) {
		/* Check if both vectors are parallel. If they are parallel then no intersection point will exist: */
		if (ray.a - side.a == 0) {
			return false;
		}
		intersectPoint.x = ((side.b - ray.b) / (ray.a - side.a));
		intersectPoint.y = side.a * intersectPoint.x + side.b;
	}
	else if (ray.vertical && !side.vertical) {
		intersectPoint.x = ray.v[0].x;
		intersectPoint.y = side.a * intersectPoint.x + side.b;
	}
	else if (!ray.vertical && side.vertical) {
		intersectPoint.x = side.v[0].x;
		intersectPoint.y = ray.a * intersectPoint.x + ray.b;
	}
	else {
		return false;
	}
	if (is_inside(side, intersectPoint) && is_inside(ray, intersectPoint)) {
		return true;
	}
	return false;
}

/*-----------------------------------------------------------------------*/
LineSegment create_ray(glm::vec3 point, bounding_box bb, int index) {
	/* Create outside point: */
	double epsilon = (bb.xMax - bb.xMin) / 100.0;
	glm::vec3 outside(bb.xMin - epsilon, bb.yMin,0);
	LineSegment v(outside, point, index);
	return v;
}


/*-----------------------------------------------------------------------*/
void update_bounding_box(glm::vec3 point, bounding_box* bb) {
	/* Setting the bounding box: */
	if (point.x > bb->xMax) {
		bb->xMax = point.x;
	}
	else if (point.x < bb->xMin) {
		bb->xMin = point.x;
	}
	if (point.y > bb->yMax) {
		bb->yMax = point.y;
	}
	else if (point.y < bb->yMin) {
		bb->yMin = point.y;
	}
}

/*-----------------------------------------------------------------------*/
bool insided_bounding_box(glm::vec3 point, bounding_box bb) {
	if ((point.x < bb.xMin) || (point.x > bb.xMax) || (point.y < bb.yMin) || (point.y > bb.yMax)) {
		return false;
	}
	return true;
}

/*-----------------------------------------------------------------------*/
bool contains(glm::vec3 point, bounding_box bb, std::vector<LineSegment> sides, int index) {
	if (insided_bounding_box(point, bb)) {
		LineSegment ray = create_ray(point, bb, index);
		int intersection = 0;
		for (int i = 0; i < sides.size(); i++) {
			if ((sides.at(i).index != index) && ray_intersect(ray, sides.at(i))) {
				intersection++;
			}
		}
		/* If the number of intersections is odd, then the point is inside the polygon: */
		if ((intersection % 2) == 1) {
			return true;
		}
	}
	return false;
}
/*-----------------------------------------------------------------------*/
void add_point(glm::vec3 p1, glm::vec3 p2, std::vector<LineSegment>& t, bounding_box* bb, bool first, int index) {
	if (first) {
		update_bounding_box(p1, bb);
	}
	update_bounding_box(p2, bb);
	LineSegment line(p1, p2, index);
	t.push_back(line);
}

/*Function to (re)orient the contour clockwise and counter-clockwise.*/
void ray_casting(std::vector<Contour>& polygons) {

	std::vector<LineSegment> segments;

	bounding_box bb = create_bounding_box();

	/*Creating the line segments of each contour: */
	for (int i = 0; i < polygons.size(); i++) {
		double area = 0.0;
		std::vector<glm::f64vec3> Pi = polygons.at(i).points;
		for (int j = 1; j < Pi.size(); j++) {
			glm::f64vec3 p0 = Pi.at(j - 1);
			glm::f64vec3 p1 = Pi.at(j + 0);
			area += (p0.x * p1.y - p0.y * p1.x);
			add_point(p0, p1, segments, &bb, (j == 1 ? true : false), i);
			if (j == Pi.size() - 1) {
				add_point(p1, Pi.at(0), segments, &bb, (j == 1 ? true : false), i);
				area += (p1.x * Pi.at(0).y - p1.y * Pi.at(0).x);
			}
		}
		area /= 2.0;
		if (area < 0.0) {
			polygons.at(i).clockwise = true;
		}
		else {
			polygons.at(i).clockwise = false;
		}
	}

	/*Using the point in polygon algorithm to test the first segment of each contour: */
	for (int i = 0; i < polygons.size(); i++) {
		std::vector<glm::f64vec3> Pi = polygons.at(i).points;
		if (contains(Pi.at(0), bb, segments, i)) {
			/*Internal contour: */
			polygons.at(i).external = false;
		}
		else {
			/*External contour: */
			polygons.at(i).external = true;
		}

		/*Reversing contours: */
		if (polygons.at(i).external && polygons.at(i).clockwise) {
			std::reverse(polygons.at(i).points.begin(), polygons.at(i).points.end());
			polygons.at(i).clockwise = false;
		}
		else if (!polygons.at(i).external && !polygons.at(i).clockwise) {
			std::reverse(polygons.at(i).points.begin(), polygons.at(i).points.end());
			polygons.at(i).clockwise = true;
		}
	}
	segments.clear();
}


void IncrementalSlicing(const std::vector<glm::vec3>& tri_pts, std::vector<float> P, float delta, bool srt, std::vector<std::vector<Contour>>& polygons, bool chaining, bool orienting) {

	/*Slicing*/
	clock_t slice_begin = clock();

	int k = P.size();

	std::vector<std::vector<LineSegment>> segs(k);//[k];

	/* Classify triangles by the plane gaps that contain their {zMin}: */
	Mesh_Triangle_List_t** L = IncrementalSlicing_buildLists(srt, delta, tri_pts, P);
	/* Now perform a plane sweep from bottom to top: */

	Mesh_Triangle_List_t* A = Mesh_Triangle_List_create(); /* Active triangle list. */
	for (int p = 0; p < k; p++) {
		/* Add triangles that start between {P[p-1]} and {P[p]}: */
		Mesh_Triangle_List_union(A, L[p]);
		/* Scan the active triangles: */
		Mesh_Triangle_Node_t* aux = A->head;
		while (aux != NULL) {
			Mesh_Triangle_Node_t* next = aux->next;
			if (aux->t.zMax < P[p]) {
				/* Triangle is exhausted: */
				Mesh_Triangle_List_remove(A, aux);
			}
			else {
				/* Compute intersection: */
				if ((aux->t.zMin < P[p]) && (aux->t.zMax > P[p])) {
					LineSegment seg = R3_Mesh_Triangle_slice(aux, P[p]);
					segs[p].push_back(seg);
					intersections++;
				}
			}
			aux = next;
		}
	}
	free(L);
	clock_t slice_end = clock();
	slicing_time = double(slice_end - slice_begin) / CLOCKS_PER_SEC;
	/*End-Slicing*/

	if (chaining) {
		/*Contour construction:*/
		for (size_t p = 0; p < k; p++) {
			if (!segs[p].empty()) {
				ContourConstruction(segs[p], polygons,p, P[p]);
#ifdef DEBUG
				char fname[256];
				sprintf(fname, "slice_%03d.txt", (int)p);
				FILE* fslice = fopen(fname, "w");
				fprintf(fslice, "----------- Segmentos ---------------\n");
				for (int ii = 0; ii < segs[p].size(); ii++) {
					LineSegment ss = segs[p].at(ii);
					fprintf(fslice, "%f %f   %f %f\n", ss.v[0].x, ss.v[0].y, ss.v[1].x, ss.v[1].y);
				}
				fprintf(fslice, "---------- End Segmentos --------------\n");
				record_polygons(polygons[p], fslice);
				fclose(fslice);
#endif
				if (orienting) {
					ray_casting(polygons[p]);
				}
				segs[p].clear();
			}
		}
		/*End construction.*/
	}

}

std::vector<std::vector<ContourPoint>> mergeLineSegments(const std::vector<std::vector<ContourPoint>>& linesegments)
{
	//set input
	std::vector<LineSegment> segs;
	// 保存ContourPoint的映射，用于后续恢复
	std::map<std::pair<int, int>, ContourPoint> pointMap;
	
	int i_idx = 0;
	for (int segIdx = 0; segIdx < linesegments.size(); segIdx++)
	{
		auto& lineseg = linesegments[segIdx];
		for (int i = 0; i < lineseg.size() - 1; i++)
		{
			// 使用Position的xyz进行计算
			glm::vec3 p0(lineseg[i].Position.x, lineseg[i].Position.z, lineseg[i].Position.y);
			glm::vec3 p1(lineseg[i + 1].Position.x, lineseg[i + 1].Position.z, lineseg[i + 1].Position.y);
			
			LineSegment seg(p0, p1, i_idx++);
			segs.emplace_back(seg);
			
			// 保存原始ContourPoint信息
			pointMap[{segIdx, i}] = lineseg[i];
			pointMap[{segIdx, i + 1}] = lineseg[i + 1];
		}
	}

	//process
	std::vector<std::vector<Contour>> polygons(1);
	ContourConstruction(segs, polygons, 0,0.0f);

	//output - 将结果转换回ContourPoint
	std::vector<std::vector<ContourPoint>> outSegs;
	for (auto& contours : polygons)
	{
		for (auto& contour : contours)
		{
			std::vector<ContourPoint> contourPoints;
			for (auto& pt : contour.points)
			{
				ContourPoint cp;
				cp.Position = glm::vec4(pt.x, pt.z, pt.y, 0);
				
				// 尝试从原始数据中查找最接近的点来恢复法线信息
				float minDist = std::numeric_limits<float>::max();
				for (auto& pair : pointMap)
				{
					glm::f64vec3 origPos(pair.second.Position.x, pair.second.Position.z, pair.second.Position.y);
					float dist = glm::distance(pt, origPos);
					if (dist < minDist)
					{
						minDist = dist;
						cp.Normal = pair.second.Normal;
					}
				}
				
				contourPoints.push_back(cp);
			}
			outSegs.emplace_back(contourPoints);
		}
	}

	return outSegs;
}

std::map<int, std::vector<std::vector<glm::vec3>> > LoadModelAndMakeSlices(const std::string& filepath,const glm::vec3& normal,float heightstep)
{
	//1 load model with assimp -> std::vector<glm::vec3> tri_pts 
	std::vector<glm::vec3> tri_pts;
	Assimp::Importer importer;

	auto fullpath = std::string(APP_ROOT_PATH) + "/"+ filepath;
	const aiScene* scene = importer.ReadFile(fullpath, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
	if (!scene || !scene->HasMeshes()) {
		std::cerr << "Error loading model: " << importer.GetErrorString() << std::endl;
		return std::map<int, std::vector<std::vector<glm::vec3>> >();
	}
	for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
		aiMesh* mesh = scene->mMeshes[m];
		// Extract vertices
		std::vector<glm::vec3> tri_local_pts;
		for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
			aiVector3D vertex = mesh->mVertices[v];
			tri_local_pts.emplace_back(glm::vec3(vertex.x, vertex.y, vertex.z));
		}
		// Extract indices
		for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
			aiFace face = mesh->mFaces[f];
			for (unsigned int i = 0; i < face.mNumIndices; ++i) {
				//tri_indices.push_back(face.mIndices[i] + tri_pts.size() - mesh->mNumVertices); // Adjust index based on current vertex count
				auto tri_pt = tri_local_pts[face.mIndices[i]];
				tri_pts.emplace_back(tri_pt);
			}
		}
	}


	//2 slice triangles

	auto tri_pts_all = transformPointsToBasis(tri_pts, normal);
	glm::AABB tri_pts_box;
	for (auto pt : tri_pts_all)
	{
		tri_pts_box.extend(pt);
	}

	double eps = 0.0004;
	float delta = 0.0;
	auto planes = compute_planes(tri_pts_box, heightstep, "false", eps, &delta);

	int nplanes = planes.size();
	std::vector<std::vector<Contour>> polygons(nplanes);
	bool srt = false;
	bool chaining = true;
	bool orienting = true;
	IncrementalSlicing(tri_pts_all, planes, delta, srt, polygons, chaining, orienting);

	printf("polygon size:%d\n", polygons.size());

	auto slice_contours = polygons;
	
	std::map<int, std::vector<std::vector<glm::vec3>>> positions_all;
	for (int i = 0; i < slice_contours.size(); i++)
	{
		float real_height = i * heightstep;
		auto& cur_slice = slice_contours[i];
		std::vector < std::vector<glm::vec3>> contours;
		for (auto& contour : cur_slice)
		{
			if (contour.clockwise)//default is counter-clockwise,wont enter this if
			{
				//reverse contour points
				std::reverse(contour.points.begin(), contour.points.end());
				contour.clockwise = false;
			}

			std::vector<glm::vec3> positions;
			for (int j = 0; j < contour.size(); j++)
			{
				positions.emplace_back(contour.points[j]);
				//positions.emplace_back(contour.points[(j + 1) % contour.size()]);
			}
			positions = transformPointsBasisBack(positions, normal);

			contours.push_back(positions);
		}

		positions_all[i] = contours;
	}

	return positions_all;
}
