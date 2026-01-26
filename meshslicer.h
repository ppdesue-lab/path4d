#ifndef _MESH_SLICER_H_
#define _MESH_SLICER_H_

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <map>

#include "MathHelper.hpp"

class MyPoint
{
public:
	glm::f64vec3 pt;
	glm::vec3 normal;

	MyPoint() {}

	double x() { return pt.x; };
	double y() { return pt.y; };
	double z() { return pt.z; };

	bool operator==(const MyPoint& other) const
	{
		return pt == other.pt;
	}
	bool operator!=(const MyPoint& other) const
	{
		return pt != other.pt;
	}
};

struct Contour {
	bool external; // not used
	bool clockwise;// not used
	std::vector<MyPoint> points;

	void clear() { points.clear(); }

	unsigned int size() { return points.size(); }

};


std::map<int, std::vector<std::vector<glm::vec3>> > LoadModelAndMakeSlices(const std::string& filepath,const glm::vec3& normal,float heightstep=0.1f);


extern std::vector<std::vector<ContourPoint>> mergeLineSegments(const std::vector<std::vector<ContourPoint>>& linesegments);

#endif