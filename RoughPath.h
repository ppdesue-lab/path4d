#ifndef _ROUGH_PATH_H_
#define _ROUGH_PATH_H_

#include <vector>
#include <glm/glm.hpp>

//simple offset method
class RoughPath
{
public:

};

class RoughPathRotate :public RoughPath
{
public:
	void Plan();

	void Test();

	int MaxOffsetCount = 10;
	double OffsetLength = 1.0;//mm
	

	//test data
	std::vector<std::vector<glm::vec3>> cursclice_contours;
};

#endif