#include "RoughPath.h"
#include "clipper2/include/clipper2/clipper.h"
#include <fstream>
#include <glm/gtc/constants.hpp>

namespace cp = Clipper2Lib;

#pragma region utility

void writeLinesToOBJ(const std::string& filename, 
                     const std::vector<std::vector<glm::vec3>>& lines)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        return;
    }
    
    // Write vertices first
    int vertexIndex = 1; // OBJ indices are 1-based
    std::vector<int> lineStartIndices; // Store starting vertex index for each line
    
    for (const auto& line : lines) {
        lineStartIndices.push_back(vertexIndex);
        for (const auto& point : line) {
            file << "v " << point.x << " " << point.y << " " << point.z << "\n";
            vertexIndex++;
        }
    }
    
    // Calculate and write normals
    int normalIndex = 1;
    std::vector<int> lineNormalStartIndices; // Store starting normal index for each line
    
    for (const auto& line : lines) {
        lineNormalStartIndices.push_back(normalIndex);
        
        for (size_t i = 0; i < line.size(); ++i) {
            glm::vec3 normal(1.0f,0.0f, 0.0f); // Default normal (upwards)
            
            if (line.size() > 1) {
                //pre->cur point
				glm::vec3 pre = line[(i - 1+line.size())%line.size()];
				glm::vec3 cur = line[i];
				auto dir1 = glm::normalize(cur - pre);
                //next -> cur point
				glm::vec3 next = line[(i + 1) % line.size()];
				auto dir2 = glm::normalize(next - cur);
				auto up = glm::vec3(0.0f, 1.0f, 0.0f);
				auto nor1 = glm::normalize(glm::cross(dir1, up));
				auto nor2 = glm::normalize(glm::cross(dir2, up));
				normal =  glm::normalize(nor1 + nor2);
                // Normalize normal
                //normal = glm::normalize(normal);
            }
            
            file << "vn " << normal.x << " " << normal.y << " " << normal.z << "\n";
            normalIndex++;
        }
    }
    
    // Write line indices with normals
    int lineIdx = 0;
    for (const auto& line : lines) {
        file << "l";
        int startIdx = lineStartIndices[lineIdx];
        int normalStartIdx = lineNormalStartIndices[lineIdx];
        for (size_t i = 0; i < line.size(); ++i) {
            file << " " << (startIdx + i) << "//" << (normalStartIdx + i);
        }
        file << "\n";
        lineIdx++;
    }
    
    file.close();
}
#pragma endregion

void RoughPathRotate::Plan()
{

}


cp::PathsD ToClipperPath(const std::vector < std::vector<glm::vec3>>& contours,float heightY = 0.f)
{
	cp::PathsD borderPath;
	//height = cursclice_contours[0][0].y;
	for (int i = 0; i < contours.size(); i++)
	{
		std::vector<double> inputline;
		std::for_each(contours[i].begin(), contours[i].end(), [&inputline](const glm::vec3& v) {
			inputline.emplace_back(double(v.x));
			inputline.emplace_back(double(v.z));
			});
		
		auto line = cp::MakePathD(inputline);
		if (cp::IsPositive(line))
			std::reverse(line.begin(), line.end());
			//skip
			//continue;

		borderPath.emplace_back(line);
	}
	return borderPath;
}
void RoughPathRotate::Test()
{
	if (!cursclice_contours.size())
		return;

	//1 load model and slice it
	//auto slices = /**/;

	//2 offset contours in each slice
	//std::vector<std::vector<glm::vec3>> cursclice_contours;
	
	//writeLinesToOBJ("roughpath_input.obj", cursclice_contours);
	
	//get center of cursclice_contours
	glm::f64vec3 center(0.0f);
	for (int i = 0; i < cursclice_contours.size(); i++)
	{
		for (int j = 0; j < cursclice_contours[i].size(); j++)
			center += cursclice_contours[i][j];
	}
	center /= (cursclice_contours.size() * cursclice_contours[0].size());
	float firstRoughPathMaxDistance2Center = 0.0f;

	float height = 0.0f;
#pragma region clipper offset

	//2.1 create clipper path
	height = cursclice_contours[0][0].y;
	cp::PathsD borderPath = ToClipperPath(cursclice_contours, height);

	//2.2 offset path
	std::vector<std::vector<glm::vec3>> result;
	double resampleDist = 0.01;
	if (borderPath.size())
	{

		cp::PathsD solution = borderPath;
		//generate contour
		for (int k = 1; k <= MaxOffsetCount; k++)
		{
			//EndJointType: polygon(round on single end)
			//Round:(round on both ends)double side
			solution = cp::InflatePaths(solution, OffsetLength,
				cp::JoinType::Round, cp::EndType::Polygon, 2.0,4);

			solution = cp::SimplifyPaths(solution, resampleDist);

			//assert(solution.size());
		
			if (solution.size())
			{
				//update firstRoughPathMaxDistance2Center
				if(firstRoughPathMaxDistance2Center == 0.0f)
				{
					//first rough path
					float maxDistance2Center = 0.0f;
					for (int i = 0; i < solution.size(); i++)
					{
						for (int j = 0; j < solution[i].size(); j++)
						{
							glm::f64vec3 testpt(solution[i][j].x,height,solution[i][j].y);
							float distance2Center = glm::distance(testpt, center);
							if (distance2Center > maxDistance2Center)
								maxDistance2Center = distance2Center;
						}
					}
					if (maxDistance2Center > firstRoughPathMaxDistance2Center)
						firstRoughPathMaxDistance2Center = maxDistance2Center;
				}
				else
				{
					//other rough path
					//1 circle - solution path
					double R = OffsetLength * (k - 2) + firstRoughPathMaxDistance2Center;//R
					//align R to int offsetLength
					R = int(R / OffsetLength) * OffsetLength;


					double arc_angle = 1 / R;//1mm
					//create an circle
					std::vector<glm::vec3> circleLines;
					for (double i = 0; i < glm::pi<double>() * 2.0; i+= arc_angle)
					{
						auto offset = glm::f64vec3(cos(i),0,sin(i))*R;
						auto pt = center + offset;
						circleLines.emplace_back(pt);
					}
					//create clipper path
					auto circlePath = ToClipperPath({ circleLines }, height);
				
					//2 clip path
					auto& clipPath = solution;

					//2.1 check if all knife point is outside of first rough path
					bool skipIntersect = true;
					for (int i = 0; i < solution.size(); i++)
					{
						for (int j = 0; j < solution[i].size(); j++)
						{
							glm::f64vec3 testpt(solution[i][j].x, height, solution[i][j].y);
							float distance2Center = glm::distance(testpt, center);
							if (distance2Center < firstRoughPathMaxDistance2Center)
							{
								skipIntersect = false;
								break;
							}
						}
						if (!skipIntersect)break;
					}

					if (skipIntersect)
					{
						solution = circlePath;
					}
					else
					{
						//3 circle path intersect with clip path
						cp::PathsD intersectPaths = cp::Intersect(circlePath, clipPath, cp::FillRule::NonZero);

						//4 update solution
						solution = intersectPaths;// circlePath;/*debug*/
					}
				}
				
				//find max
				int maxid = 0;
				int maxnum = 0;
				for (int i = 0; i < solution.size(); i++)
					if (solution[i].size() > maxnum)
					{
						maxnum = solution[i].size();
						maxid = i;
					}
				//new contour
				//parse solution to connected points
				for (auto& contour : solution)
				{
					if(cp::IsPositive(contour))
						std::reverse(contour.begin(), contour.end());
					//if (contour.size() < 3)
					//	continue;
					std::vector<glm::vec3> onepath;
					for (int i = 0; i < contour.size(); i++)
					{
						glm::vec3 pt(contour[i].x, height,contour[i].y);
						onepath.emplace_back(pt);
					}
					result.emplace_back(onepath);
				}
			}
			else
				break;
		}
	}

#pragma endregion
	
	//debug
	writeLinesToOBJ("roughpath.obj", result);
}