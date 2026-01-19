#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <map>
#include <math.h>

struct MDS
{
    //counter-clockwise direction
    //0~360degree
    // +x axis represents start angle 0
    float start;
    float end;

    MDS(float s=0, float e=0)
    {
        start = s;
        end = e;
    }

    float Range()
    {
        if(end < start)
            return (360 - start) + end;
        return end - start;
    }

    bool Intersects(const MDS& other) const
    {
        return !(end < other.start || start > other.end);
    }
};

struct MDSContour
{
	std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    std::vector<std::vector<MDS>> MDSs;

    void fromPoints(const std::vector<glm::vec3>& pts)
    {
        //the first point == the last point,need remove it
        
        for (int i = 0; i < pts.size()-1; i++)
        {
            if(points.size())
                if(points[points.size() - 1] == pts[i])
					continue;

			points.emplace_back(pts[i]);
        }
        if(pts[pts.size()-1] != pts[0])
			points.emplace_back(pts[pts.size() - 1]);

        int count = (int)points.size();
        for(int i=0;i< count;i++)
        {
			auto dir = glm::normalize(points[(i + 1) % count] - points[i]);
			auto normal = glm::normalize(glm::vec3(-dir.z, 0, dir.x));


            auto dir2 = glm::normalize(-points[(i - 1+ count) % count] + points[i]);
            auto normal2 = glm::normalize(glm::vec3(-dir.z, 0, dir.x));

			auto avg_normal = glm::normalize(normal + normal2);
            normals.emplace_back(avg_normal);
		}
	}
};

typedef std::vector<MDSContour> MDSContours;

extern MDSContours computeMDSContours(const std::vector<std::vector<glm::vec3>>& contours);
//y is i[
//xz plane contour
class Tool
{

public:
    //ball end mill radius
    float radius = 1.0f;//mm
    //knife length
    float height = 10.0f;//mm
    //location
    glm::vec3 Position = glm::vec3(0,0,0);
	glm::vec3 LenDir = glm::vec3(0, 0, 1);


    //construct ball end & cylinder height
    Tool(float r=1.0f,float height=10.0f)
    {
        radius = r;
        this->height = height;
    }

    void SetPosition(const glm::vec3& pos)
    {
        Position = pos;
	}

    void SetRotation(const glm::vec3& dir)
    {
        this->LenDir = glm::normalize(dir) * height;
    }

    void SetRotationAngle(float angleRadian)
    {
        glm::vec3 dir(cosf(angleRadian), 0, sinf(angleRadian));
        this->LenDir = dir * height;
    }

    glm::vec3 GetTopPosition()
    {
        return Position + LenDir;
	}

    bool isPointReachable(const glm::vec3& point) const
    {
        //considering ball end mill only
        auto to_start = glm::vec2(point.x - Position.x, point.z - Position.z);
       
        float dist = glm::length(to_start);
        if(dist <= radius)
            return true;

        auto dir_along = glm::vec2(LenDir.x, LenDir.z);
		auto dir_perp = glm::vec2(-dir_along.y, dir_along.x);

		float to_centerline = glm::dot(to_start, glm::normalize(dir_perp));
		float to_along = glm::dot(to_start, glm::normalize(dir_along));
        //test cylinder part
		if (abs(to_centerline) <= radius && to_along >= 0 && to_along <= height)
            return true;
        return false;
    } 

};

class Pipe
{
public:
    Pipe() {};



    void initTool(float radius, float height)
    {
		tool = Tool(radius, height);
	}

    void drawAndSaveCanvas(const std::map<int, MDSContours>& positions_all, int idx = -1);

    void CalMDSForEachSlice(std::map<int, MDSContours>& positions_all);


    Tool tool;
};