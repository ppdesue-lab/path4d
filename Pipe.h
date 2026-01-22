#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <map>
#include <math.h>
#include <iostream>
#include <optional>

struct MDS
{
    //counter-clockwise direction
    //0~360degree
    // +x axis represents start angle 0
    float start;
    float end;

    MDS(float s=0, float e=0)
    {
        start = fmodf(s +360,360);
        end = fmodf(e + 360, 360);
    }

    float Range() const
    {
        if(end < start)
            return (360 + end) - start;
        return end - start;
    }

    bool Intersects(const MDS& other) const
    {
        //return !(end < other.start || start > other.end);

        //intersect angles(may be circular)
        if (start <= end) //normal
        {
            if (other.start <= other.end) //normal
            {
                return !(end < other.start || start > other.end);
            }
            else //other circular
            {
                return !(end < other.start && start > other.end);
            }
        }
        else //circular
        {
            if (other.start <= other.end) //normal
            {
                return !(other.end < start && other.start > end);
            }
            else //other circular
            {
                return true; //always intersect
            }
        }
    }

    std::optional<MDS> IntersectsRange(const MDS& other) const
    {
        if (this->Intersects(other))
        {
            float new_start,new_end;
            //get intersection range(0~360,maybe circular)
            if (start <= end) //normal
            {
                if (other.start <= other.end) //normal
                {
                    new_start = std::max(start, other.start);
                    new_end = std::min(end, other.end);
                }
                else //other circular
                {
                    if (start <= other.end)
                    {
                        new_start = start;
                        new_end = other.end;
                    }
                    else //end >= other.start
                    {
                        new_start = other.start;
                        new_end = end;
                    }
                }
            }
            else //circular
            {
                if (other.start <= other.end) //normal
                {
                    if (other.start <= end)
                    {
                        new_start = other.start;
                        new_end = end;
                    }
                    else //start <= other.end
                    {
                        new_start = start;
                        new_end = other.end;
                    }
                }
                else //other circular
                {
                    new_start = std::max(start, other.start);
                    new_end = std::min(end, other.end);
                }
            }


            return MDS(new_start, new_end);
        }
        return std::nullopt; //no intersection
    }

    std::optional<MDS> IntersectsRange(const std::vector<MDS>& others) const
    {
        MDS max_intersection(0,0);
        for (const auto& other : others)
        {
            auto intersection = IntersectsRange(other);
            if (intersection)
            {
                if (intersection->Range() > max_intersection.Range())
                    max_intersection = *intersection;
            }
        }
        if (max_intersection.Range() > 0)
            return max_intersection;
        return std::nullopt; //no intersection
    }

    std::optional<MDS> IntersectsRangeReturnSelected(const std::vector<MDS>& others) const
    {
        MDS max_intersection(0, 0);
        MDS selectedMDS;
        for (const auto& other : others)
        {
            auto intersection = IntersectsRange(other);
            if (intersection)
            {
                if (intersection->Range() > max_intersection.Range())
                {
                    max_intersection = *intersection;
                    selectedMDS = other;
                }
            }
        }
        if (max_intersection.Range() > 0)
            return selectedMDS;
        return std::nullopt; //no intersection
    }


    float GetMidAngle() const
    {
        if (end >= start)
            return (start + end) / 2.0f;
        else
            return (start + (end + 360.0f)) / 2.0f >= 360.0f ? (start + (end + 360.0f)) / 2.0f - 360.0f : (start + (end + 360.0f)) / 2.0f;
	}

    glm::vec2 GetMidNormalizedDir() const
    {
        float midAngle = GetMidAngle();
        float rad = glm::radians(midAngle);
        return glm::vec2(cosf(rad), sinf(rad));
    }
};

struct MDSSegment
{
    int ID_Start;
    int ID_End;
    bool isConnected = false; // for cross-contour connections
    int otherContourID = -1; // for cross-contour, the ID of the other contour
    int otherPointID = -1; // for cross-contour, the point ID in the other contour

    bool operator == (const MDSSegment& other) const
    {
        return (ID_Start == other.ID_Start && ID_End == other.ID_End);
	}
};
struct MDSContour
{
	std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    std::vector<std::vector<MDS>> MDSs;
    std::vector<MDS> selectMDS;

	std::vector<MDSSegment> MDSSegments;

    void fromPoints(const std::vector<glm::vec3>& pts)
    {
        //the first point == the last point,need remove it
        
        for (int i = 0; i < pts.size(); i++)
        {
            if(points.size())
                if(points[points.size() - 1] == pts[i])
					continue;

			points.emplace_back(pts[i]);
        }
        if (pts[pts.size() - 1] == pts[0])
            points.pop_back();// (pts[pts.size() - 1]);

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

    void GreedySearchMDS()
    {
        //resize 
        int countur_count = (int)points.size();
        selectMDS.resize(countur_count);
        std::vector<char> visited(countur_count, 0);

        //iterate each point
        for (int i = 0; i < countur_count; i++)
        {
            
            if (visited[i])
                continue;
            auto& pointMDSArray = MDSs[i];

            std::vector<std::map<int, MDS>> possibleSelectedMDSs;
            
            for (int mds_idx = 0; mds_idx < pointMDSArray.size(); mds_idx++)
            {
                //test each mds
                auto traverseMDS = pointMDSArray[mds_idx];
                
                std::map<int, MDS> possibleSelectedMDS;
                possibleSelectedMDS[i] = traverseMDS;

                int next_idx = (i + 1);
                for(;next_idx< countur_count;next_idx++)
                {

                    //get next_idx's mds
                    auto& otherMDSArray = MDSs[next_idx];
                    //test intersection
                    auto intersectSelection = traverseMDS.IntersectsRangeReturnSelected(otherMDSArray);
                    if(intersectSelection)
                    {
                        traverseMDS = *intersectSelection;

                        possibleSelectedMDS[next_idx] = traverseMDS;
                    }
                    else 
                    {
                        //no intersection,break
                        break;
                    }
                }


                possibleSelectedMDSs.emplace_back(possibleSelectedMDS);
            }

            //finished all mds test for point i
            //0 find max connected mds
            int max_connect_points = 0;
            int bestMDSIndex = -1;
            for(int k=0;k< possibleSelectedMDSs.size();k++)
            {
                int connect_points = (int)possibleSelectedMDSs[k].size();
                if(connect_points > max_connect_points)
                {
                    max_connect_points = connect_points;
                    bestMDSIndex = k;
                }
            }
            //1 set visited
            if (bestMDSIndex >= 0)
            {
                auto& bestMDSMap = possibleSelectedMDSs[bestMDSIndex];
                for (auto& [idx, mds] : bestMDSMap)
                {
                    visited[idx] = 1;
                    selectMDS[idx] = mds;
                }
            }
            //2 record segment
            if (max_connect_points > 1)
            {
                auto& bestMDSMap = possibleSelectedMDSs[bestMDSIndex];
                MDSSegment seg;
                seg.ID_Start = bestMDSMap.begin()->first;
                seg.ID_End = bestMDSMap.rbegin()->first;

                if (MDSSegments.size() && seg == MDSSegments[MDSSegments.size() - 1])
                    MDSSegments.pop_back();

                MDSSegments.emplace_back(seg);
            }
            
        }

        //merge end segment & first segment
        if (MDSSegments.size() > 1)
        {
            auto lastSeg = MDSSegments[MDSSegments.size() - 1];
            auto startSeg = MDSSegments[0];

            if (lastSeg.ID_End == countur_count - 1 && startSeg.ID_Start == 0)
            {
                //check mds intersection
                auto mds1 = selectMDS[lastSeg.ID_End];
                auto mds2 = selectMDS[startSeg.ID_Start];
                if (mds1.Intersects(mds2))
                {
                    MDSSegment mergedSeg;
                    mergedSeg.ID_Start = lastSeg.ID_Start;
                    mergedSeg.ID_End = startSeg.ID_End;
                    //remove last & first
                    MDSSegments.erase(MDSSegments.begin());
                    MDSSegments.pop_back();
                    //add merged
                    MDSSegments.insert(MDSSegments.begin(), mergedSeg);
                }
            }
        }
    }
    void GreedySearchMDS2()
    {

		//search all MDS, connect mds point to segment if two mds has mds intersection
		int contour_count = (int)points.size();
        selectMDS.resize(contour_count);

        std::vector<char> visited(contour_count, 0);


        for (int i = 0; i < contour_count; i++)
        {
            if(visited[i])
                continue;

            //test each point's mds
            int max_connect_points = 0;
            auto& pointMDSArray = MDSs[i];

			std::vector<std::map<int, MDS>> bestMDSPerPoints;
            int bestMDSIndex = -1;
            for (int j = 0; j < pointMDSArray.size(); j++)
            {
                auto traverseMDS = pointMDSArray[j];

                std::map<int, MDS> bestMDSPerPoint;
                bestMDSPerPoint[i] = traverseMDS;
                //search forward
                int current_idx = i;
                int next_idx = (current_idx + 1) % contour_count;
                
                while (next_idx != i)
                {
                    if (visited[next_idx])
                        break;
                    auto& otherMDSArray = MDSs[next_idx];

                    auto bestMDSForThisPoint = traverseMDS.IntersectsRangeReturnSelected(otherMDSArray);
                    if (bestMDSForThisPoint)
                    {
                        //found intersection,extend
                        traverseMDS = *bestMDSForThisPoint;

						bestMDSPerPoint[next_idx] = traverseMDS;
                    }
                    else
                    {
                        //no intersection,stop extend
                        break;
                    }

                    current_idx = next_idx;
                    next_idx = (current_idx + 1) % contour_count;
                }

                //mark visited
                int end_idx = current_idx;
                int mark_idx = i;

                visited[mark_idx] = 1;
                while (mark_idx != (end_idx % contour_count))
                {
                    visited[mark_idx] = 1;
                    mark_idx = (mark_idx + 1) % contour_count;
                }

                //search backward
                //traverseMDS = pointMDSArray[j];
                //current_idx = i;
                //int back_idx = (current_idx - 1 + contour_count) % contour_count;
                //
                //while (back_idx != i)
                //{
                //    if (visited[back_idx])
                //        break;
                //    auto& otherMDSArray = MDSs[back_idx];
                //
                //    auto bestMDSForThisPoint = traverseMDS.IntersectsRangeReturnSelected(otherMDSArray);
                //    if (bestMDSForThisPoint)
                //    {
                //        //found intersection,extend
                //        traverseMDS = *bestMDSForThisPoint;
				//		bestMDSPerPoint[back_idx] = traverseMDS;
                //    }
                //    else
                //    {
                //        //no intersection,stop extend
                //        break;
                //    }
                //
                //    current_idx = back_idx;
                //    back_idx = (current_idx - 1 + contour_count) % contour_count;
                //}
                ////mark visited
                int start_idx = i;// current_idx;
                //mark_idx = i;
                //while (mark_idx != (start_idx - 1 + contour_count) % contour_count)
                //{
                //    visited[mark_idx] = 1;
                //    mark_idx = (mark_idx - 1 + contour_count) % contour_count;
                //}
                ////record segment

                bestMDSPerPoints.emplace_back(bestMDSPerPoint);
                if (end_idx != start_idx)
                {
					int pointcount = start_idx <= end_idx ? end_idx - start_idx + 1 : contour_count - start_idx + end_idx + 1;

                    if (max_connect_points < pointcount)
                    {
						max_connect_points = pointcount;

                        //update last segment

                        MDSSegment seg;
                        seg.ID_Start = start_idx;
                        seg.ID_End = end_idx;

                        if (MDSSegments.size() && seg == MDSSegments[MDSSegments.size() - 1])
                            MDSSegments.pop_back();


                        bestMDSIndex = j;

						MDSSegments.emplace_back(seg);
                    }
                }
            }

            //merge end segment & first segment
            //if (MDSSegments.size() > 1)
            //{
            //    auto lastSeg = MDSSegments[MDSSegments.size() - 1];
            //    auto startSeg = MDSSegments[0];

            //    if (lastSeg.ID_End == contour_count - 1 && startSeg.ID_Start == 0)
            //    {
            //        MDSSegment mergedSeg;
            //        mergedSeg.ID_Start = lastSeg.ID_Start;
            //        mergedSeg.ID_End = startSeg.ID_End;
            //        //remove last & first
            //        MDSSegments.erase(MDSSegments.begin());
            //        MDSSegments.pop_back();
            //        //add merged
            //        MDSSegments.insert(MDSSegments.begin(), mergedSeg);
            //    }
            //}


            //store best mds for each point
            if(bestMDSIndex>=0)
                for (auto& [idx, mds] : bestMDSPerPoints[bestMDSIndex])
                    selectMDS[idx] = mds;

        }

    }
};

typedef std::vector<MDSContour> MDSContours;

extern void ConnectMDSSegments(MDSContours& contours);

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
	glm::vec3 LenDir = glm::vec3(0, 0, 10);


    //construct ball end & cylinder height
    Tool(float r=1.0f,float height=10.0f)
    {
        radius = r;
        this->height = height;
        LenDir = glm::vec3(0, 0, height);
    }

    void SetPosition(const glm::vec3& pos)
    {
        Position = pos;
	}

    glm::vec3 GetDirection() const
    {
		return glm::normalize(LenDir);
    }

    glm::vec3 GetPerpDirection() const
    {
        return glm::normalize(glm::vec3(-LenDir.z, 0, LenDir.x));
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
    
    std::vector<std::vector<glm::vec2>> getAllConnectionSegments(const MDSContours& contours);

    std::vector<std::vector<glm::vec3>> mergeSegmentsToContour(const MDSContours& contours);

public:
    Pipe() {};


    void initTool(float radius, float height)
    {
		tool = Tool(radius, height);
	}

    void drawAndSaveCanvas(const std::map<int, MDSContours>& positions_all, int idx = -1);

    void CalMDSForEachSlice(std::map<int, MDSContours>& positions_all);

    void GenerateContoursFromMDS(const std::map<int, MDSContours>& positions_all);
    
    int test_idx = 40;
    Tool tool;

    std::map<int, std::vector<std::vector<glm::vec3>>> SavedContours;
};