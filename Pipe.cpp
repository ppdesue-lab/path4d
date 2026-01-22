#include "Pipe.h"   

//#define SHOW_ALL_MDS

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define CANVAS_ITY_IMPLEMENTATION
#include <src/canvas_ity.hpp>

#include "KDTree.hpp"

#include <set>
#include <tuple>

#include <fstream>
#include <time.h>
#include <random>
glm::vec3 randColor()
{
    float r = rand()/(float)RAND_MAX;
    float g = rand()/(float)RAND_MAX;
    float b = rand()/(float)RAND_MAX;
    return glm::vec3(r, g, b);
};

void Pipe::drawAndSaveCanvas(const std::map<int, MDSContours>& positions_all, int idx)
{
    
    // Canvas size
    int width = 1000;
    int height = 1000;
    canvas_ity::canvas context(width, height);

    // Calculate bounding box
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (const auto& pair : positions_all)
    {
        const auto& contours = pair.second;
        for (const auto& contour : contours)
        {
            for(const auto& pos : contour.points)
            {
                minX = std::min(minX, pos.x);
                maxX = std::max(maxX, pos.x);
                minY = std::min(minY, pos.z);
                maxY = std::max(maxY, pos.z);
			}
        }
        
    }

    float dataWidth = maxX - minX;
    float dataHeight = maxY - minY;
    float scale = std::min(width / dataWidth, height / dataHeight) * 0.95f; // 80% to leave margin
    float offsetX = (width - dataWidth * scale) / 2.0f - minX * scale;
    float offsetY = (height - dataHeight * scale) / 2.0f - minY * scale;

    // Draw lines
	int layer_idx = 0;
    for (const auto& pair : positions_all)
    {
        if(layer_idx !=idx && idx!=-1)
        {
            layer_idx++;
            continue;
		}
        const auto& contours = pair.second;
		int contour_idx = 0;
        for (const auto& contour : contours)
        {

            context.set_line_width(1.f); 
            int ptcount = contour.points.size();
            for (size_t i = 0; i < ptcount; i++)
            {
                float x1 = contour.points[i].x * scale + offsetX;
                float y1 = contour.points[i].z * scale + offsetY;
                float x2 = contour.points[(i + 1)% ptcount].x * scale + offsetX;
                float y2 = contour.points[(i + 1) % ptcount].z * scale + offsetY;

				glm::vec3 normalpt = contour.points[i] + contour.normals[i] * tool.radius;
				float x3 = normalpt.x * scale + offsetX;
				float y3 = normalpt.z * scale + offsetY;

                //canvas.set_color(canvas_ity::stroke_style, 1, 0, 0, 1);

                context.begin_path();
                context.move_to(x1, y1);
                context.line_to(x2, y2);

#if 0
                //add normal
                context.move_to(x1, y1);
                context.line_to(x3, y3);
#endif
                //add mds fan
                //static bool firstdraw = true;
                //if (firstdraw)
                {
                    //firstdraw = !firstdraw;
#ifdef SHOW_ALL_MDS
                    auto mdsArray = contour.MDSs[i];
                    for (auto& mds : mdsArray)
#else
                    auto mds = contour.selectMDS[i];
#endif
                    if(mds.Range()>0)
                    {
                        float startRad = glm::radians(mds.start);
                        float endRad = glm::radians(mds.end);
                        context.addFan(
                            x3,
                            y3,
                            8.0f, // radius
                            startRad,
                            endRad,
                            true
                        );

                    }
                }

                context.close_path();
                auto color = randColor();
                context.set_color(canvas_ity::stroke_style, color.r, color.g, color.b, 1.f);
                context.stroke();

            }

            contour_idx++;
        }
        layer_idx++;
    }

    layer_idx = 0;
    for (const auto& pair : positions_all)
    {
        if (layer_idx != idx && idx != -1)
        {
            layer_idx++;
            continue;
        }
        const auto& contours = pair.second;
        int contour_idx = 0;
        for (const auto& contour : contours)
        {
			//draw mds segments
			for (const auto& seg : contour.MDSSegments)
			{
                
                if (seg.isConnected) {
                    // Draw a line from this contour's ID_Start to other contour's otherPointID
                    const glm::vec3& posA = contour.points[seg.ID_Start];
                    
                    glm::vec3 posB;
                    if(seg.otherContourID>=0)
                        posB = contours[seg.otherContourID].points[seg.otherPointID];
                    else
                        posB = contour.points[seg.ID_End];
                    float x1 = posA.x * scale + offsetX;
                    float y1 = posA.z * scale + offsetY;
                    float x2 = posB.x * scale + offsetX;
                    float y2 = posB.z * scale + offsetY;
                    context.begin_path();
                    context.move_to(x1, y1);
                    context.line_to(x2, y2);
                    context.close_path();
                    context.set_line_width(3.f);
                    context.set_color(canvas_ity::stroke_style, 0, 1, 0, 1.f); // green for connected


                    context.stroke();
                } else{
                    int start_idx = seg.ID_Start;
                    int end_idx = seg.ID_End;



                    //float x1 = contour.points[start_idx].x * scale + offsetX;
                    //float y1 = contour.points[start_idx].z * scale + offsetY;
                    //float x2 = contour.points[end_idx].x * scale + offsetX;
                    //float y2 = contour.points[end_idx].z * scale + offsetY;
                    //auto mds = contour.selectMDS[seg.ID_Start];
                    //float startRad = glm::radians(mds.start);
                    //float endRad = glm::radians(mds.end);
                    //context.begin_path();
                    //context.addFan(
                    //    x1,
                    //    y1,
                    //    8.0f, // radius
                    //    startRad,
                    //    endRad,
                    //    true
                    //);
                    //auto mds2 = contour.selectMDS[seg.ID_End];
                    //float startRad2 = glm::radians(mds2.start);
                    //float endRad2 = glm::radians(mds2.end);
                    //context.addFan(
                    //    x2,
                    //    y2,
                    //    8.0f, // radius
                    //    startRad2,
                    //    endRad2,
                    //    true
                    //);
                    //context.close_path();
                    //context.set_color(canvas_ity::stroke_style,0, 1, 1, 1);
                    //context.stroke();

#if 1
                    if(start_idx < end_idx)
                    {
                        //draw line from start_idx to end_idx
                        for(int i=start_idx;i< end_idx;i++)
                        {
                            float x1 = contour.points[i].x * scale + offsetX;
                            float y1 = contour.points[i].z * scale + offsetY;
                            float x2 = contour.points[(i + 1) % contour.points.size()].x * scale + offsetX;
                            float y2 = contour.points[(i + 1) % contour.points.size()].z * scale + offsetY;

                            context.begin_path();
                            context.move_to(x1, y1);
                            context.line_to(x2, y2);
                            context.close_path();
                            context.set_line_width(3.f);
                            context.set_color(canvas_ity::stroke_style, 1, 0, 0, 1.f);
                            context.stroke();
                        }
                    }
                    else
                    {
                        //split it into two parts
                        for (int i = start_idx; i < contour.points.size(); i++)
                        {
                            float x1 = contour.points[i].x * scale + offsetX;
                            float y1 = contour.points[i].z * scale + offsetY;
                            float x2 = contour.points[(i + 1) % contour.points.size()].x * scale + offsetX;
                            float y2 = contour.points[(i + 1) % contour.points.size()].z * scale + offsetY;

                            context.begin_path();
                            context.move_to(x1, y1);
                            context.line_to(x2, y2);
                            context.close_path();
                            context.set_line_width(3.f);
                            context.set_color(canvas_ity::stroke_style, 1, 0, 0, 1.f);
                            context.stroke();
                        }
                        for (int i = 0; i < end_idx; i++)
                        {
                            float x1 = contour.points[i].x * scale + offsetX;
                            float y1 = contour.points[i].z * scale + offsetY;
                            float x2 = contour.points[(i + 1) % contour.points.size()].x * scale + offsetX;
                            float y2 = contour.points[(i + 1) % contour.points.size()].z * scale + offsetY;

                            context.begin_path();
                            context.move_to(x1, y1);
                            context.line_to(x2, y2);
                            context.close_path();
                            context.set_line_width(3.f);
                            context.set_color(canvas_ity::stroke_style, 1, 0, 0, 1.f);
                            context.stroke();
                        }

                    }

#endif
                }
            }
        }
    }

    //paint tool on canvas
    {
        Tool testtool(tool.radius, tool.height);
        int layer_idx = 0;
        for (const auto& pair : positions_all)
        {
            if (layer_idx != idx && idx != -1)
            {
                layer_idx++;
                continue;
            }
            const auto& contours = pair.second;
            for (const auto& contour : contours)
            {
                //for (const auto& pos : contour.points)
                for(int i=0;i<contour.points.size();i++)
                {
                    auto pos = contour.points[i];
                    testtool.Position = pos +contour.normals[i] * (tool.radius + 0.01f);
                    testtool.SetRotation(contour.normals[i]);
                    //float angle = atan2f(contour.normals[i].z, contour.normals[i].x);
                    //if (contour.MDSs[i].size())
                    //{
                    //    angle = 0.5f * (contour.MDSs[0][0].start + contour.MDSs[0][0].end);

                    //}
                    //testtool.SetRotationAngle(angle);
                    goto A;
                }
            }
        }
        A:
        

        float x1 = testtool.Position.x * scale + offsetX;
        float y1 = testtool.Position.z * scale + offsetY;

		float x2 = testtool.GetTopPosition().x * scale + offsetX;
		float y2 = testtool.GetTopPosition().z * scale + offsetY;

		float scaled_radius = testtool.radius * scale;

        auto prepDistance = testtool.GetPerpDirection() * scaled_radius;

        context.begin_path();
        //context.move_to(x1, y1);
        //context.line_to(x2, y2);

        context.move_to(x2 + prepDistance.x, y2 + prepDistance.z);
        context.line_to(x1+prepDistance.x, y1 + prepDistance.z);


        context.line_to(x1 - prepDistance.x, y1 - prepDistance.z);
        context.line_to(x2 - prepDistance.x, y2 - prepDistance.z);
        context.addFan(
                    x1,
                    y1,
                    scaled_radius, // radius
                    0.0f,
                    2.0f * 3.14159265f,
            true
                );



        context.close_path();
        auto color = randColor();
        context.set_color(canvas_ity::fill_style, color.r, color.g, color.b, 0.5f);
        context.fill();
    }

    // Get image data
    unsigned char* image = new unsigned char[height * width * 4];
    context.get_image_data(image, width, height, width * 4, 0, 0);

    //save as tga
	//stbi_write_tga("sliced_model.tga", width, height, 4, image);

	stbi_flip_vertically_on_write(1);
    // Save as PNG
    stbi_write_png("sliced_model.png", width, height, 4, image, width * 4);
}

void Pipe::CalMDSForEachSlice(std::map<int, MDSContours>& positions_all)
{
    auto ToKDtree2D = [](const glm::vec3& pt) -> glm::vec2
    {
        return glm::vec2(pt.x, pt.z);
	};

    auto isAngleValid = [&ToKDtree2D](const Tool& tool, const MDSContours& contours, const KDTree& kdtree) -> bool
    {
        // 使用KDTree找到最近的点
        glm::vec2 nearest_pt = kdtree.nearest(ToKDtree2D(tool.Position));
        float dist_sq = glm::length(ToKDtree2D(tool.Position) - nearest_pt);
        dist_sq *= dist_sq;
        float threshold = tool.radius;
        if (dist_sq < threshold * threshold) {
            return false;
        }
        //todo:check cylinder part
        //check point inside cylinder part or not
        auto search_cylinder_center = ToKDtree2D(tool.Position + tool.GetDirection() * (tool.height / 2.0f));
        auto search_cylinder_radius = sqrtf(tool.radius * tool.radius + (tool.height / 2.0f) * (tool.height / 2.0f));
        auto potential_pts = kdtree.nearestByRadius(search_cylinder_center, search_cylinder_radius);
        for (const auto& pt : potential_pts)
        {
            auto to_pt = pt - ToKDtree2D(tool.Position);
            float to_along = glm::dot(to_pt, ToKDtree2D(tool.GetDirection()));
            if (to_along < 0 || to_along > tool.height)
                continue;
            auto proj_pt = ToKDtree2D(tool.Position + tool.GetDirection() * to_along);
            float dist_to_axis = glm::length(pt - proj_pt);
            if (dist_to_axis < tool.radius)
                return false;
        }
        return true;
    };

    auto calMDSForPoint = [&](const MDSContours& contours,Tool& testtool,const KDTree& kdtree) -> std::vector<MDS>
    {
        
        //for each angle 0~360degree,every 5degree
		const int sample_num = 72;
		const int sample_step = 360 / sample_num;
        char validangle[sample_num];
		memset(validangle, 0, sizeof(validangle));
        for (int angle = 0; angle < sample_num; angle ++)
        {
            float angleRad = glm::radians((float)angle*sample_step);
            testtool.SetRotationAngle(angleRad);

            if (isAngleValid(testtool, contours, kdtree))
            {
				validangle[angle] = 1;
            }
        }
		//find continuous valid angle range
        auto getValidSubIdx = [&](int idx)->int
        {
            return (idx + sample_num) % sample_num;
        };
        std::vector<MDS> mdsArray;
        int start_idx = -1;
        for (int i = 0; i < sample_num; i++)
        {
            if(validangle[i]==0)continue;

            //search back to find start angle(maybe negative)
            int backid = i - 1 ;
            while(validangle[getValidSubIdx(backid)]==1 && getValidSubIdx(backid) !=i )
            {
                backid = backid - 1 ;
            }

            if(getValidSubIdx(backid) == i) //all valid
            {
                mdsArray.emplace_back(MDS(0.0f, 360.0f));
                break;
            }
            else
            {
                start_idx = (backid + 1);
            }

            //search forward to find end angle
            int frontid = i + 1 ;
            while(validangle[getValidSubIdx(frontid)]==1 && getValidSubIdx(frontid) != start_idx )
            {
                frontid = frontid + 1 ;
            }
            int end_idx = getValidSubIdx(frontid - 1);
            
            //clear tag
            for(int k=start_idx;k<=end_idx;k++)
            {
				int valid_idx = getValidSubIdx(k);
                validangle[valid_idx]=0;
			}

            //construct MDS
            float start_angle = start_idx * sample_step;
            float end_angle = end_idx * sample_step ;
            //narrow mds
            //MDS tmp(start_angle, end_angle);
            //if (tmp.Range() > sample_step)
            //{
            //    start_angle = fmodf(start_angle + sample_step / 2, 360.0);
            //    end_angle = fmodf(end_angle - sample_step / 2, 360.0);
            //}
            mdsArray.emplace_back(MDS(start_angle, end_angle));
        }

        //remain 1 with larger range
        if(mdsArray.size()>1)
        {
            float max_range = 0.0f;
            int max_idx = -1;
            for(int i=0;i< mdsArray.size();i++)
            {
                float range = mdsArray[i].Range();
                if(range > max_range)
                {
                    max_range = range;
                    max_idx = i;
                }
            }
            std::vector<MDS> filteredMDS;
            filteredMDS.emplace_back( mdsArray[max_idx]);
            mdsArray = filteredMDS;
        }

        return mdsArray;
    };

    for (auto& pair : positions_all)
    {
        //for each slice
#ifndef NDEBUG
        if(pair.first != test_idx)
            continue;//skip bottom layer
#endif
        auto& contours = pair.second;
        // 构建KDTree
        KDTree kdtree;
        std::vector<glm::vec3> points;
        for (auto& contour : contours)
            points.insert(points.end(), contour.points.begin(), contour.points.end());
        kdtree.init(points);

        //test kdtree
        //auto testpt = points[0] + glm::vec3(0, 0, 0.01f);
		//auto testresult = kdtree.nearest(testpt);
        for (auto& contour : contours)
        {
            contour.MDSs.clear();
            for (uint32_t i=0;i<contour.points.size();i++)
            {
                auto& contour_pt = contour.points[i];
                
				Tool testtool(tool.radius, tool.height);
				testtool.Position = contour_pt + contour.normals[i] * (tool.radius + 0.05f);

				auto mdsArray = calMDSForPoint(contours, testtool, kdtree);
                contour.MDSs.emplace_back(mdsArray);
            }

            contour.GreedySearchMDS();
            int a = 0;
        }
        
        ConnectMDSSegments(contours);
	}
    // After processing all contours in the slice, connect segments
    // for (auto& pair : positions_all) {
    //     auto& contours = pair.second;
    //     ConnectMDSSegments(contours);
    // }
}



void ConnectMDSSegments(MDSContours& contours)
{
    // Function to check if two line segments intersect
    auto doIntersect = [](const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2) -> bool {
        auto orientation = [](const glm::vec2& p, const glm::vec2& q, const glm::vec2& r) -> int {
            float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
            if (fabs(val) < 1e-6) return 0; // collinear
            return (val > 0) ? 1 : 2; // clock or counterclock wise
        };
        auto onSegment = [](const glm::vec2& p, const glm::vec2& q, const glm::vec2& r) -> bool {
            if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
                return true;
            return false;
        };
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
        if (o1 != o2 && o3 != o4)
            return true;
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        return false;
    };

    // Function to check if line segment intersects with any polygon in contours
    auto lineIntersectsAnyContour = [&](const glm::vec2& A, const glm::vec2& B, const MDSContours& contours) -> bool {
        for (const auto& contour : contours) {
            int n = contour.points.size();
            for (int i = 0; i < n; ++i) {
                glm::vec2 p1(contour.points[i].x, contour.points[i].z);
                glm::vec2 p2(contour.points[(i + 1) % n].x, contour.points[(i + 1) % n].z);
                if (doIntersect(A, B, p1, p2)) {
                    return true;
                }
            }
        }
        return false;
    };
    struct ContourSegmentPoint
    {
        int contourID;
        int segmentID;
        int pointID;
        bool isStart;
        bool extruded = false;
    };
    
    std::vector<ContourSegmentPoint> allSegmentPoints;

    //insert all points
    for(int c=0;c< contours.size();c++)
    {
        //for each contour
        auto& contour = contours[c];
        for(int i=0;i< contour.MDSSegments.size();i++)
        {
            //push to allsegmentspoints for later processing
            auto& seg = contour.MDSSegments[i];
            allSegmentPoints.push_back({ c, i, seg.ID_Start,true });
            allSegmentPoints.push_back({ c, i, seg.ID_End,false });
        }
    }
    //container
    std::vector< MDSSegment> segment_in_contours;
    //connect single contour first
    int segpoint_idx = 0;
    for(int c=0;c< contours.size();c++)
    {
        //for each contour
        auto& contour = contours[c];
        int segpoint_idx_incontour = 0;
        int fixed_segment_count = contour.MDSSegments.size();
        int total_segpoint_count = contour.MDSSegments.size() * 2;
        for(int i=0;i< fixed_segment_count;i++)
        {
            //connect cur segment'end to next segment's start
            auto& seg = contour.MDSSegments[i];
            int next_idx = (i + 1) % fixed_segment_count;
            auto& next_seg = contour.MDSSegments[next_idx];
            //check if two mds has intersection
            const MDS& mdsA = contour.selectMDS[seg.ID_End];
            const MDS& mdsB = contour.selectMDS[next_seg.ID_Start];
            

            if (!seg.isConnected && !next_seg.isConnected
                && mdsA.Intersects(mdsB))
            {
                //check intersection with contour
                auto tmpA = contour.points[seg.ID_End] + contour.normals[seg.ID_End] * 0.05f;
                auto tmpB = contour.points[next_seg.ID_Start] + contour.normals[next_seg.ID_Start] * 0.05f;

                glm::vec2 primeA(tmpA.x, tmpA.z);
                glm::vec2 primeB(tmpB.x, tmpB.z);

                if (!lineIntersectsAnyContour(primeA, primeB, contours))
                {

                    //mark these two points as extruded
                    //add segment
                    MDSSegment newSeg;
                    newSeg.ID_Start = seg.ID_End;
                    newSeg.ID_End = next_seg.ID_Start;
                    newSeg.isConnected = true; // same contour
                    contour.MDSSegments.push_back(newSeg);

                    allSegmentPoints[segpoint_idx + (segpoint_idx_incontour + 1) % total_segpoint_count].extruded = true;
                    allSegmentPoints[segpoint_idx + (segpoint_idx_incontour + 2) % total_segpoint_count].extruded = true;
                }
                
            }

            segpoint_idx_incontour += 2;
            

        }
        segpoint_idx += segpoint_idx_incontour;
    }

#if 0
    for (int c = 0; c < contours.size(); ++c) {
        auto& contour = contours[c];
        

        #if 1
        //for each segment points
        std::vector<ContourSegmentPoint> segmentPoints;
        for (int i=0;i<contour.MDSSegments.size();i++) {
            const auto& seg = contour.MDSSegments[i];
            segmentPoints.push_back({c, i, seg.ID_Start,true});
            segmentPoints.push_back({c, i, seg.ID_End,false});
        }

        
        //for each segmentpoint, find nearest point in same contour(different segment)
        //for (auto& pointA : segmentPoints) {
        for(int i=0;i<segmentPoints.size();i++){
            auto& pointA = segmentPoints[i];
            const auto& segA = contour.MDSSegments[pointA.segmentID];
            if (segA.isConnected || pointA.extruded)
                continue;
            const glm::vec3& posA = contour.points[pointA.pointID];
            // Get MDS for A, assume first MDS
            if (contour.selectMDS[pointA.pointID].Range()==0.0f) continue;
            const MDS& mdsA = contour.selectMDS[pointA.pointID];
            glm::vec2 midNormalA = mdsA.GetMidNormalizedDir();

            float minDist = std::numeric_limits<float>::max();
            int bestEndIdx = -1;
            for (int k = 0; k < segmentPoints.size();k++){
                auto pointB = segmentPoints[k];
                if ((contours.size()>1 && pointB.segmentID == pointA.segmentID) ||
                    i==k || pointB.extruded)
                    continue; // same segment

                //find nearest seg end point
                const glm::vec3& posB = contour.points[pointB.pointID];
                float dist = glm::distance(posA, posB);
                if (dist < minDist) {
                    
                    minDist = dist;
                    bestEndIdx = k;
                }
            }
			//now check the nearest point's mds midnormal

            if (bestEndIdx != -1) {
                auto& pointB = segmentPoints[bestEndIdx];

                if(!(abs(pointA.segmentID - pointB.segmentID)==1 ||
                    (pointA.segmentID==0 && pointB.segmentID == contour.MDSSegments.size()-1) ||
                    (pointB.segmentID==0 && pointA.segmentID == contour.MDSSegments.size()-1)
                    ))
                {
                    //not adjacent segment,no need connect
                    continue;
				}

                if (!(pointA.isStart ^ pointB.isStart))
                {
                    //same direction,no need connect
                    continue;
                }

                if (contour.selectMDS[pointB.pointID].Range()==0.0f) continue;
                const MDS& mdsB = contour.selectMDS[pointB.pointID];
                glm::vec2 midNormalB = mdsB.GetMidNormalizedDir();
                //float dot = glm::dot(midNormalA, midNormalB);
                //if (dot > 0) 
                
                if (mdsA.Intersects(mdsB))
                {

                    //mark these two points as extruded
                    pointA.extruded = true;
                    pointB.extruded = true;



                    // If found a valid point to connect
                    // Add segment
                    MDSSegment newSeg;
                    newSeg.ID_Start = pointA.pointID;
                    newSeg.ID_End = pointB.pointID;
                    newSeg.isConnected = true; // same contour
                    contour.MDSSegments.push_back(newSeg);

                }
                else
                    int a = 0;

            }
        }
        #endif

        
        allSegmentPoints.insert(allSegmentPoints.end(), segmentPoints.begin(), segmentPoints.end());
    }
#endif
    //return;
    //-----------------------------------------------------------------
    //outter contour connection can be added here
    allSegmentPoints.erase(
        std::remove_if(
            allSegmentPoints.begin(),
            allSegmentPoints.end(),
            [](const ContourSegmentPoint& p) { return p.extruded; }
        ),
        allSegmentPoints.end()
	);

    for(int i=0;i< allSegmentPoints.size();i++)
    {
        auto& pointA = allSegmentPoints[i];
        if (pointA.extruded)
            continue;

        const auto& contourA = contours[pointA.contourID];
        const glm::vec3 posA = contourA.points[pointA.pointID];
        const glm::vec3 normalA = contourA.normals[pointA.pointID];
        // Get MDS for A, assume first MDS
        if (contourA.selectMDS[pointA.pointID].Range() == 0.0f) 
            continue;
        const MDS& mdsA = contourA.selectMDS[pointA.pointID];
        glm::vec2 midNormalA = mdsA.GetMidNormalizedDir();

        float minDist = std::numeric_limits<float>::max();
        int bestEndIdx = -1;
        for (int k = 0; k < allSegmentPoints.size(); k++) {
            auto& pointB = allSegmentPoints[k];
            if (pointB.contourID == pointA.contourID)
                continue; // same contour
            if (pointB.extruded)
                continue;

            if (i == k)
                continue;

            const MDS& mdsB = contours[pointB.contourID].selectMDS[pointB.pointID];
            glm::vec2 midNormalB = mdsB.GetMidNormalizedDir();

            float dot = glm::dot(midNormalA, midNormalB);

            //find nearest seg end point
            const glm::vec3& posB = contours[pointB.contourID].points[pointB.pointID];
            const glm::vec3& normalB = contours[pointB.contourID].normals[pointB.pointID];
            float dist = glm::distance(posA, posB);

            auto tmpA = posA + normalA * 0.05f;
            auto tmpB = posB + normalB * 0.05f;

            glm::vec2 primeA(tmpA.x, tmpA.z);
            glm::vec2 primeB(tmpB.x, tmpB.z);

            if (dot > 0 && dist < minDist && !lineIntersectsAnyContour(primeA, primeB, contours)) {

                //if (lineIntersectsAnyContour(primeA, primeB, contours))
                //{
                //    //intersect with other contour,no connect
                //    continue;
                //}

                minDist = dist;
                bestEndIdx = k;
            }
        }

        //connect A and B
        if (bestEndIdx != -1) {
            auto& pointB = allSegmentPoints[bestEndIdx];


            //if (contours[pointB.contourID].selectMDS[pointB.pointID].Range() == 0.0f) 
            //    continue;
            //const MDS& mdsB = contours[pointB.contourID].selectMDS[pointB.pointID];
            //glm::vec2 midNormalB = mdsB.GetMidNormalizedDir();

            //auto primeA = posA + glm::vec3(midNormalA.x, 0, midNormalA.y) * 0.01f;
            //auto primeB = contours[pointB.contourID].points[pointB.pointID] + glm::vec3(-midNormalB.x, 0, -midNormalB.y) * 0.01f;
            //if(lineIntersectsAnyContour(primeA,primeB,contours))
            //{
            //    //intersect with other contour,no connect
            //    continue;
            //}

            //float dot = glm::dot(midNormalA, midNormalB);
            //if (dot > 0) 
            {

                //mark these two points as extruded
                pointA.extruded = true;
                pointB.extruded = true;

                // If found a valid point to connect
                // Add segment
                MDSSegment newSeg;
                newSeg.ID_Start = pointA.pointID;
                newSeg.ID_End = pointB.pointID;
                newSeg.isConnected = true; // cross contour
                newSeg.otherContourID = pointB.contourID;
                newSeg.otherPointID = pointB.pointID;
                contours[pointA.contourID].MDSSegments.push_back(newSeg);

            }
        }
        else
        {
            int a = 0;
        }
    }


#pragma region Debug Test

//#ifndef NDEBUG
    allSegmentPoints.erase(
        std::remove_if(
            allSegmentPoints.begin(),
            allSegmentPoints.end(),
            [](const ContourSegmentPoint& p) { return p.extruded; }
        ),
        allSegmentPoints.end()
    );

    static int i = 0;
    if (allSegmentPoints.size())
    {
        printf("remaining segment [%d] points:%d\n", i, (int)allSegmentPoints.size());
        int a = 0;
    }
    i++;
//#endif

#pragma endregion

    
	

}

MDSContours computeMDSContours(const std::vector<std::vector<glm::vec3>>& contour_positions)
{
    MDSContours contours;
    for (const auto& contour : contour_positions)
    {
        MDSContour mds_contour;
        mds_contour.fromPoints(contour);
        contours.push_back(mds_contour);
    }

    return contours;
}