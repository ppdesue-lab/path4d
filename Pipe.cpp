#include "Pipe.h"   

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define CANVAS_ITY_IMPLEMENTATION
#include "src/canvas_ity.hpp"

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

				glm::vec3 normalpt = contour.points[i] + contour.normals[i] * 0.2f;
				float x3 = normalpt.x * scale + offsetX;
				float y3 = normalpt.z * scale + offsetY;

                //canvas.set_color(canvas_ity::stroke_style, 1, 0, 0, 1);

                context.begin_path();
                context.move_to(x1, y1);
                context.line_to(x2, y2);

                context.move_to(x1, y1);
                context.line_to(x3, y3);

                //add normal
                
                //context.addFan(
                //    x1,
                //    y1,
                //    4.0f, // radius
                //    0.0f,
                //    0.5f * 3.14159265f,
                //    true
                //);

                context.close_path();
                auto color = randColor();
                context.set_color(canvas_ity::stroke_style, color.r, color.g, color.b, 1.f);
                context.stroke();
            }
            
            



            contour_idx++;
        }
        layer_idx++;
    }

    // Get image data
    unsigned char* image = new unsigned char[height * width * 4];
    context.get_image_data(image, width, height, width * 4, 0, 0);

    //save as tga
	//stbi_write_tga("sliced_model.tga", width, height, 4, image);

    // Save as PNG
    stbi_write_png("sliced_model.png", width, height, 4, image, width * 4);
}

void Pipe::CalMDSForEachSlice(std::map<int, MDSContours>& positions_all)
{
    auto isAngleValid = [](const Tool& tool,const MDSContours& contours)-> bool
    {
        for (const auto& contour : contours)
        {
            for (const auto& pt : contour.points)
            {
                if (tool.isPointReachable(pt))
                    return false;
            }
        }
		return true;
	};

    auto calMDSForPoint = [&](const MDSContours& contours,Tool& testtool) -> std::vector<MDS>
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

            if (isAngleValid(tool, contours))
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
            for(int k=start_idx;k<end_idx;k++)
            {
				int valid_idx = getValidSubIdx(k);
                validangle[valid_idx]=0;
			}

            //construct MDS
            float start_angle = start_idx * sample_step;
            float end_angle = end_idx * sample_step + sample_step;
            mdsArray.emplace_back(MDS(start_angle, end_angle));
        }

        return mdsArray;
    };

    for (auto& pair : positions_all)
    {
        auto& contours = pair.second;
        for (auto& contour : contours)
        {
            contour.MDSs.clear();
            for (uint32_t i=0;i<contour.points.size();i++)
            {
                auto& contour_pt = contour.points[i];
                
				Tool testtool(tool.radius, tool.height);
				testtool.Position = contour_pt + contour.normals[i] * (tool.radius + 0.01f);

				auto mdsArray = calMDSForPoint(contours, testtool);
                contour.MDSs.emplace_back(mdsArray);
            }
        }
	}
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