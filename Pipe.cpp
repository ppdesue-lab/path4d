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
    float scale = std::min(width / dataWidth, height / dataHeight) * 0.8f; // 80% to leave margin
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
            for (size_t i = 0; i < contour.points.size(); i += 2)
            {
                float x1 = contour.points[i].x * scale + offsetX;
                float y1 = contour.points[i].z * scale + offsetY;
                float x2 = contour.points[i + 1].x * scale + offsetX;
                float y2 = contour.points[i + 1].z * scale + offsetY;


                auto color = randColor();
                //canvas.set_color(canvas_ity::stroke_style, 1, 0, 0, 1);
                context.begin_path();
                context.move_to(x1, y1);
                context.line_to(x2, y2);

                context.addFan(
                    x1,
                    y1,
                    4.0f, // radius
                    0.0f,
                    0.5f * 3.14159265f,
                    true
                );
                context.close_path();
                context.set_color(canvas_ity::stroke_style, color.r,color.g, color.b, 1.f);
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