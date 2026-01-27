#pragma once

#include <vector>
#include <glm/glm.hpp>

class ConvexHull {
public:
    ConvexHull();
    ~ConvexHull();
    
    /**
     * 计算给定2D点集的凸包轮廓
     * 使用Graham扫描算法实现
     * @param inputPoints 输入的2D点集
     * @return 凸包轮廓上的点（按逆时针顺序排列）
     */
    std::vector<glm::vec2> computeConvexHull(const std::vector<glm::vec2>& inputPoints);

private:
    /**
     * 计算三个点的叉积，用于判断转向
     * @param p1 第一个点
     * @param p2 第二个点  
     * @param p3 第三个点
     * @return 叉积值，正数表示左转，负数表示右转，零表示共线
     */
    float crossProduct(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3);
    
    /**
     * 计算两点间距离的平方
     * @param p1 第一个点
     * @param p2 第二个点
     * @return 距离的平方
     */
    float distanceSquared(const glm::vec2& p1, const glm::vec2& p2);
    
    /**
     * 找到最下方的点（y最小），如果有多个则选择最左边的
     * @param points 点集
     * @return 最下方点的索引
     */
    int findBottomMostPoint(const std::vector<glm::vec2>& points);
};