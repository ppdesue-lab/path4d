#include "ConvexHull.h"
#include <algorithm>
#include <stack>
#include <cmath>

ConvexHull::ConvexHull() {
}

ConvexHull::~ConvexHull() {
}

float ConvexHull::crossProduct(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3) {
    // 计算向量 (p1->p2) 和 (p1->p3) 的叉积
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

float ConvexHull::distanceSquared(const glm::vec2& p1, const glm::vec2& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}

int ConvexHull::findBottomMostPoint(const std::vector<glm::vec2>& points) {
    int minIndex = 0;
    float minY = points[0].y;
    float minX = points[0].x;
    
    for (int i = 1; i < points.size(); ++i) {
        if (points[i].y < minY || (points[i].y == minY && points[i].x < minX)) {
            minIndex = i;
            minY = points[i].y;
            minX = points[i].x;
        }
    }
    
    return minIndex;
}

std::vector<glm::vec2> ConvexHull::computeConvexHull(const std::vector<glm::vec2>& inputPoints) {
    std::vector<glm::vec2> result;
    
    // 处理边界情况
    if (inputPoints.size() < 3) {
        return inputPoints; // 少于3个点无法构成凸包
    }
    
    // 创建点的副本用于排序
    std::vector<glm::vec2> points = inputPoints;
    
    // 1. 找到最下方的点作为起始点
    int startIndex = findBottomMostPoint(points);
    std::swap(points[0], points[startIndex]);
    glm::vec2 startPoint = points[0];
    
    // 2. 按照相对于起始点的极角进行排序
    std::sort(points.begin() + 1, points.end(), [this, &startPoint](const glm::vec2& a, const glm::vec2& b) {
        float cross = crossProduct(startPoint, a, b);
        if (cross == 0) {
            // 如果共线，按距离排序
            return distanceSquared(startPoint, a) < distanceSquared(startPoint, b);
        }
        return cross > 0; // 逆时针排序
    });
    
    // 3. 处理共线的最远点（保留距离最大的）
    int m = 1;
    for (int i = 1; i < points.size(); ++i) {
        // 跳过与前一个点共线的点
        while (i < points.size() - 1 && 
               crossProduct(startPoint, points[i], points[i + 1]) == 0) {
            i++;
        }
        points[m++] = points[i];
    }
    
    if (m < 3) {
        return result; // 所有点共线，无法构成凸包
    }
    
    // 4. 使用栈进行Graham扫描
    std::stack<glm::vec2> hull;
    hull.push(points[0]);
    hull.push(points[1]);
    hull.push(points[2]);
    
    for (int i = 3; i < m; ++i) {
        // 移除右转的点
        while (hull.size() > 1) {
            glm::vec2 top = hull.top();
            hull.pop();
            glm::vec2 nextToTop = hull.top();
            
            if (crossProduct(nextToTop, top, points[i]) > 0) {
                hull.push(top);
                break;
            }
            // 否则继续移除top点
        }
        hull.push(points[i]);
    }
    
    // 5. 将栈中的点转换为结果向量
    while (!hull.empty()) {
        result.insert(result.begin(), hull.top());
        hull.pop();
    }
    
    return result;
}