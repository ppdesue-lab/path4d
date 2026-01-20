#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <limits>

class KDTree {
public:
    struct Node {
        glm::vec2 point;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;

        Node(const glm::vec2& p) : point(p) {}
    };

    KDTree() : root(nullptr) {}

    void init(const std::vector<glm::vec2>& points) {
        root.reset();
        for (const auto& p : points) {
            insert(p);
        }
    }

    void insert(const glm::vec2& point) {
        root = insertRecursive(std::move(root), point, 0);
    }
#pragma region vec3
    void init(const std::vector<glm::vec3>& points) {
        root.reset();
        for (const auto& p : points) {
            insert({p.x,p.z});
        }
    }

    glm::vec2 nearest(const glm::vec3& target) const {
        if (!root) return glm::vec2(0.0f);
        glm::vec2 best = root->point;
        float bestDist = glm::distance({target.x, target.z}, best);
        nearestRecursive(root.get(), {target.x, target.z}, 0, best, bestDist);
        return best;
    }

    std::vector<glm::vec2> nearestByRadius(const glm::vec3& center, float radius) const {
        std::vector<glm::vec2> result;
        if (!root) return result;
        nearestByRadiusRecursive(root.get(), {center.x, center.z}, radius, 0, result);
        return result;
    }

#pragma endregion

    glm::vec2 nearest(const glm::vec2& target) const {
        if (!root) return glm::vec2(0.0f);
        glm::vec2 best = root->point;
        float bestDist = glm::distance(target, best);
        nearestRecursive(root.get(), target, 0, best, bestDist);
        return best;
    }

    std::vector<glm::vec2> nearestByRadius(const glm::vec2& center, float radius) const {
        std::vector<glm::vec2> result;
        if (!root) return result;
        nearestByRadiusRecursive(root.get(), center, radius, 0, result);
        return result;
    }

private:
    std::unique_ptr<Node> root;

    std::unique_ptr<Node> insertRecursive(std::unique_ptr<Node> node, const glm::vec2& point, int depth) {
        if (!node) {
            return std::make_unique<Node>(point);
        }

        int axis = depth % 2;
        if ((axis == 0 && point.x < node->point.x) || (axis == 1 && point.y < node->point.y)) {
            node->left = insertRecursive(std::move(node->left), point, depth + 1);
        } else {
            node->right = insertRecursive(std::move(node->right), point, depth + 1);
        }

        return node;
    }

    void nearestRecursive(const Node* node, const glm::vec2& target, int depth, glm::vec2& best, float& bestDist) const {
        if (!node) return;

        float dist = glm::distance(target, node->point);
        if (dist < bestDist) {
            bestDist = dist;
            best = node->point;
        }

        int axis = depth % 2;
        float diff = (axis == 0) ? (target.x - node->point.x) : (target.y - node->point.y);

        const Node* first = (diff <= 0) ? node->left.get() : node->right.get();
        const Node* second = (diff <= 0) ? node->right.get() : node->left.get();

        nearestRecursive(first, target, depth + 1, best, bestDist);

        // Check if we need to search the other subtree
        float radius = (axis == 0) ? std::abs(target.x - node->point.x) : std::abs(target.y - node->point.y);
        if (radius < bestDist) {
            nearestRecursive(second, target, depth + 1, best, bestDist);
        }
    }

    void nearestByRadiusRecursive(const Node* node, const glm::vec2& center, float radius, int depth, std::vector<glm::vec2>& result) const {
        if (!node) return;

        float dist = glm::distance(center, node->point);
        if (dist <= radius) {
            result.push_back(node->point);
        }

        int axis = depth % 2;
        float diff = (axis == 0) ? (center.x - node->point.x) : (center.y - node->point.y);

        const Node* first = (diff <= 0) ? node->left.get() : node->right.get();
        const Node* second = (diff <= 0) ? node->right.get() : node->left.get();

        nearestByRadiusRecursive(first, center, radius, depth + 1, result);

        // Check if the other subtree could contain points within radius
        float distToPlane = std::abs(diff);
        if (distToPlane <= radius) {
            nearestByRadiusRecursive(second, center, radius, depth + 1, result);
        }
    }
};