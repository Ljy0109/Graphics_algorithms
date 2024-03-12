//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };  // 定义了两种分裂方法，即NAIVE和SAH

    // BVHAccel Public Methods
    // 接受一个对象指针的向量，以及最大节点中允许的物体数量和分裂方法
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const; // 返回整个BVH的世界边界框
    ~BVHAccel();

    // Intersection定义在Intersection.hpp中
    Intersection Intersect(const Ray &ray) const;   // 对输入的光线执行相交测试，并返回相交处的Intersection结构
    // 归地从给定的BVHBuildNode开始执行相交测试，并返回相交处的Intersection结构
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;  // 对输入的光线执行相交测试，但只返回是否存在相交
    BVHBuildNode* root; // 指向BVH的根节点

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);  // 递归地构建BVH树

    // BVHAccel Private Data
    const int maxPrimsInNode;   // 每个节点中允许的最大物体数量
    const SplitMethod splitMethod;  // 指定BVH的分裂方法
    std::vector<Object*> primitives;    // 存储了场景中的所有物体的指针

    // 从BVH中获取一个采样点，以及该点的概率密度函数值
    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);
    // 从BVH中采样一个点，并返回该点的概率密度函数值
    void Sample(Intersection &pos, float &pdf);
};

struct BVHBuildNode {
    // BVH树的节点，包含了节点的边界框、左右子节点、指向物体的指针以及面积
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;
    float area;

public:
    // splitAxis：表示当前节点在哪个轴上进行分裂。如果是0，则表示在x轴上进行分裂；如果是1，则表示在y轴上进行分裂；如果是2，则表示在z轴上进行分裂。默认初始化为0。
    // firstPrimOffset：表示当前节点所包含的第一个物体在整个物体列表中的偏移量（索引）。默认初始化为0。
    // nPrimitives：表示当前节点所包含的物体数量。默认初始化为0。
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};

#endif //RAYTRACING_BVH_H
