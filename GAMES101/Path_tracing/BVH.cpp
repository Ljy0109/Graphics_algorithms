#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))  // 在初始化列表中对成员变量进行初始化。maxPrimsInNode被限制在255以下，以避免节点包含过多物体，造成树的深度过深。splitMethod是通过传入的参数直接初始化。
                                // primitives使用std::move将传入的物体指针数组 p 移动到成员变量 primitives 中。
{
    time_t start, stop;
    time(&start);   // 记录开始时间
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);  // 递归地构建BVH树，并将返回的根节点指针赋值给 root 成员变量

    time(&stop);
    double diff = difftime(stop, start);    // 计算构建BVH树所花费的时间
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{   // BVH树的递归构建函数
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)    
    // 遍历物体数组，计算所有物体的边界框的并集，以得到当前节点的边界框。
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector<Object *>{objects[0]});
        node->right = recursiveBuild(std::vector<Object *>{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();   // 计算物体中心的边界框，并找出其中最长的轴。
        switch (dim) {
        case 0:// 根据最长轴将物体数组排序
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        // 将物体数组分成左右两半
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);    // 递归构建BVH
        node->right = recursiveBuild(rightshapes);

        // 将当前节点的边界框设置为左右子节点边界框的并集。
        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{   // 获取光线与BVH树中物体的交点信息
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection intersect;
    Vector3f invdir(1./ray.direction.x,1./ray.direction.y,1./ray.direction.z);  // 乘法比除法快，invdir=ray.direction_inv
    std::array<int, 3> dirIsNeg;    // 用于存储光线方向是否为负的信息
    dirIsNeg[0] = ray.direction.x>0;
    dirIsNeg[1] = ray.direction.y>0;
    dirIsNeg[2] = ray.direction.z>0;
    if(!node || !node->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg))
    {   // 检查当前节点是否存在以及光线是否与当前节点的边界框相交
        // 如果当前节点不存在或者光线不与当前节点的边界框相交，则返回空的交点结构体 
        return intersect;
    }

    if(!node->right && !node->left) // 检查当前节点是否为叶子节点
        // 这里的node->object有两种意思
        // 第一种，node是scene.bvh的，此时node->object是指MeshTrangles.bvh
        // 第二种，node是MeshTrangles.bvh的，此时node->object是指单个trangle，
        // 此时的node->object->getIntersection就不是BVHAccel::getIntersection，
        // 而是Triangle::getIntersection，实际计算交点信息的时候(by MT算法)
        return node->object->getIntersection(ray);  // 返回光线与物体的交点信息

    Intersection isect2= getIntersection(node->left,ray);
    Intersection isect1= getIntersection(node->right,ray);
    return isect1.distance < isect2.distance ? isect1:isect2;   // 返回左子树和右子树中距离光线起点最近的交点信息
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){    // 叶子节点
        // node是MeshTriangle.bvh的，node->object->Sample是triangle.Sample
        node->object->Sample(pos, pdf);
        pdf *= node->area; // ？乘回来不就是1了吗？
        // 不太懂，目前的理解中，真正的pdf在BVHAccel::Sample计算
        return;
    }
    // 如果随机数 p 小于左子树节点的面积，则在左子树中继续进行采样。
    // 递归调用 getSample 函数，传递给左子树，并更新 p 值和 pdf
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    // 在单个MeshTriangle中随机选取一个面积，这里的MeshTriangle应该是表示光源
    // 相当于根据选取的面积在MeshTriangle.bvh中随机选取一个叶子节点
    // 将root->area看作一个线段，叶子节点将线段划分成多个区域
    // 在线段中随机选取一个点，就是随机选取一个叶子节点
    float p = std::sqrt(get_random_float()) * root->area;   // 均匀采样
    getSample(root, p, pos, pdf);
    pdf /= root->area;  // 这里才是真正的pdf，是指在整个MeshTriangle中对每个点均匀采样
}