#pragma once

#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object
{   // 一个球体类 Sphere，它是 Object 类的子类，因此继承了 Object 类的一些成员和方法
public:
    Sphere(const Vector3f& c, const float& r)   // 初始化球体的中心点和半径
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // 实现了与光线的相交检测。它采用解析方法来计算光线与球体的相交点。
        // 如果存在相交点，则返回 true，并将最接近相机的相交点的距离存储在参数 tnear 中
        // analytic solution
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        tnear = t0;

        return true;
    }

    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
                              Vector3f& N, Vector2f&) const override
    {   // 重构了Object类中的虚函数getSurfaceProperties
        // 在球体中，法线方向即为从球心指向相交点的方向
        N = normalize(P - center);
    }

    Vector3f center;
    float radius, radius2;
};
