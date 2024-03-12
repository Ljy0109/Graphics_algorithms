//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{   // 用于表示三维空间中的边界框
  public:
    // 对角线上的两个端点
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {   // 创建一个未初始化的边界框
        // 样做的目的是为了确保该边界框是一个无效的边界框，因为它的最小点的坐标比最大点的坐标都要大，
        // 从而在后续使用中可以轻松地检测出这种无效情况并进行处理
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {} // 框变成了一个点
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }   // 计算边界框的对角线向量
    int maxExtent() const
    {   // 返回边界框中最长的边的索引（0表示x轴，1表示y轴，2表示z轴）
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {   // 计算边界框的表面积
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; } // 计算边界框的中心点
    Bounds3 Intersect(const Bounds3& b)
    {   // 计算两个边界框的交集
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {   // 计算点相对于边界框的偏移
        // 返回的是一个比值：p-pMin/(pMax-pMin)
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {   //  判断是否重叠
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {   // 判断点p是否在盒子内
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {   // 重构[]，也就是说Bounfs3 b[0]表示pMin，b[1]表示pMax
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{   // 测试光线是否与边界框相交,通过光线与三对平面交点的时间
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    Vector3f t_minTemp=(pMin-ray.origin)*invDir;
    Vector3f t_maxTemp=(pMax-ray.origin)*invDir;
    Vector3f t_min=Vector3f::Min(t_minTemp,t_maxTemp);
    Vector3f t_max=Vector3f::Max(t_minTemp,t_maxTemp);

    float t_min_time=std::max(t_min.x,std::max(t_min.y,t_min.z));
    float t_max_time=std::min(t_max.x,std::min(t_max.y,t_max.z));
    return t_max_time >= -std::numeric_limits<float>::epsilon()&& t_min_time <= t_max_time;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{   // 计算两个边界框的并集
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{   // 边界框和点的并集
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
