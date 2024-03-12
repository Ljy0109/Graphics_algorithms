#pragma once

#include "Object.hpp"

#include <cstring>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{   // 实现了射线和三角形的相交检测功能 MT算法
    // v0、v1、v2：三角形的三个顶点。
    // orig：射线的起始点。
    // dir：射线的方向。
    // tnear：相交点到射线起始点的距离。
    // u、v：相交点在三角形上的重心坐标。
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    Vector3f E0=v1-v0,E1=v2-v0,B=orig-v0;
    Vector3f B_Cross_E0=crossProduct(B,E0),D_Cross_E1=crossProduct(dir,E1);
    tnear= dotProduct(B_Cross_E0,E1)/(dotProduct(D_Cross_E1,E0));
    u=dotProduct(D_Cross_E1,B)/(dotProduct(D_Cross_E1,E0));
    v=dotProduct(B_Cross_E0,dir)/(dotProduct(D_Cross_E1,E0));
    return tnear>=-std::numeric_limits<float>::epsilon()&& u>=-std::numeric_limits<float>::epsilon()&& v>=-std::numeric_limits<float>::epsilon()&& 1.0f-u-v >= -std::numeric_limits<float>::epsilon();
}

class MeshTriangle : public Object
{   // // 一个三角形类，它是 Object 类的子类
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)
    {   // 构造函数接受网格模型的顶点数组、顶点索引数组、三角形数量以及纹理坐标数组作为参数，用于初始化网格模型的属性
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            // 计算一共有多少顶点
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;  // 索引是从0开始的
        // 动态分配一个具有足够大小的 Vector3f 类型对象数组，并将其管理权委托给 std::unique_ptr
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
                   Vector2f& uv) const override
    {   // 用于检测光线与网格模型中的三角形是否相交。它遍历每个三角形，调用 rayTriangleIntersect
        // const override 是 C++ 中对成员函数的特殊修饰符，用于表明该函数是一个覆盖了基类虚函数的派生类函数，并且在派生类中该函数是常量成员函数
        // override: 表明该成员函数是对基类虚函数的重写（覆盖）。它用来显式指明派生类中的函数是对基类中的虚函数的覆盖，确保基类中存在同名的虚函数，否则会产生编译错误
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {   // 获得三角平面的法向量，根据重心坐标返回交点坐标
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {   // 计算并返回网格三角形的漫反射颜色
        float scale = 5;
        // fmodf取余数，确保值在 [0,1] 的范围内
        // ^ 异或操作符
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        // lerp 是一个线性插值函数调用，根据模式值 pattern 在两种颜色之间进行插值
        // 在Vector.hpp中定义
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};

