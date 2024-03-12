#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

Triangle::Triangle()
{
    // 初始化顶点坐标
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    // 初始化顶点颜色rgb
    color[0] << 0.0, 0.0, 0.0;
    color[1] << 0.0, 0.0, 0.0;
    color[2] << 0.0, 0.0, 0.0;

    // 初始化顶点纹理坐标uv
    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

// 将三维坐标ver赋值给顶点数组v的第ind个元素
void Triangle::setVertex(int ind, Eigen::Vector3f ver) { v[ind] = ver; }

// 将三维向量n赋值为法向量normal的第ind个元素
void Triangle::setNormal(int ind, Vector3f n) { normal[ind] = n; }

void Triangle::setColor(int ind, float r, float g, float b)
{
    if ((r < 0.0) || (r > 255.) || (g < 0.0) || (g > 255.) || (b < 0.0) ||
        (b > 255.)) {
        throw std::runtime_error("Invalid color values");
    }
    
    // 颜色归一化，0-1
    color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    return;
}
void Triangle::setTexCoord(int ind, float s, float t)
{
    // 将(s,t)赋值为第ind个纹理坐标
    tex_coords[ind] = Vector2f(s, t);
}

// 静态变量成员函数
std::array<Vector4f, 3> Triangle::toVector4() const
{
    // 将顶点坐标变为四维齐次坐标
    std::array<Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](const Vector3f& vec) {
        return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
    });
    return res;
}
