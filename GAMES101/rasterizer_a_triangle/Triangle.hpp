//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>

using namespace Eigen;
class Triangle
{
  public:
    // 三角形的三个顶点坐标，三维的
    Vector3f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in
                      counter clockwise order*/
    /*Per vertex values*/
    // 三个顶点的rgb
    Vector3f color[3];      // color at each vertex;
    // 纹理坐标uv
    Vector2f tex_coords[3]; // texture u,v
    // 三个顶点的法向量
    Vector3f normal[3];     // normal vector for each vertex

    // Texture *tex;
    Triangle();

    // 返回三维的向量（坐标），abc（const成员函数）分别表示三个顶点的坐标
    Eigen::Vector3f a() const { return v[0]; }
    Eigen::Vector3f b() const { return v[1]; }
    Eigen::Vector3f c() const { return v[2]; }

    void setVertex(int ind, Vector3f ver); /*set i-th vertex coordinates */
    void setNormal(int ind, Vector3f n);   /*set i-th vertex normal vector*/
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/
    void setTexCoord(int ind, float s,
                     float t); /*set i-th vertex texture coordinate*/
    std::array<Vector4f, 3> toVector4() const;
};

#endif // RASTERIZER_TRIANGLE_H
