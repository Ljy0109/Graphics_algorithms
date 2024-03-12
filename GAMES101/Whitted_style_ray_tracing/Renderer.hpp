#pragma once
#include "Scene.hpp"

struct hit_payload
{   // 指向与光线相交的物体的指针
    float tNear;    // 光线与物体的最近相交点的时间
    uint32_t index; // 与光线相交的物体在场景中的索引
    Vector2f uv;    // 存储了相交点的纹理坐标
    Object* hit_obj;// 指向与光线相交的物体的指针
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};