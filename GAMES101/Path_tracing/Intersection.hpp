//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{   
    // happened: 表示光线是否与物体相交。
    // coords: 相交点的坐标。
    // tcoords: 相交点的纹理坐标。
    // normal: 相交点处的法向量。
    // emit: 相交点处的辐射能量（用于光源）。
    // distance: 光线从射线原点到相交点的距离。初始化为无穷大，也就是不相交
    // obj: 与光线相交的物体指针。
    // m: 相交物体的材质指针。
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f tcoords;
    Vector3f normal;
    Vector3f emit;
    double distance;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
