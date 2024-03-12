//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "Vector.hpp"
#include "Light.hpp"
#include "global.hpp"

class AreaLight : public Light
{
public:
    AreaLight(const Vector3f &p, const Vector3f &i) : Light(p, i)   
    {   // 构造函数，接受光源的位置和辐射强度作为参数，并初始化光源的位置、辐射强度、法线方向、以及 u 和 v 向量
        // : Light(p, i) 是调用基类 Light 的构造函数，并将参数 p 和 i 传递给基类构造函数进行初始化。
        // 这种语法被称为基类初始化列表，它用于在派生类的构造函数中调用基类的构造函数并传递参数
        // 也就是说AreaLight本身也有position和identity
        normal = Vector3f(0, -1, 0);
        u = Vector3f(1, 0, 0);
        v = Vector3f(0, 0, 1);
        length = 100;
    }

    Vector3f SamplePoint() const
    {
        auto random_u = get_random_float();
        auto random_v = get_random_float();
        return position + random_u * u + random_v * v;
    }

    float length;
    Vector3f normal;
    Vector3f u; // 平面内的两个正交向量，用于表示点的位移
    Vector3f v;
};
