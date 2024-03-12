#pragma once

#include "Vector.hpp"

// 定义一个描述光线的类
// 这个类具有光源位置和光强的信息
class Light
{
public:
    Light(const Vector3f& p, const Vector3f& i)
        : position(p)
        , intensity(i)
    {}
    virtual ~Light() = default;
    Vector3f position;
    Vector3f intensity; // 用三通道来表示
};
