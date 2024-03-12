#pragma once

#include <cmath>
#include <iostream>
#include <random>

#define M_PI 3.14159265358979323846

// 定义无穷大
constexpr float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float& lo, const float& hi, const float& v)
{   // 将一个值v限制在指定的范围[lo, hi]内
    return std::max(lo, std::min(hi, v));
}

inline bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{   // 用于解二次方程ax^2 + bx + c = 0
    // 如果方程有解，函数返回true，并且通过引用参数x0和x1返回两个根（如果只有一个根，则x0 = x1）。
    // 这个函数对于光线追踪中计算光线与球体等几何体的交点非常有用。
    float discr = b * b - 4 * a * c;
    if (discr < 0)
        return false;
    else if (discr == 0)
        x0 = x1 = -0.5 * b / a;
    else
    {
        float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1)
        std::swap(x0, x1);
    return true;
}

enum MaterialType
{
    DIFFUSE_AND_GLOSSY, // 漫反射和光泽
    REFLECTION_AND_REFRACTION,  // 反射和折射
    REFLECTION  // 仅反射
};

inline float get_random_float()
{   // 生成一个在[0, 1]范围内的随机浮点数
    // 模拟光线散射、抖动等随机过程
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(rng);
}

inline void UpdateProgress(float progress)
{   // 用于可视化地显示进度条
    // 对于长时间运行的渲染过程来说是一个很好的用户反馈机制
    // 通过计算progress参数来决定进度条的填充状态
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}
