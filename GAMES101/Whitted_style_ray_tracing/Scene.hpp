#pragma once

#include <vector>
#include <memory>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"

class Scene
{
public:
    // setting up options
    int width = 1280;
    int height = 960;
    double fov = 90;    // 视场角
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);   // 背景颜色
    int maxDepth = 5;   // 最大递归深度，用于控制光线追踪的递归层数
    float epsilon = 0.00001;    // 交点的微小偏移量

    Scene(int w, int h) : width(w), height(h)
    {}

    void Add(std::unique_ptr<Object> object) { objects.push_back(std::move(object)); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    // [[nodiscard]] 是 C++11 中的属性（attribute），用于标记函数的返回值，表示调用函数时不应忽略其返回值
    // 第一个 const 关键字修饰了成员函数自身，表示这个成员函数是一个常量成员函数。这意味着在这个函数内部，不能修改类的任何成员变量，除非这些成员变量被声明为 mutable。
    // 第二个 const 关键字修饰了函数的返回值类型，表示返回的对象是一个常量对象。这意味着调用者不能通过返回的引用来修改 objects 成员变量的内容，因为返回的引用是 const 的。
    [[nodiscard]] const std::vector<std::unique_ptr<Object> >& get_objects() const { return objects; }
    [[nodiscard]] const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }

private:
    // creating the scene (adding objects and lights)
    std::vector<std::unique_ptr<Object> > objects;
    std::vector<std::unique_ptr<Light> > lights;
};