#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960);
    
    // 创建第一个球体Sphere对象sph1，位置在(-1, 0, -12)，半径为2。
    // 设置其材质为DIFFUSE_AND_GLOSSY，并指定一种蓝绿色调的漫反射颜色。
    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);
    sph1->materialType = DIFFUSE_AND_GLOSSY;
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);

    // 创建第二个球体Sphere对象sph2，位置在(0.5, -0.5, -8)，半径为1.5。
    // 设置其材质为REFLECTION_AND_REFRACTION，折射率(ior)为1.5，这表示该球体会有反射和折射效果。
    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
    sph2->ior = 1.5;
    sph2->materialType = REFLECTION_AND_REFRACTION;

    // 使用Scene的Add方法将这两个球体添加到场景中。注意，这里使用了std::move来转移所有权
    scene.Add(std::move(sph1));
    scene.Add(std::move(sph2));

    // 定义一个简单的平面网格，由四个顶点定义，并通过索引指定两个三角形。该网格用于表示一个四边形平面。
    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);
    mesh->materialType = DIFFUSE_AND_GLOSSY;

    scene.Add(std::move(mesh));
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    Renderer r;
    r.Render(scene);

    return 0;
}