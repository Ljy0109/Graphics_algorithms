//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {    // Scene的bvh是以宏观物体为对象
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{   // this在成员函数中使用，相当于一个实例化的对象的指针
    // 也就是说this->bvh相当于Scene x.bvh
    // 好处是即使不知道实例化对象的名称也可以使用
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{   // 这里的pos表示光源采样点的位置和法向量
    float emit_area_sum = 0; // 遍历场景中的所有物体，如果某个物体具有发射光，则将其发光面积加到 emit_area_sum 变量中。
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;   // 生成一个随机数 p，取值范围为 [0, emit_area_sum)
    emit_area_sum = 0;  
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                // MeshTriangle.Sample
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if(depth>5) return Vector3f();  // 相当于反射次数
    //Dont have Intersection In Scene
    // 递归求解ray在scene.bvh中最先相交的triangle的交点
    Intersection intersToScene = Scene::intersect(ray); 
    if (!intersToScene.happened)
        return Vector3f();
    if (intersToScene.m->hasEmission()) // 检查材质是否具有发射光,即打到光源
        return intersToScene.m->getEmission();

    Vector3f L_dir={0, 0, 0},L_indir= {0, 0, 0};

    //Calculate the Intersection from point to light in order to calculate direct Color
    // 有交点，且不是光源，先对光源采样
    Intersection LightPos;  // 光源采样点的位置和法向量
    float lightpdf = 0.0f;  // lightpdf=1？ 
    sampleLight(LightPos, lightpdf);    // LightPos没有获得emit？ -> 在MeshTriangle.Sample中获得的
    
    // 从光源发出一条光线，判断是否能打到该物体，即中间是否有阻挡
    Vector3f LightDir = LightPos.coords - intersToScene.coords; // 从交点指向光源
    float dis = dotProduct(LightDir, LightDir); // 光线距离的平方，平方比开方快
    Vector3f LightDirNormal = LightDir.normalized();   // 光线的单位向量
    Ray rayToLight(intersToScene.coords, LightDirNormal);
    Intersection interToLight = Scene::intersect(rayToLight);   // 从交点指向光源的射线与物体的相交情况
    // f_r：BRDF函数的计算
    auto f_r = intersToScene.m -> eval(ray.direction, LightDirNormal, intersToScene.normal);    // ray.direction从相机指向交点，LightDirNormal从交点指向光源
    if (interToLight.distance - LightDir.norm() > -0.005)
    {   // LightDir.norm是交点到光源的距离
        // 如果从交点指向光源的射线与物体相交的距离大于交点到光源的距离，则说明光源没有遮挡
        // 渲染公式- 直接光照的贡献
        L_dir = LightPos.emit * f_r * dotProduct(LightDirNormal, intersToScene.normal) * dotProduct(-LightDirNormal, LightPos.normal) / dis / lightpdf;
    }

    //Calculate the Intersection from point to point in order to calculate indirect Color
    if(get_random_float()>RussianRoulette)  // 俄罗斯轮盘赌，按概率停止递归
        return L_dir;   // 如果光源和交点之间始终有遮挡物，则L_dir是0，否则就是该射线上累计的亮度
    
    // Material::sample 随机采样一条反射光线，从交点指向远处
    Vector3f wi =intersToScene.m->sample(ray.direction,intersToScene.normal).normalized();
    //Ray indirRay = Ray(intersToScene.coords, wi);
    Ray indirRay(intersToScene.coords, wi); // 定义这条随机采样光线，从交点指向远处
    Intersection intersToPoint = Scene::intersect(indirRay);    // 随机采样光线与物体的相交情况
    if( intersToPoint.happened && !intersToPoint.m->hasEmission())  
    {   // 如果有交点且交点不发光
        // Material::pdf 对wi均匀采样的概率密度函数，wi的范围是0，2pi，所以pdf=1/(2*pi)
        float pdf=intersToScene.m->pdf(ray.direction,wi,intersToScene.normal);
        // intersToScene.m->eval(ray.direction,wi,intersToScene.normal) 表示intersToScene处的BRDF函数
        L_indir= castRay(indirRay,depth+1) * intersToScene.m->eval(ray.direction,wi,intersToScene.normal) * dotProduct(wi,intersToScene.normal) /RussianRoulette/pdf;
    }

    return L_dir+ L_indir ;



}

