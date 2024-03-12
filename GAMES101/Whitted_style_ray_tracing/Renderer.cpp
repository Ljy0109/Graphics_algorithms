#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float &deg)
{ return deg * M_PI/180.0; }

// Compute reflection direction
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{   // 实现了折射（refraction）的计算逻辑
    // 接受三个参数：I：入射光线的方向向量。N：表面法线向量。ior：介质的折射率（index of refraction）。
    
    // 计算入射光线与表面法线的夹角的余弦值
    // clamp 函数确保其在 [-1, 1] 的范围内，定义在gobal.hpp中
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior; // etai 是入射介质的折射率，通常为1，etat 是出射介质的折射率
    Vector3f n = N;

    // 如果入射角 cosi 小于0，则将其取反
    // 否则交换 etai 和 etat 的值，并将法线 n 取反。这是因为入射光线可能来自介质内部而不是外部。
    // ~~ 为什么是这个逻辑？因为正常的入射光线是从光源到折射点，所以本来cosi 就是小于0 ~~
    // 错
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    // 如果 k 小于 0，表示全反射，此时返回零向量。
    // 否则，根据折射定律计算折射光线的向量。
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

// [comment]
// Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{   // 计算菲涅尔方程中的反射系数
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {  std::swap(etai, etat); } // 从内部射出
    // Compute sini using Snell's law
    // sini = sqrtf(std::max(0.f, 1 - cosi * cosi))
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) { // 全反射
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.
// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
// \param[out] uv stores the u and v barycentric coordinates of the intersected point
// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// [/comment]
std::optional<hit_payload> trace(
        const Vector3f &orig, const Vector3f &dir,
        const std::vector<std::unique_ptr<Object> > &objects)
{   // 光线追踪中的射线和场景中物体的相交检测函数
    // orig：射线的起点
    // dir：射线的方向
    // objects：场景中的物体列表，每个物体由一个唯一指针指向
    float tNear = kInfinity;    // 无穷大
    std::optional<hit_payload> payload; // Renderer.hpp中定义的结构体
    for (const auto & object : objects)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            payload.emplace();
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            tNear = tNearK;
        }
    }

    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]
Vector3f castRay(
        const Vector3f &orig, const Vector3f &dir, const Scene& scene,
        int depth)
{   // 用于跟踪从相机发出的光线与场景中的物体之间的相互作用
    // 这里的dir是从相机打出的光线
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Vector3f hitColor = scene.backgroundColor;
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {   
        // hitpoint是从相机打出的光源与物体的交点
        Vector3f hitPoint = orig + dir * payload->tNear;    // 根据光线表达式计算
        Vector3f N; // normal
        Vector2f st; // 表面 coordinates
        // 计算光线与物体交点的位置 hitPoint、法线 N、表面坐标 st
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);
        switch (payload->hit_obj->materialType) {// 根据物体的材质类型进行不同的处理
            case REFLECTION_AND_REFRACTION:
            {   
                // 入射光线的反射方向和折射方向
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
                // 根据反射和折射方向以及交点位置，计算反射光线和折射光线的起点 
                // N * scene.epsilon 表示表面法线方向（N）乘以一个小的偏移量,以避免光线和表面碰撞时产生自相交
                // hitPoint - N * scene.epsilon 表示将光线的起始点沿着法线方向朝着物体外部移动一个小的距离 epsilon
                // hitPoint + N * scene.epsilon 表示将光线的起始点沿着法线方向朝着物体内部移动一个小的距离 epsilon
                // 根据光线方向和法线方向的夹角判断
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? // 避免光线和表面法线方向相反的情况
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                // 递归计算反射光线和折射光线在场景中的颜色
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
                // 根据菲涅尔方程计算反射比例 kr，即入射光线被反射的比例
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
            {
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                Vector3f reflectionDirection = reflect(dir, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                                           hitPoint + N * scene.epsilon :
                                           hitPoint - N * scene.epsilon;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (auto& light : scene.get_lights()) {
                    Vector3f lightDir = light->position - hitPoint; // 是从交点指向光源！！！
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir); // 光线距离的平方，平方比开平方快
                    lightDir = normalize(lightDir); //  获得距离后，归一化光线方向向量
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    // 计算从交点出发沿着与原光线相反的方向与物体的相交情况，返回hit_payload对象
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);   // 为什么是负的？

                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity; // r平方呢？->这里使用的Phong模型，该模型没有除以r平方
                }

                hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;// r平方呢？
                break;
            }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
// 渲染函数。在此函数中，我们遍历图像中的所有像素，生成主光线并将这些光线投射到场景中。帧缓冲区的内容将保存到文件中
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    // deg2rad将角度转换为弧度
    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    // 使用这个变量作为眼睛的位置来开始你的光线
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x;
            float y;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*     
            // 计算了像素位置 i 对应的水平方向上的坐标 x
            // (i + 0.5f) / float(scene.width)：将像素位置 i 映射到 [0, 1] 范围内，加上 0.5f 是为了将像素的中心作为参考点
            // * 2.0f - 1.0f：将 [0, 1] 范围内的坐标映射到 [-1, 1] 范围内，因为一般相机空间中的坐标范围是从左下角 (-1, -1) 到右上角 (1, 1)
            // * scale * imageAspectRatio：将得到的坐标乘以投影平面和视点之间的距离，以及图像的宽高比，以便保持图像的纵横比例       
            x = (((i + 0.5f) / float(scene.width) * 2.0f) - 1.0f) * scale * imageAspectRatio;
            // (j + 0.5f) / float(scene.height)：将像素位置 j 映射到 [0, 1] 范围内，加上 0.5f 是为了将像素的中心作为参考点
            // * 2.0f：将 [0, 1] 范围内的坐标映射到 [0, 2] 范围内，以便后续的坐标变换
            // 1.0f - ...：倒置坐标，使得原点在屏幕的左上角，而不是左下角
            // * scale：将得到的坐标乘以投影平面和视点之间的距离，以便将坐标转换到相机空间中
            y = (1.0f - (j + 0.5f) / (float)scene.height * 2.0f) * scale;
            //  x 和 y 分量表示在相机空间中水平和垂直方向上的偏移量，而 z 分量设为 -1，表示光线沿着相机朝向的方向
            Vector3f dir = normalize( Vector3f(x, y, -1)); // Don't forget to normalize this direction!
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height); // 可视化进度条
    }

    // save framebuffer to file
    // 将帧缓冲区保存到文件中
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
