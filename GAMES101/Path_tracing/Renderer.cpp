//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include<stdlib.h>
#include<pthread.h>
#include<omp.h>
//linux library for getting cpu num
#include "unistd.h"

#include "windows.h"
SYSTEM_INFO sysInfo;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;


/*
void RenderWithMultiThread(int minY,int maxY,std::vector<Vector3f>& framebuffer)
{
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = minY; j < maxY; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
}
*/


// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    bool IsUseMultiThread=true; // 多线程
    int spp = 500;   //  每个像素采样的次数。增加这个值可以提高图像的质量，但也会线性增加计算时间
    if(!IsUseMultiThread)
    {
        float scale = tan(deg2rad(scene.fov * 0.5));    // tan(fov/2)
        float imageAspectRatio = scene.width / (float)scene.height; // 宽高比
        Vector3f eye_pos(278, 273, -800);   // 相机位置
        int m = 0;
        
        // change the spp value to change sample ammount
        std::cout << "SPP: " << spp << "\n";
        for (uint32_t j = 0; j < scene.height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                // 以左上角为原点的屏幕坐标系下像素变换到设备坐标系NDC
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                // 始终是右手系，但是旋转了相机，相当于将相机从指向-z的方向变为了指向+z的方向
                Vector3f dir = normalize(Vector3f(-x, y, 1));   // 按理说应该是(x,y,-1)才对
                for (int k = 0; k < spp; k++){  
                    // 根据射线与场景中物体的交互，计算每次采样的颜色值，并累加到framebuffer
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
                m++;
            }
            UpdateProgress(j / (float)scene.height);
        }
        UpdateProgress(1.f);
                // save framebuffer to file
        FILE* fp = fopen("binaryWithoutMultiThread.ppm", "wb");
        (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
        for (auto i = 0; i < scene.height * scene.width; ++i) {
            static unsigned char color[3];
            color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
            color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
            color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
            fwrite(color, 1, 3, fp);
        }
        fclose(fp); 
    }
    else
    {

        float pocess=0;

        float scale = tan(deg2rad(scene.fov * 0.5));
        float imageAspectRatio = scene.width / (float)scene.height;
        Vector3f eye_pos(278, 273, -800);
        std::cout << "SPP: " << spp << "\n"; 

        GetSystemInfo( &sysInfo );
        int cpuNum = sysInfo.dwNumberOfProcessors;
        // int cpuNum= sysconf(_SC_NPROCESSORS_CONF); Linux中的用法
        
        std::cout<<"Cpu Num :" <<cpuNum<<std::endl;
        
        omp_set_num_threads(cpuNum);

        //int handle[cpuNum];
        float minY=0.0f,maxY=0.0f;
        
        int m = 0;
        int hxw=scene.width*scene.height;
        #pragma omp parallel for
        for (uint32_t p = 0; p < hxw; ++p) {
            int i=p%scene.height;
            int j=p/scene.height;
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++){
                framebuffer[p] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
            
            //#pragma omp atomic
            //pocess++;
            //UpdateProgress( pocess/(float)scene.height);
        }
            
             //Threadlist[i]=std::thread(RenderWithMultiThread,minY,maxY);
            minY=maxY+1.0f;
        
        

        UpdateProgress(1.f);
        // save framebuffer to file
        FILE* fp = fopen("binaryWithMultiThread_And_Microfacet.ppm", "wb");
        (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
        for (auto i = 0; i < scene.height * scene.width; ++i) {
            static unsigned char color[3];
            color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
            color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
            color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
            fwrite(color, 1, 3, fp);
        }
        fclose(fp); 
    }
       
}
