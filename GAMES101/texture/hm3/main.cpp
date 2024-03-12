#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();  //  旋转矩阵是单位阵

    Eigen::Matrix4f translate;  // 将视点位置移动到原点
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    rotation_angle=rotation_angle/180.0f*MY_PI;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model<<cos(rotation_angle),-sin(rotation_angle),0,0,
           sin(rotation_angle),cos(rotation_angle),0,0,
           0,0,1,0,
           0,0,0,1;
    return model;
}

//Rodrigues rotation formula
Eigen::Matrix4f get_rotation(Eigen::Vector3f axis,float angle)
{
    angle=angle/180.0f*MY_PI;
    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f ResultMat3 = Eigen::Matrix3f::Identity();
    N<< 0,-axis[2],axis[1],
        axis[2],0,-axis[0],
        -axis[1],axis[0],0;
    ResultMat3 = E*cos(angle) +(1-cos(angle))*axis*axis.transpose()+ sin(angle)*N;
    Result<<ResultMat3(0,0),ResultMat3(0,1),ResultMat3(0,2),0,
            ResultMat3(1,0),ResultMat3(1,1),ResultMat3(1,2),0,
            ResultMat3(2,0),ResultMat3(2,1),ResultMat3(2,2),0,
            0,0,0,1;
    
    return Result;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // zNear = -zNear;
    // zFar = -zFar;

    Eigen::Matrix4f m_projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_scale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_translate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_pro_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_viewport = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    eye_fov=eye_fov/180*MY_PI;

    float r_l = 2*aspect_ratio*abs(zNear)*tan(eye_fov/2.0f);
    float t_b = 2*abs(zNear)*tan(eye_fov/2.0f);

    m_scale<<   2/r_l, 0, 0, 0,
                0, 2/t_b, 0, 0,
                0, 0, 2/(zNear - zFar), 0,
                0, 0, 0, 1;
    
    m_translate<<   1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    m_ortho = m_scale * m_translate;

    m_pro_ortho<<   zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear + zFar, -zNear*zFar,
                    0, 0, 1, 0;

    m_viewport<<    350, 0, 0, 350,
                    0, 350, 0, 350,
                    0, 0, 1, 0, 
                    0, 0, 0, 1;
    // m_projection<<1/(aspect_ratio*tan(eye_fov/2.0f)) ,0,0,0,
    //             0,1/tan(eye_fov/2.0f),0,0,
    //             0,0,-(zFar+zNear)/(zFar-zNear),2*zFar*zNear/(zNear-zFar),
    //             0,0,-1,0;
    m_projection = m_ortho * m_pro_ortho;

    return m_projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    // 将法向量可视化
    // .normalized()的范围是[-1,1]
    // return_color的范围是[0,1]
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{   // 计算入射向量 vec 和法线 axis 之间的夹角的余弦值。
    // 使用反射向量的公式：reflect_vec = 2 * costheta * axis - vec。其中 costheta 是夹角的余弦值，axis 是法线向量，vec 是入射向量。
    // 归一化反射向量，以确保其为单位向量。
    // 返回归一化后的反射向量。
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{   // 定义点光源的光线结构体
    Eigen::Vector3f position;
    Eigen::Vector3f intensity; // 通常情况下，光的强度在三个颜色通道（红、绿、蓝）上都有一个值
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color=payload.texture->getColorBilinear(payload.tex_coords.x(),payload.tex_coords.y());
        //return_color=payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // 环境光系数
    Eigen::Vector3f kd = texture_color / 255.f; // 漫反射系数
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // 高光系数

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2}; // 点光源的数组
    Eigen::Vector3f amb_light_intensity{10, 10, 10}; // 环境光的强度
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150; // 高光公式中的p

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos; // 目标点在观测坐标系下的坐标
    Eigen::Vector3f normal = payload.normal; // 法向量

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f LightDir=light.position-point;  // 光线向量，从point指向light
        Eigen::Vector3f ViewDir=eye_pos-point;  // 观测向量，从point指向eye
        float d=LightDir.dot(LightDir); // 光线距离的平方
        Eigen::Vector3f H=(LightDir.normalized()+ViewDir.normalized()).normalized(); // 是光线方向向量和视线方向向量的和，然后归一化得到的半程向量。
        Eigen::Vector3f Ambient= ka.cwiseProduct(amb_light_intensity); // 对两个向量进行逐元素相乘操作,等同于*
        float LdotN=(normal.normalized()).dot(LightDir.normalized());  // 光线方向向量和表面法线的夹角的余弦值。
        float NdotH=(H.normalized()).dot(normal.normalized());  // 表面法线和半程向量的夹角的余弦值。
        Eigen::Vector3f Diffuse= std::max( LdotN , 0.0f)*kd.cwiseProduct(light.intensity/d);    // 漫反射公式
        Eigen::Vector3f Specular= std::pow(std::max( NdotH , 0.0f),150)*ks.cwiseProduct(light.intensity/d); // 高光公式
        result_color+=Ambient+Diffuse+Specular;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f LightDir=light.position-point;
        Eigen::Vector3f ViewDir=eye_pos-point;
        float d=LightDir.dot(LightDir);
        Eigen::Vector3f H=(LightDir.normalized()+ViewDir.normalized()).normalized();
        Eigen::Vector3f Ambient= ka.cwiseProduct(amb_light_intensity);
        float LdotN=(normal.normalized()).dot(LightDir.normalized());
        float NdotH=(H.normalized()).dot(normal.normalized());
        Eigen::Vector3f Diffuse= std::max( LdotN , 0.0f)*kd.cwiseProduct(light.intensity/d);
        Eigen::Vector3f Specular= std::pow(std::max( NdotH , 0.0f),150)*ks.cwiseProduct(light.intensity/d);
        result_color+=Ambient+Diffuse+Specular;
    }

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    // 位移贴图，改变三角形顶点位置
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;   // 用于控制位移贴图的效果
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    float x=normal.x(),y=normal.y(),z=normal.z();
    Vector3f t =Vector3f(x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z)); // 切线向量
    Vector3f b = normal.cross(t);   //副切线向量 n叉乘t
    Eigen:Matrix3f TBN ;    // 切线-副切线-法线矩阵
    TBN<<t.x(),b.x(),normal.x(),
         t.y(),b.y(),normal.y(),
         t.z(),b.z(),normal.z();
    float u=payload.tex_coords.x(); // 纹理坐标
    float v=payload.tex_coords.y();
    float w=payload.texture->width;
    float h=payload.texture->height;
    // 纹理坐标是归一化之后的，所以加1/w就是加一个纹理坐标
    // 计算水平和竖直方向的变化梯度
    float dU = kh * kn * (payload.texture->getColor(u+1/w,v).norm()-payload.texture->getColor(u,v).norm());
    float dV = kh * kn * (payload.texture->getColor(u,v+1/h).norm()-payload.texture->getColor(u,v).norm());
    Vector3f ln = Vector3f(-dU, -dV, 1);
    point += kn*normal*payload.texture->getColor(u,v).norm(); // 根据纹理来位移点
    Eigen::Vector3f result_color = {0, 0, 0};
    normal = (TBN * ln).normalized();

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f LightDir=light.position-point;
        Eigen::Vector3f ViewDir=eye_pos-point;
        float d=LightDir.dot(LightDir);
        Eigen::Vector3f H=(LightDir.normalized()+ViewDir.normalized()).normalized();
        Eigen::Vector3f Ambient= ka.cwiseProduct(amb_light_intensity);
        float LdotN=(normal.normalized()).dot(LightDir.normalized());
        float NdotH=(H.normalized()).dot(normal.normalized());
        Eigen::Vector3f Diffuse= std::max( LdotN , 0.0f)*kd.cwiseProduct(light.intensity/d);
        Eigen::Vector3f Specular= std::pow(std::max( NdotH , 0.0f),150)*ks.cwiseProduct(light.intensity/d);
        result_color+=Ambient+Diffuse+Specular;

    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    // 凹凸贴图，纹理包含的其实是颜色信息 
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    float x=normal.x(),y=normal.y(),z=normal.z();
    Vector3f t =Vector3f(x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z));
    Vector3f b = normal.cross(t);
    Eigen:Matrix3f TBN ;
    TBN<<t.x(),b.x(),normal.x(),
         t.y(),b.y(),normal.y(),
         t.z(),b.z(),normal.z();
    float u=payload.tex_coords.x();
    float v=payload.tex_coords.y();
    float w=payload.texture->width;
    float h=payload.texture->height;
    float dU = kh * kn * (payload.texture->getColor(u+1/w,v).norm()-payload.texture->getColor(u,v).norm());
    float dV = kh * kn * (payload.texture->getColor(u,v+1/h).norm()-payload.texture->getColor(u,v).norm());
    Vector3f ln = Vector3f(-dU, -dV, 1);
     
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = (TBN * ln).normalized();

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i+j].Position.X, mesh.Vertices[i+j].Position.Y, mesh.Vertices[i+j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i+j].Normal.X, mesh.Vertices[i+j].Normal.Y, mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j, mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y);
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}