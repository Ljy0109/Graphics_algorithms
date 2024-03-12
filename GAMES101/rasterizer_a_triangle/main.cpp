#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

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
    zNear = -zNear;
    zFar = -zFar;

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

int main(int argc, const char** argv)
{
    // 输入 程序名 -r 20（旋转角度） 图像名
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700); // 初始化渲染器对象，(w,h)

    Eigen::Vector3f eye_pos = {0, 5, 25}; // 视图向量

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; // 顶点坐标

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) { // 如果程序输入正确
        // 同时初始化color和depth缓冲器
        // 使用位运算同时表示两个种类，很精妙的用法
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 根据旋转角度获得旋转矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    Eigen::Vector3f axis= Eigen::Vector3f(0.f,1.f,0.f);

    while (key != 27) { // ASCII码值为27对应的字符是ESC
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        Eigen::Vector3f axis{0,1,0};

        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
