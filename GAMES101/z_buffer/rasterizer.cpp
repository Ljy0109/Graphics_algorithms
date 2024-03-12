//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return rst::pos_buf_id(id);
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return rst::ind_buf_id(id);
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

static Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{   // 将顶点坐标变为四维齐次坐标
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   // 通过向量叉乘判断二维点是否在三角形内
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P=Vector3f(x,y,_v[0].z());
    Vector3f AC=_v[2]-_v[0];
    Vector3f CB=_v[1]-_v[2];
    Vector3f BA=_v[0]-_v[1];
    Vector3f AP=P-_v[0];
    Vector3f BP=P-_v[1];
    Vector3f CP=P-_v[2];
    
    //if cross product in the same direction ,its inside the triangle
    if(AP.cross(AC).dot(BP.cross(BA))>0.0f&&
        BP.cross(BA).dot(CP.cross(CB))>0.0f&&
        CP.cross(CB).dot(AP.cross(AC))>0.0f)
        {
            return true;
        }
        return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{   // 计算重心坐标,表示点 (x, y) 关于三角形三个顶点的重心坐标 
    // 见https://ljy0109.github.io/2024/02/29/%E7%8E%B0%E4%BB%A3%E8%AE%A1%E7%AE%97%E6%9C%BA%E5%9B%BE%E5%BD%A2%E5%AD%A6%E5%9F%BA%E7%A1%80/
    // 的4.7节
    // Barycentric coordinates
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}


// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
// https://ljy0109.github.io/2024/03/06/Bresenham%E5%B8%83%E9%9B%B7%E6%A3%AE%E6%9B%BC%E7%AE%97%E6%B3%95/
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1; //p0初始化
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2; // xe:x_end
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color); // 初始点赋值
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0) // pk<0: d1<d2
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0)) // 斜率是正的
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color); // 画线
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);  // 初始点pixel赋值
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}



void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, col_buf_id col_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {   // 目前还没有实现三角形以外的绘图基元
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    // n=0.1,f=100
    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;
    
    // MVP: model view projection 模型视图投影矩阵
    // 将模型投影到视图平面
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f), // 变换三个顶点
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        t.w_a = v[0].w();
        t.w_b = v[1].w();
        t.w_c = v[2].w();

        for (auto& vec : v) {
            // 坐标归一化
            vec /= vec.w();
        }

        for (auto & vert : v)
        {   // 此处从正交投影变为了屏幕投影
            // 所以不需要在之前计算投影矩阵时算上屏幕投影的矩阵
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2; // 近平面和远平面的拉伸
        }

        for (int i = 0; i < 3; ++i)
        {   // 将顶点赋值给三角形类的顶点
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        //rasterize_wireframe(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{   // 画线
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    bool IsUseSurperSampling=true;

    //Sample Offset
    //超采样像素内的四个点(0.25,0.25);(0.25,0.75);(0.75,0.25);(0.75,0.75)
    std::vector<Vector3f> SampleOffset={
        {0.25f,0.25f,0},
        {0.25f,0.75f,0},
        {0.75f,0.25f,0},
        {0.75f,0.75f,0}
    };

    //First: Ceater bounding box
    float minX=t.v[0].x(),maxX=t.v[0].x(),minY=t.v[0].y(),maxY=t.v[0].y(),minZ=t.v[0].z(),maxZ=t.v[0].z();
    for(auto& v : t.v)
    {
        minX=std::min(minX,v.x());
        maxX=std::max(maxX, v.x());
        minY=std::min(minY,v.y());
        maxY=std::max(maxY, v.y());
        minZ=std::min(minZ,v.z());
        maxZ=std::max(maxZ, v.z());
    }
    // min下取整,max上取整
    minX=(int)std::floor(minX);maxX=(int)std::ceil(maxX);minY=(int)std::floor(minY);maxY=(int)std::ceil(maxY);
    //int minX_Int=int(std::floor(minX)),maxX_Int=int(std::ceil(maxX)),minY_Int=int(std::floor(minY)),maxY_Int=int(std::ceil(maxY)),minZ_Int=int(std::floor(minZ)),maxZ_Int=int(std::ceil(maxZ));
    for(int i=minX;i<maxX;i++)
    {   // 遍历包围盒内的点
        for(int j=minY;j<maxY;j++)
        {
            //check whether the point is inside the triangle
            int Index=get_index(i,j);
            int l=0;
            int IsInTriangleCount=0;
            int IsDontBeCover=0;
            if(IsUseSurperSampling)
            {
                float depth_4 = 0;
                //SurperSampling
                for(auto&k : SampleOffset)
                {      
                    float SampleX=i+k.x();
                    float SampleY=j+k.y();
                    if(insideTriangle(SampleX,SampleY,t.v))
                    {

                        auto[alpha, beta, gamma] = computeBarycentric2D(SampleX, SampleY, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        // 上下两种z_interpolated计算结果一致，但是不知道上面计算公式的推导，所以直接看下面这个好了
                        // 根据投影校正系数，将二维平面坐标还原回三维坐标，从而计算出平面点对应的深度值
                        // https://www.zhihu.com/question/332096916
                        float z_interpolated2 = (alpha*v[0].z()/t.w_a + beta*v[1].z()/t.w_b + gamma*v[2].z()/t.w_c)
                                                 /(alpha/t.w_a + beta/t.w_b + gamma/t.w_c);
                        // check zbuff
                        if(z_interpolated < surpersample_depth_buf[Index][l])
                        {   // 更新depth缓冲器里最小的深度
                            surpersample_depth_buf[Index][l]=z_interpolated;
                            depth_buf[Index]=z_interpolated;
                            if(min_depth > z_interpolated){
                                min_depth = z_interpolated;
                            }
                            if(max_depth < z_interpolated){
                                max_depth = z_interpolated;
                            }
                            IsDontBeCover++;
                        }
                        IsInTriangleCount++;
                    }
                    l=l+1;
                }
                Vector3f color= t.getColor()*IsInTriangleCount/4.0f;

                if(IsDontBeCover>0)
                {
                    set_pixel(Vector3f(i,j,0),color);
                }
            }
            else
            {
                if(insideTriangle(i+0.5f,j+0.5f,t.v))
                {

                    auto[alpha, beta, gamma] = computeBarycentric2D(i+0.5f, j+0.5f, t.v);
                    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    // z_interpolated *= w_reciprocal;
                    // 注释见上文
                    float z_interpolated = (alpha*v[0].z()/t.w_a + beta*v[1].z()/t.w_b + gamma*v[2].z()/t.w_c)
                                            /(alpha/t.w_a + beta/t.w_b + gamma/t.w_c);
                    // check zbuff
                    if(z_interpolated<depth_buf[Index])
                    {
                        depth_buf[Index]=z_interpolated;
                        set_pixel(Vector3f(i,j,1),t.getColor());
                        
                    }
                }
            }
        }
    }
}

// 创建深度值对应的灰度图
cv::Mat rst::rasterizer::createDepthImage(int width, int height) {
    cv::Mat depth_image(height, width, CV_8UC1);

    // 根据深度值计算灰度值并设置图像像素
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int index = i * width + j;
            if(depth_buf[index] == std::numeric_limits<float>::infinity()){
                depth_buf[index] = max_depth;
            }
            // 将深度值映射到灰度级别，范围为0到255
            // int gray_value = (depth_buf[index] - min_depth) / (max_depth - min_depth) * 255;
            int gray_value = (max_depth - depth_buf[index]) / (max_depth - min_depth) * 255;
            depth_image.at<uchar>(i, j) = static_cast<uchar>(gray_value);
        }
    }

    return depth_image;
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{   // 初始化缓冲器
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::array<float,4> inf;
        inf.fill(std::numeric_limits<float>::infinity()); 
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(surpersample_depth_buf.begin(), surpersample_depth_buf.end(), inf);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{   // rasterizer类的构造函数
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    surpersample_corlor_buf.resize(w * h);
    surpersample_depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{   // 
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;
    auto ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

