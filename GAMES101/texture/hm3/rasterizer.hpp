//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <optional>
#include "Shader.hpp"
#include "global.hpp"

using namespace Eigen;

namespace rst {
enum class Buffers
{
    Color = 1, // 01
    Depth = 2  // 10
};

inline Buffers operator|(Buffers a, Buffers b)
{   // 对color和depth进行位与和位或
    // color | depth = 11 同时表示color和depth，也就是说
    // (color | depth) & color = color
    // (color | depth) & depth = depth
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b)
{
    return Buffers((int)a & (int)b);
}

enum class Primitive
{
    Line,
    Triangle
};

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
struct pos_buf_id
{
    int pos_id = 0;
    pos_buf_id(int id) : pos_id(id) {}
};

struct ind_buf_id
{
    int ind_id = 0;
    ind_buf_id(int id) : ind_id(id) {}
};

struct col_buf_id
{
    int col_id = 0;
    col_buf_id(int id) : col_id(id) {}
};


class rasterizer
{
  public:
    rasterizer(int w, int h);
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
    col_buf_id load_colors(const std::vector<Eigen::Vector3f> &colors);
    col_buf_id load_normals(const std::vector<Eigen::Vector3f> &normals);

    void set_model(const Eigen::Matrix4f& m); // 将m赋值给model
    void set_view(const Eigen::Matrix4f& v); // 将v赋值给view
    void set_projection(const Eigen::Matrix4f& p); // 将p赋值给projection

    void set_texture(Texture tex) { texture = tex; }  // Texture类定义在Texture.hpp

    // 它接受一个参数，这个参数是一个类型为 std::function 的函数对象，
    // 该函数对象接受一个 vertex_shader_payload 类型的参数并返回一个 Eigen::Vector3f 类型的值。
    void set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader);
    void set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

    void set_pixel(const Eigen::Vector3i& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    // 类中声明两个名称相同的函数，则根据参数的输入来决定调用哪个函数
    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
    void draw(std::vector<Triangle *> &TriangleList);

    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);
    void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos);

  private:
    Eigen::Matrix4f model;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
    std::map<int, std::vector<Eigen::Vector3f>> col_buf;
    std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

    std::vector<Eigen::Vector3f> frame_buf; // 边框的颜色缓冲器

    std::vector<std::array<float,4>> surpersample_corlor_buf; // 超采样颜色缓冲器
    std::vector<std::array<float,4>> surpersample_depth_buf;
        
    std::vector<float> depth_buf;

    std::optional<Texture> texture;
    std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
    std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

    int get_index(int x, int y);

    int width, height;

    int normal_id = -1;
    int next_id = 0; // 表示当前处理的顶点索引
    int get_next_id() { return next_id++; }
};
} // namespace rst
