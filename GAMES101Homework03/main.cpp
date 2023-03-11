#include <iostream>
#include "opencv2/opencv.hpp"

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f change,proj;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    eye_fov = eye_fov / 180 * MY_PI;
    proj << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;//透视投影矩阵

    double w, h, z;
    h = -zNear * tan(eye_fov / 2) * 2;
    w = h * aspect_ratio;
    z = zFar - zNear;

    change << 2 / w, 0, 0, 0,
        0, 2 / h, 0, 0,
        0, 0, 2 / z, -(zFar + zNear) / 2,
        0, 0, 0, 1;//正交投影矩阵，默认x0y平面视角是中心

    return change * proj;
}

//顶点着色器
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

//法线贴图
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload) {
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f; // 将法线归一化并将其范围转换到[0, 1]
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255; // 将结果转换到[0, 255]范围
    return result;
}

//计算反射向量
static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis) {
    // 计算两个向量的夹角余弦值
    auto costheta = vec.dot(axis);
    // 计算反射向量
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

//纹理贴图
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload) {
    // 初始化返回颜色
    Eigen::Vector3f return_color = { 0, 0, 0 };
    // 如果有纹理，则获取纹理值
    if (payload.texture) {
        return_color = payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y());
    }
    // 转换颜色到[0, 1]范围
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();
    // 计算环境光、漫反射和镜面反射系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;//通过这一步来把纹理着色上去
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);


    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    Eigen::Vector3f amb, dif, spe, l, v;



    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        l = (light.position - point).normalized();
        v = (eye_pos - point).normalized() + l;
        dif = kd.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * std::fmax(0, normal.dot(l));
        spe = ks.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * pow(std::fmax(0, normal.dot(v.normalized())), p);
        amb = ka.cwiseProduct(amb_light_intensity);
        result_color += (dif + spe + amb);
    }

    return result_color * 255.f;
}

//blinn-phong光照模型
//部分注释可以参考后续dump mapping和displacement mapping
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    //定义环境光,漫反射，镜面反射系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    //定义光源、光强、视点位置
    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };
    
    //镜面反射的指数,一块金属表面的镜面反射指数可能在50-100之间，而一块油漆表面的镜面反射指数可能在2-5之间
    float p = 150;

    //传参
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };

    Eigen::Vector3f amb, dif, spe, l, v;
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // 计算漫反射、镜面反射和环境光分量
        l = (light.position - point).normalized();
        v = (eye_pos - point).normalized() + l;
        dif = kd.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * std::fmax(0, normal.dot(l));
        spe = ks.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * pow(std::fmax(0, normal.dot(v.normalized())), p);
        amb = ka.cwiseProduct(amb_light_intensity);
        result_color += (dif + spe + amb);
    } 
    // 返回最终颜色
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    //注释参照后续dumpmapping代码中的注释
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

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

    float x, y, z;
    Vector3f t, b;
    x = normal.x(), y = normal.y(), z = normal.z();
    t << x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z* y / sqrt(x * x + z * z);
    b = normal.cross(t);

    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u, v, w, h;
    u = payload.tex_coords.x();
    v = payload.tex_coords.y();
    w = payload.texture->width;
    h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColorBilinear(u + 1.0 / w, v).norm() - payload.texture->getColorBilinear(u, v).norm());
    float dV = kh * kn * (payload.texture->getColorBilinear(u, v + 1.0 / h).norm() - payload.texture->getColorBilinear(u, v).norm());

    Vector3f ln;
    ln << -dU, dV, 1;

    point += kn * normal * payload.texture->getColorBilinear(u, v).norm();
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };

    //与dump mapping代码从这里开始不同，这个导出的图片效果是添加了材质的效果，而dump mapping仅表示了颜色
    Eigen::Vector3f amb, dif, spe, l, v1;
    for (auto& light : lights) {
        // 计算漫反射、镜面反射和环境光分量
        l = (light.position - point).normalized();
        v1 = (eye_pos - point).normalized() + l;
        dif = kd.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * std::fmax(0, normal.dot(l));
        spe = ks.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * pow(std::fmax(0, normal.dot(v1.normalized())), p);
        amb = ka.cwiseProduct(amb_light_intensity);
        // 累加到最终颜色中
        result_color += (dif + spe + amb);
    }

    // 返回最终颜色
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    //定义环境光,漫反射，镜面反射系数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };//定义了两个光源
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    //定义光源、光强、视点位置
    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    //传参
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
     
    // 定义凹凸纹理系数
    float kh = 0.2, kn = 0.1;

    // TODO: bump mapping具体公式计算步骤如下
    // Let n = normal = (x, y, z)    n是表面法线，它表示物体表面的凹凸纹理，它的值为(x, y, z)，其中x、y、z分别表示法线的x、y、z方向的值。
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))   t是tan向量，它是由表面法线n和x、y、z坐标轴的叉积计算得出的
    // Vector b = n cross product t  b是binormal向量，它是由表面法线n和tan向量t的叉积计算得出的。它可以用来表示物体表面的接触方向，从而实现bump mapping的效果。
    // Matrix TBN = [t b n]  TBN矩阵是tangent-binormal-normal矩阵，它是由tan向量t、binormal向量b和表面法线n组成的3x3矩阵，它可以用来将物体表面的法线转换到世界坐标系中，从而实现bump mapping的效果。
    // dU = kh * kn * (h(u+1/w,v)-h(u,v)) dU和dV是bump mapping中的偏导数，它们分别表示u方向和v方向上的偏导数，它们可以用来计算出法线ln
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    //以下是代码
    float x, y, z;
    Vector3f t, b;
    x = normal.x(), y = normal.y(), z = normal.z();
    t << x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z* y / sqrt(x * x + z * z);
    b = normal.cross(t);

    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u, v, w, h;
    u = payload.tex_coords.x();
    v = payload.tex_coords.y();
    w = payload.texture->width;
    h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColorBilinear(u + 1.0 / w, v).norm() - payload.texture->getColorBilinear(u, v).norm());
    float dV = kh * kn * (payload.texture->getColorBilinear(u, v + 1.0 / h).norm() - payload.texture->getColorBilinear(u, v).norm());

    Vector3f ln;
    ln << -dU, dV, 1;

    point += kn * normal * payload.texture->getColorBilinear(u, v).norm();
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    //替换成自己的文件夹所在路径
    std::string filename = "D:/Desktop/output.png";
    objl::Loader Loader;
    std::string obj_path = "D:/Desktop/GAMES101_Homework3_S2021/Homework3/Assignment3/models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("D:/Desktop/GAMES101_Homework3_S2021/Homework3/Assignment3/models/spot/spot_triangulated_good.obj");
    for (auto mesh : Loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle* t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
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

    Eigen::Vector3f eye_pos = { 0,0,10 };

    //通过不同的调试选项调用不同的函数进行着色，返回不同的颜色，顶点着色器不变，片元着色器变化
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        // 清除颜色缓冲和深度缓冲
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // 设置模型矩阵
        r.set_model(get_model_matrix(angle));
        // 设置视图矩阵
        r.set_view(get_view_matrix(eye_pos));
        // 设置投影矩阵
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
        // 绘制三角形列表
        r.draw(TriangleList);
        // 将帧缓冲转换为OpenCV格式的图像
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // 将图像从浮点数转换为8位无符号整数
        image.convertTo(image, CV_8UC3, 1.0f);
        // 将图像从RGB转换为BGR
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        // 保存图像
        cv::imwrite(filename, image);
        // 返回0
        return 0;
    }


    while (key != 27)
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

        if (key == 'a')
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