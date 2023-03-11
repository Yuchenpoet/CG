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
        0, 0, 1, 0;//͸��ͶӰ����

    double w, h, z;
    h = -zNear * tan(eye_fov / 2) * 2;
    w = h * aspect_ratio;
    z = zFar - zNear;

    change << 2 / w, 0, 0, 0,
        0, 2 / h, 0, 0,
        0, 0, 2 / z, -(zFar + zNear) / 2,
        0, 0, 0, 1;//����ͶӰ����Ĭ��x0yƽ���ӽ�������

    return change * proj;
}

//������ɫ��
Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

//������ͼ
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload) {
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f; // �����߹�һ�������䷶Χת����[0, 1]
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255; // �����ת����[0, 255]��Χ
    return result;
}

//���㷴������
static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis) {
    // �������������ļн�����ֵ
    auto costheta = vec.dot(axis);
    // ���㷴������
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

//������ͼ
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload) {
    // ��ʼ��������ɫ
    Eigen::Vector3f return_color = { 0, 0, 0 };
    // ������������ȡ����ֵ
    if (payload.texture) {
        return_color = payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y());
    }
    // ת����ɫ��[0, 1]��Χ
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();
    // ���㻷���⡢������;��淴��ϵ��
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;//ͨ����һ������������ɫ��ȥ
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

//blinn-phong����ģ��
//����ע�Ϳ��Բο�����dump mapping��displacement mapping
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    //���廷����,�����䣬���淴��ϵ��
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    //�����Դ����ǿ���ӵ�λ��
    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };
    
    //���淴���ָ��,һ���������ľ��淴��ָ��������50-100֮�䣬��һ���������ľ��淴��ָ��������2-5֮��
    float p = 150;

    //����
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };

    Eigen::Vector3f amb, dif, spe, l, v;
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // ���������䡢���淴��ͻ��������
        l = (light.position - point).normalized();
        v = (eye_pos - point).normalized() + l;
        dif = kd.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * std::fmax(0, normal.dot(l));
        spe = ks.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * pow(std::fmax(0, normal.dot(v.normalized())), p);
        amb = ka.cwiseProduct(amb_light_intensity);
        result_color += (dif + spe + amb);
    } 
    // ����������ɫ
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    //ע�Ͳ��պ���dumpmapping�����е�ע��
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

    //��dump mapping��������￪ʼ��ͬ�����������ͼƬЧ��������˲��ʵ�Ч������dump mapping����ʾ����ɫ
    Eigen::Vector3f amb, dif, spe, l, v1;
    for (auto& light : lights) {
        // ���������䡢���淴��ͻ��������
        l = (light.position - point).normalized();
        v1 = (eye_pos - point).normalized() + l;
        dif = kd.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * std::fmax(0, normal.dot(l));
        spe = ks.cwiseProduct(light.intensity / ((light.position - point).dot(light.position - point))) * pow(std::fmax(0, normal.dot(v1.normalized())), p);
        amb = ka.cwiseProduct(amb_light_intensity);
        // �ۼӵ�������ɫ��
        result_color += (dif + spe + amb);
    }

    // ����������ɫ
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    //���廷����,�����䣬���淴��ϵ��
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };//������������Դ
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    //�����Դ����ǿ���ӵ�λ��
    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    //����
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
     
    // ���尼͹����ϵ��
    float kh = 0.2, kn = 0.1;

    // TODO: bump mapping���幫ʽ���㲽������
    // Let n = normal = (x, y, z)    n�Ǳ��淨�ߣ�����ʾ�������İ�͹��������ֵΪ(x, y, z)������x��y��z�ֱ��ʾ���ߵ�x��y��z�����ֵ��
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))   t��tan�����������ɱ��淨��n��x��y��z������Ĳ������ó���
    // Vector b = n cross product t  b��binormal�����������ɱ��淨��n��tan����t�Ĳ������ó��ġ�������������ʾ�������ĽӴ����򣬴Ӷ�ʵ��bump mapping��Ч����
    // Matrix TBN = [t b n]  TBN������tangent-binormal-normal����������tan����t��binormal����b�ͱ��淨��n��ɵ�3x3�����������������������ķ���ת������������ϵ�У��Ӷ�ʵ��bump mapping��Ч����
    // dU = kh * kn * (h(u+1/w,v)-h(u,v)) dU��dV��bump mapping�е�ƫ���������Ƿֱ��ʾu�����v�����ϵ�ƫ���������ǿ����������������ln
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    //�����Ǵ���
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

    //�滻���Լ����ļ�������·��
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

    //ͨ����ͬ�ĵ���ѡ����ò�ͬ�ĺ���������ɫ�����ز�ͬ����ɫ��������ɫ�����䣬ƬԪ��ɫ���仯
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        // �����ɫ�������Ȼ���
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // ����ģ�;���
        r.set_model(get_model_matrix(angle));
        // ������ͼ����
        r.set_view(get_view_matrix(eye_pos));
        // ����ͶӰ����
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
        // �����������б�
        r.draw(TriangleList);
        // ��֡����ת��ΪOpenCV��ʽ��ͼ��
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        // ��ͼ��Ӹ�����ת��Ϊ8λ�޷�������
        image.convertTo(image, CV_8UC3, 1.0f);
        // ��ͼ���RGBת��ΪBGR
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        // ����ͼ��
        cv::imwrite(filename, image);
        // ����0
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