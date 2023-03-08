#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();//4*4单位矩阵
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;//得到效果和摄像机移动方向相反的矩阵
    view = translate * view;
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model << cos(rotation_angle), -sin(rotation_angle), 0, 0,
        sin(rotation_angle), cos(rotation_angle), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;//生成的旋转矩阵
    return model;
}

//参数分别为相机视场角，视口宽高比，近裁剪面距离和远裁剪面距离
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    eye_fov = eye_fov / 180 * MY_PI;
    projection << 1 / (aspect_ratio * tan(eye_fov)), 0, 0, 0,
        0, 1 / tan(eye_fov), 0, 0,
        0, 0, (zFar + zNear) / (zFar - zNear), 2 * zFar * zNear / (zNear - zFar),
        0, 0, 1, 0;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{//输入参数：向量 axis 和 角度 angle，输出：Eigen矩阵类型 Matrix4f    
    angle = angle / 180.0f * MY_PI;//角度制转弧度制   
    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();//4x4 单位矩阵(旋转矩阵）
    Eigen::Matrix3f E = Eigen::Matrix3f::Identity();//3x3 单位矩阵（欧拉角）    
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();//3x3 单位矩阵 （旋转轴的单位向量）   
    Eigen::Matrix3f ResultMat3 = Eigen::Matrix3f::Identity();//3x3 单位矩阵    
    N << 0, -axis[2], axis[1],   //绕 X 轴旋转        
        axis[2], 0, -axis[0],   //绕 Y 轴旋转        
        -axis[1], axis[0], 0;   //绕 Z 轴旋转         
        //旋转矩阵的计算公式：R = E * cos(angle) + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N    
    ResultMat3 = E * cos(angle) + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N;
    Result << ResultMat3(0, 0), ResultMat3(0, 1), ResultMat3(0, 2), 0,   //赋值到结果矩阵中        
        ResultMat3(1, 0), ResultMat3(1, 1), ResultMat3(1, 2), 0,
        ResultMat3(2, 0), ResultMat3(2, 1), ResultMat3(2, 2), 0,
        0, 0, 0, 1;
    return Result;//返回结果
}

int main(int argc, const char** argv)
{
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

    rst::rasterizer r(700, 700);//实例化光栅器r，设置分辨率为700x700

    Eigen::Vector3f eye_pos = {0, 0, 5};//相机位置

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//三角形三个顶点

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};//相机朝向

    auto pos_id = r.load_positions(pos);//把相机和朝向传给光栅器
    auto ind_id = r.load_indices(ind);//

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//清除画布，也就是把frame_buf 和 depth_buf 初始化为全0 和全无穷大

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(0, 0, 1), angle));//进行MVP变换，
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//调用绘制函数，输入顶点ID和索引ID以及绘制图元方法

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//700×700， 32F是32位浮点，C3是3通道，最后一位是具体的数值
        image.convertTo(image, CV_8UC3, 1.0f);//CV_8UC3: 三通道、每个通道是unsigned char类型的数据
        cv::imshow("image", image);
        key = cv::waitKey(10);//一个循环函数，10ms的时间等待用户按键esc触发

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {//旋转输入控制：a逆时针旋转，d顺时针旋转
            angle += 1.0f;
        }
        else if (key == 'd') {
            angle -= 1.0f;
        }
    }

    return 0;
}
