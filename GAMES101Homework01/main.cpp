#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();//4*4��λ����
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;//�õ�Ч����������ƶ������෴�ľ���
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
        0, 0, 0, 1;//���ɵ���ת����
    return model;
}

//�����ֱ�Ϊ����ӳ��ǣ��ӿڿ�߱ȣ����ü�������Զ�ü������
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
{//������������� axis �� �Ƕ� angle�������Eigen�������� Matrix4f    
    angle = angle / 180.0f * MY_PI;//�Ƕ���ת������   
    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();//4x4 ��λ����(��ת����
    Eigen::Matrix3f E = Eigen::Matrix3f::Identity();//3x3 ��λ����ŷ���ǣ�    
    Eigen::Matrix3f N = Eigen::Matrix3f::Identity();//3x3 ��λ���� ����ת��ĵ�λ������   
    Eigen::Matrix3f ResultMat3 = Eigen::Matrix3f::Identity();//3x3 ��λ����    
    N << 0, -axis[2], axis[1],   //�� X ����ת        
        axis[2], 0, -axis[0],   //�� Y ����ת        
        -axis[1], axis[0], 0;   //�� Z ����ת         
        //��ת����ļ��㹫ʽ��R = E * cos(angle) + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N    
    ResultMat3 = E * cos(angle) + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N;
    Result << ResultMat3(0, 0), ResultMat3(0, 1), ResultMat3(0, 2), 0,   //��ֵ�����������        
        ResultMat3(1, 0), ResultMat3(1, 1), ResultMat3(1, 2), 0,
        ResultMat3(2, 0), ResultMat3(2, 1), ResultMat3(2, 2), 0,
        0, 0, 0, 1;
    return Result;//���ؽ��
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

    rst::rasterizer r(700, 700);//ʵ������դ��r�����÷ֱ���Ϊ700x700

    Eigen::Vector3f eye_pos = {0, 0, 5};//���λ��

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//��������������

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};//�������

    auto pos_id = r.load_positions(pos);//������ͳ��򴫸���դ��
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
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);//���������Ҳ���ǰ�frame_buf �� depth_buf ��ʼ��Ϊȫ0 ��ȫ�����

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(0, 0, 1), angle));//����MVP�任��
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0f, 1.0f, 0.1f, 50.0f));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//���û��ƺ��������붥��ID������ID�Լ�����ͼԪ����

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());//700��700�� 32F��32λ���㣬C3��3ͨ�������һλ�Ǿ������ֵ
        image.convertTo(image, CV_8UC3, 1.0f);//CV_8UC3: ��ͨ����ÿ��ͨ����unsigned char���͵�����
        cv::imshow("image", image);
        key = cv::waitKey(10);//һ��ѭ��������10ms��ʱ��ȴ��û�����esc����

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {//��ת������ƣ�a��ʱ����ת��d˳ʱ����ת
            angle += 1.0f;
        }
        else if (key == 'd') {
            angle -= 1.0f;
        }
    }

    return 0;
}
