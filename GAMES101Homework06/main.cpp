#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    Scene scene(1280, 960); //����һ���������ֱ���Ϊ1280x960
    //ģ��·��
    MeshTriangle bunny("D:/Desktop/GAMES101_Homework6_S2021/Homework6/Assignment6/models/bunny/bunny.obj"); //����һ����Ϊbunny��ģ��

    scene.Add(&bunny); //��bunnyģ����ӵ�������
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1)); //���һ����Դ��������
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1)); //�����һ����Դ��������
    scene.buildBVH(); //����������BVH

    Renderer r; //����һ����Ⱦ��

    auto start = std::chrono::system_clock::now(); //��¼��ʼ��Ⱦ��ʱ��
    r.Render(scene); //��Ⱦ����
    auto stop = std::chrono::system_clock::now(); //��¼��Ⱦ������ʱ��

    std::cout << "Render complete: \n"; //�����Ⱦ���
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n"; //�����Ⱦ���õ�Сʱ��
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n"; //�����Ⱦ���õķ�����
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n"; //�����Ⱦ���õ�����

    return 0;
}