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
    Scene scene(1280, 960); //创建一个场景，分辨率为1280x960
    //模型路径
    MeshTriangle bunny("D:/Desktop/GAMES101_Homework6_S2021/Homework6/Assignment6/models/bunny/bunny.obj"); //加载一个名为bunny的模型

    scene.Add(&bunny); //将bunny模型添加到场景中
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1)); //添加一个光源到场景中
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1)); //添加另一个光源到场景中
    scene.buildBVH(); //构建场景的BVH

    Renderer r; //创建一个渲染器

    auto start = std::chrono::system_clock::now(); //记录开始渲染的时间
    r.Render(scene); //渲染场景
    auto stop = std::chrono::system_clock::now(); //记录渲染结束的时间

    std::cout << "Render complete: \n"; //输出渲染完成
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n"; //输出渲染所用的小时数
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n"; //输出渲染所用的分钟数
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n"; //输出渲染所用的秒数

    return 0;
}