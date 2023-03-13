#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960); // 创建一个场景，分辨率为 1280 x 960

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2); // 创建一个球体，位置为 (-1, 0, -12)，半径为 2
    sph1->materialType = DIFFUSE_AND_GLOSSY; // 设置材质类型为漫反射和光泽
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8); // 设置漫反射颜色

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5); // 创建另一个球体，位置为 (0.5, -0.5, -8)，半径为 1.5
    sph2->ior = 1.5; // 设置折射率
    sph2->materialType = REFLECTION_AND_REFRACTION; // 设置材质类型为反射和折射

    scene.Add(std::move(sph1)); // 将球体添加到场景中
    scene.Add(std::move(sph2)); // 将另一个球体添加到场景中

    Vector3f verts[4] = { {-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16} }; // 定义四个顶点
    uint32_t vertIndex[6] = { 0, 1, 3, 1, 2, 3 }; // 定义顶点索引
    Vector2f st[4] = { {0, 0}, {1, 0}, {1, 1}, {0, 1} }; // 定义纹理坐标
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st); // 创建一个网格三角形
    mesh->materialType = DIFFUSE_AND_GLOSSY; // 设置材质类型为漫反射和光泽

    scene.Add(std::move(mesh)); // 将网格三角形添加到场景中
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5)); // 添加一个光源，位置为 (-20, 70, 20)，强度为 0.5
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5)); // 添加另一个光源，位置为 (30, 50, -12)，强度为 0.5

    Renderer r; // 创建一个渲染器
    r.Render(scene); // 渲染场景

    return 0;
}