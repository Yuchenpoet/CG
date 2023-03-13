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
    Scene scene(1280, 960); // ����һ���������ֱ���Ϊ 1280 x 960

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2); // ����һ�����壬λ��Ϊ (-1, 0, -12)���뾶Ϊ 2
    sph1->materialType = DIFFUSE_AND_GLOSSY; // ���ò�������Ϊ������͹���
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8); // ������������ɫ

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5); // ������һ�����壬λ��Ϊ (0.5, -0.5, -8)���뾶Ϊ 1.5
    sph2->ior = 1.5; // ����������
    sph2->materialType = REFLECTION_AND_REFRACTION; // ���ò�������Ϊ���������

    scene.Add(std::move(sph1)); // ��������ӵ�������
    scene.Add(std::move(sph2)); // ����һ��������ӵ�������

    Vector3f verts[4] = { {-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16} }; // �����ĸ�����
    uint32_t vertIndex[6] = { 0, 1, 3, 1, 2, 3 }; // ���嶥������
    Vector2f st[4] = { {0, 0}, {1, 0}, {1, 1}, {0, 1} }; // ������������
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st); // ����һ������������
    mesh->materialType = DIFFUSE_AND_GLOSSY; // ���ò�������Ϊ������͹���

    scene.Add(std::move(mesh)); // ��������������ӵ�������
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5)); // ���һ����Դ��λ��Ϊ (-20, 70, 20)��ǿ��Ϊ 0.5
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5)); // �����һ����Դ��λ��Ϊ (30, 50, -12)��ǿ��Ϊ 0.5

    Renderer r; // ����һ����Ⱦ��
    r.Render(scene); // ��Ⱦ����

    return 0;
}