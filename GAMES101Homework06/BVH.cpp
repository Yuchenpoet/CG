#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
    primitives(std::move(p))
{
    time_t start, stop;
    time(&start); //��¼��ʼ����BVH��ʱ��
    if (primitives.empty()) //���û�����壬ֱ�ӷ���
        return;

    root = recursiveBuild(primitives); //�ݹ鹹��BVH

    time(&stop); //��¼����BVH������ʱ��
    double diff = difftime(stop, start); //���㹹��BVH���õ�ʱ��
    int hrs = (int)diff / 3600; //����Сʱ��
    int mins = ((int)diff / 60) - (hrs * 60); //���������
    int secs = (int)diff - (hrs * 3600) - (mins * 60); //��������

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs); //�������BVH��ɣ��Լ����õ�ʱ��
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    //�����Ǵ���һ���ڵ�
    //Ȼ�����ÿ�������getBounds������ȡ��Χ���ǵ�AABB�ı߽�
    //Ȼ��ͨ��Union��������ЩAABB��Ϊһ�����AABB��Ϊ�ڵ�
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    //���ݵ�ǰAABB������ĸ������������ֵķ���
    if (objects.size() == 1) {//���ֻ��һ�����壬��ô������ΪҶ�ڵ㣬���������ӽڵ���Ϊ��
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {//��������������壬��ô�;ͽ�����������ֱ���������ӽڵ��н��б����������ͱ�֤��ÿ���ڵ����ֻ��һ��������
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {//��������������壬��ô��Ҫ���ֵ�ǰ��BVH
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)//��������
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());//ͨ��getBounds()��Centroid()�����ҵ��������������,ͨ��Union�����ҵ����Ǹ��ǵķ�Χ
        int dim = centroidBounds.maxExtent();//ͨ��maxExtent()�����жϵ������ĸ��Ḳ�ǵķ�Χ��
        //����0����x�᷶Χ�󣬷���1����y�᷶Χ�󣬷���2����z�᷶Χ��
        //���ݲ�ͬ������������������������ֱ���x��y��z������
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        //�����������԰뻮�֣��ֱ���������ӽڵ��н��еݹ飬ֱ��ֻʣһ������Ϊֹ
        //��󽫵�ǰ�ڵ��AABB��ֵΪ�����ӽڵ��Union
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // ������������ṹ��
    Intersection intersect, intersectl, intersectr;
    // �������Ƿ��� BVH �ڵ�İ�Χ���ཻ
    if (!node->bounds.IntersectP(ray, ray.direction_inv))
        return intersect;
    // �����ǰ BVH �ڵ���Ҷ�ӽڵ㣬��������������Ľ���
    if (node->left == nullptr && node->right == nullptr) {
        intersect = node->object->getIntersection(ray);
        return intersect;
    }
    // �����ǰ BVH �ڵ㲻��Ҷ�ӽڵ㣬��ݹ�ؼ�������������ӽڵ�Ľ���
    intersectl = getIntersection(node->left, ray);
    intersectr = getIntersection(node->right, ray);
    // �Ƚ���������ľ��룬���ؾ����С���Ǹ�����
    return intersectl.distance < intersectr.distance ? intersectl : intersectr;
}