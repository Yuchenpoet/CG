#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
    primitives(std::move(p))
{
    time_t start, stop;
    time(&start); //记录开始构建BVH的时间
    if (primitives.empty()) //如果没有物体，直接返回
        return;

    root = recursiveBuild(primitives); //递归构建BVH

    time(&stop); //记录构建BVH结束的时间
    double diff = difftime(stop, start); //计算构建BVH所用的时间
    int hrs = (int)diff / 3600; //计算小时数
    int mins = ((int)diff / 60) - (hrs * 60); //计算分钟数
    int secs = (int)diff - (hrs * 3600) - (mins * 60); //计算秒数

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs); //输出构建BVH完成，以及所用的时间
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    //首先是创建一个节点
    //然后遍历每个物体的getBounds方法获取包围它们的AABB的边界
    //然后通过Union函数把这些AABB合为一个大的AABB作为节点
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    //根据当前AABB内物体的个数来决定划分的方法
    if (objects.size() == 1) {//如果只有一个物体，那么它就作为叶节点，并把它的子节点设为空
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {//如果是有两个物体，那么就就将这两个物体分别放入两个子节点中进行遍历，这样就保证了每个节点最多只有一个物体了
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {//如果大于两个物体，那么就要划分当前的BVH
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)//遍历物体
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());//通过getBounds()的Centroid()方法找到所有物体的中心,通过Union函数找到它们覆盖的范围
        int dim = centroidBounds.maxExtent();//通过maxExtent()方法判断到底是哪个轴覆盖的范围大
        //返回0就是x轴范围大，返回1就是y轴范围大，返回2就是z轴范围大
        //根据不同的情况，将物体根据中心坐标分别按照x、y、z轴排序，
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
        //排序完后将物体对半划分，分别放入两个子节点中进行递归，直到只剩一个物体为止
        //最后将当前节点的AABB赋值为两个子节点的Union
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
    // 定义三个交点结构体
    Intersection intersect, intersectl, intersectr;
    // 检查光线是否与 BVH 节点的包围盒相交
    if (!node->bounds.IntersectP(ray, ray.direction_inv))
        return intersect;
    // 如果当前 BVH 节点是叶子节点，则计算光线与物体的交点
    if (node->left == nullptr && node->right == nullptr) {
        intersect = node->object->getIntersection(ray);
        return intersect;
    }
    // 如果当前 BVH 节点不是叶子节点，则递归地计算光线与左右子节点的交点
    intersectl = getIntersection(node->left, ray);
    intersectr = getIntersection(node->right, ray);
    // 比较两个交点的距离，返回距离较小的那个交点
    return intersectl.distance < intersectr.distance ? intersectl : intersectr;
}