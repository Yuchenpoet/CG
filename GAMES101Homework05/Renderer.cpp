#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float &deg)
{ return deg * M_PI/180.0; }

// Compute reflection direction
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior)
{
    // 计算入射光线I和法线N之间的夹角余弦值
    //clamp是个区间限定函数，在这里其实就是把cosi的值限定在了[-1,1]之间
    float cosi = clamp(-1, 1, dotProduct(I, N));
    // 定义入射介质和折射介质的折射率
    float etai = 1, etat = ior;
    // 定义法线向量
    Vector3f n = N;
    //如果入射光从介质1(etai)->介质2(etat)，则夹角>90°；
    //如果入射光从介质2->介质1，则夹角<90,N也要反过来，折射率之比也需要换一下
    // 如果夹角余弦值小于0，取其相反数
    if (cosi < 0) { cosi = -cosi; }
    // 否则交换入射介质和折射介质的折射率，并将法线取反
    else { std::swap(etai, etat); n = -N; }
    // 计算入射介质和折射介质的折射率比值
    float eta = etai / etat;
    // 计算k值
    float k = 1 - eta * eta * (1 - cosi * cosi);
    // 如果k小于0，返回0，否则返回折射向量
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}



// [comment]
// Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
float fresnel(const Vector3f& I, const Vector3f& N, const float& ior)
{
    // 计算入射光线I和法线N之间的夹角余弦值
    float cosi = clamp(-1, 1, dotProduct(I, N));
    // 定义入射介质和折射介质的折射率
    float etai = 1, etat = ior;
    // 如果夹角余弦值大于0，交换入射介质和折射介质的折射率
    if (cosi > 0) { std::swap(etai, etat); }
    // 根据斯涅尔定律计算折射角正弦值
    //snell定律：sin比等于折射率反比
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // 如果折射角正弦值大于等于1，发生全反射，返回1
    if (sint >= 1) {
        return 1;
    }
    else {
        // 计算折射角余弦值
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        // 取夹角余弦值的绝对值
        cosi = fabsf(cosi);
        // 计算Rs和Rp：Rs和Rp分别代表了垂直光和水平光的菲涅尔反射系数
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        // 返回平均反射系数
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.
// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
// \param[out] uv stores the u and v barycentric coordinates of the intersected point
// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// [/comment]

// 定义一个名为 trace 的函数，它接受四个参数并返回一个 std::optional<hit_payload> 类型的值
std::optional<hit_payload> trace(
    const Vector3f& orig, // 第一个参数是一个常量 Vector3f 类型的引用，表示光线的起点
    const Vector3f& dir, // 第二个参数也是一个常量 Vector3f 类型的引用，表示光线的方向
    const std::vector<std::unique_ptr<Object> >& objects) // 第三个参数是一个常量 std::vector<std::unique_ptr<Object> > 类型的引用，表示场景中的物体列表
{
    float tNear = kInfinity; // 定义一个浮点型变量 tNear 并将其初始化为 kInfinity
    std::optional<hit_payload> payload; // 定义一个 std::optional<hit_payload> 类型的变量 payload
    for (const auto& object : objects) // 遍历场景中的每个物体
    {
        float tNearK = kInfinity; // 定义一个浮点型变量 tNearK 并将其初始化为 kInfinity
        uint32_t indexK; // 定义一个 uint32_t 类型的变量 indexK
        Vector2f uvK; // 定义一个 Vector2f 类型的变量 uvK
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear) // 如果光线与当前物体相交且 tNearK 小于 tNear
        {
            payload.emplace(); // 调用 payload 的 emplace 方法创建一个新的 hit_payload 对象
            payload->hit_obj = object.get(); // 将 hit_obj 成员设置为当前物体的指针
            payload->tNear = tNearK; // 将 tNear 成员设置为 tNearK
            payload->index = indexK; // 将 index 成员设置为 indexK
            payload->uv = uvK; // 将 uv 成员设置为 uvK
            tNear = tNearK; // 更新 tNear 的值为 tNearK
        }
    }
    //payload（有效载荷）在代码世界里通俗一点讲代表着对有用的数据
    return payload; // 返回 payload 的值
}

// [comment]
 //Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)

 //This function is the function that compute the color at the intersection point
 //of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).

 //If the material of the intersected object is either reflective or reflective and refractive,
 //then we compute the reflection/refraction direction and cast two new rays into the scene
 //by calling the castRay() function recursively. When the surface is transparent, we mix
 //the reflection and refraction color using the result of the fresnel equations (it computes
 //the amount of reflection and refraction depending on the surface normal, incident view direction
 //and surface refractive index).

 //If the surface is diffuse/glossy we use the Phong illumation model to compute the color
 //at the intersection point.
// [/comment]
Vector3f castRay(
        const Vector3f &orig, const Vector3f &dir, const Scene& scene,
        int depth)
{
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Vector3f hitColor = scene.backgroundColor;
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear;
        Vector3f N; // normal
        Vector2f st; // st coordinates
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);
        switch (payload->hit_obj->materialType) {
        case REFLECTION_AND_REFRACTION:
        {
            //反射方向
            Vector3f reflectionDirection = normalize(reflect(dir, N));
            //折射方向
            Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
            //由于计算问题会出现误差，如果点在面内就向N方向偏移；如果点在面外就向-N方向偏移
            //epsilon是定义的一个浮点数的bias,epsilon=0.00001
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            //两条光线射线又开始作为入射光线，递归使用castRay()追踪
            //这里的depth+1意思是发生了一次反射，有最大次数限制->maxDepth
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
            //计算反射光线占比
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            //计算着色值
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
            //镜面物体
            case REFLECTION:
            {
                //菲涅耳定律->得到反射光线占比kr
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                //计算反射射线方向
                Vector3f reflectionDirection = reflect(dir, N);
                //由于计算问题会出现误差，如果点在面内就向N方向偏移；如果点在面外就向-N方向偏移
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                    hitPoint + N * scene.epsilon ://epsilon是定义的一个浮点数的bias,epsilon=0.00001
                    hitPoint - N * scene.epsilon;
                //接着递归使用castRay()把计算的反射起点Orig和方向Dir带入，并*反射光线占比kr，继续跟踪折射和反射射线
                //这里的depth+1意思是发生了一次反射，有最大次数限制->maxDepth
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default:
            {
                // 初始化环境光和镜面反射颜色
                Vector3f lightAmt = 0, specularColor = 0;
                // 偏移校正
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                    hitPoint + N * scene.epsilon :
                    hitPoint - N * scene.epsilon;
                // 遍历场景中的所有光源
                for (auto& light : scene.get_lights()) {
                    // 计算光线方向
                    Vector3f lightDir = light->position - hitPoint;
                    // 计算光线距离的平方
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    // 归一化光线方向
                    lightDir = normalize(lightDir);
                    // 计算光线方向和法线之间的夹角余弦值
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // 进行阴影测试
                    //一条shadow ray从hitpoint出发，以lightdir为方向，如果shadow_res = true->说明被遮挡了
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    // 如果返回的tnear<lightdistance -> shadow ray被路径上的某一个表面遮挡了，到达不了light，因此说明hitpoint在这条ray的阴影里
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);
                    // 如果在阴影中，不累加光照颜色，否则累加光照颜色
                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    //这里求反射ray方向，用到reflect(),这个函数里的lightdir定义的是从光源出发，因此这里lightDir要取-lightDir
                    //计算反射ray方向为后面求高光做准备
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    // 累加镜面反射颜色
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }
                // 计算最终颜色
                hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
                break;
            }
        }
    }
    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]

void Renderer::Render(const Scene& scene)
{
    // 创建一个framebuffer，大小为场景的宽度乘以高度
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    // 计算缩放比例
    float scale = std::tan(deg2rad(scene.fov * 0.5f));//fov视场角除以2，转化为弧度制后取正切值
    // 计算图像宽高比
    float imageAspectRatio = scene.width / (float)scene.height;
    // 设置相机位置为(0,0,0)
    Vector3f eye_pos(0);
    // 遍历每个像素
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            float x;
            float y;
            // 计算x和y的值
            x = 2 * scale * imageAspectRatio / scene.width * (i + 0.5) - scale * imageAspectRatio;
            y = -2 * scale / scene.height * (j + 0.5) + scale;
            //定义dir
            //这里直接让摄像机在世界坐标朝向(0,0,-1)构建screen space，就可以省去screen space ->world space的转化
            Vector3f dir = Vector3f(x, y, -1);
            dir = normalize(dir);// 计算方向向量并归一化
            // 调用castRay函数并将结果存储在framebuffer中
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        // 更新进度条
        UpdateProgress(j / (float)scene.height);
    }
    // 打开文件并写入ppm格式的framebuffer图像数据
    FILE* fp = fopen("D:/Desktop/rtresult.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}