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
    // �����������I�ͷ���N֮��ļн�����ֵ
    //clamp�Ǹ������޶���������������ʵ���ǰ�cosi��ֵ�޶�����[-1,1]֮��
    float cosi = clamp(-1, 1, dotProduct(I, N));
    // ����������ʺ�������ʵ�������
    float etai = 1, etat = ior;
    // ���巨������
    Vector3f n = N;
    //��������ӽ���1(etai)->����2(etat)����н�>90�㣻
    //��������ӽ���2->����1����н�<90,NҲҪ��������������֮��Ҳ��Ҫ��һ��
    // ����н�����ֵС��0��ȡ���෴��
    if (cosi < 0) { cosi = -cosi; }
    // ���򽻻�������ʺ�������ʵ������ʣ���������ȡ��
    else { std::swap(etai, etat); n = -N; }
    // ����������ʺ�������ʵ������ʱ�ֵ
    float eta = etai / etat;
    // ����kֵ
    float k = 1 - eta * eta * (1 - cosi * cosi);
    // ���kС��0������0�����򷵻���������
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
    // �����������I�ͷ���N֮��ļн�����ֵ
    float cosi = clamp(-1, 1, dotProduct(I, N));
    // ����������ʺ�������ʵ�������
    float etai = 1, etat = ior;
    // ����н�����ֵ����0������������ʺ�������ʵ�������
    if (cosi > 0) { std::swap(etai, etat); }
    // ����˹�������ɼ������������ֵ
    //snell���ɣ�sin�ȵ��������ʷ���
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // ������������ֵ���ڵ���1������ȫ���䣬����1
    if (sint >= 1) {
        return 1;
    }
    else {
        // �������������ֵ
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        // ȡ�н�����ֵ�ľ���ֵ
        cosi = fabsf(cosi);
        // ����Rs��Rp��Rs��Rp�ֱ�����˴�ֱ���ˮƽ��ķ���������ϵ��
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        // ����ƽ������ϵ��
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

// ����һ����Ϊ trace �ĺ������������ĸ�����������һ�� std::optional<hit_payload> ���͵�ֵ
std::optional<hit_payload> trace(
    const Vector3f& orig, // ��һ��������һ������ Vector3f ���͵����ã���ʾ���ߵ����
    const Vector3f& dir, // �ڶ�������Ҳ��һ������ Vector3f ���͵����ã���ʾ���ߵķ���
    const std::vector<std::unique_ptr<Object> >& objects) // ������������һ������ std::vector<std::unique_ptr<Object> > ���͵����ã���ʾ�����е������б�
{
    float tNear = kInfinity; // ����һ�������ͱ��� tNear �������ʼ��Ϊ kInfinity
    std::optional<hit_payload> payload; // ����һ�� std::optional<hit_payload> ���͵ı��� payload
    for (const auto& object : objects) // ���������е�ÿ������
    {
        float tNearK = kInfinity; // ����һ�������ͱ��� tNearK �������ʼ��Ϊ kInfinity
        uint32_t indexK; // ����һ�� uint32_t ���͵ı��� indexK
        Vector2f uvK; // ����һ�� Vector2f ���͵ı��� uvK
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear) // ��������뵱ǰ�����ཻ�� tNearK С�� tNear
        {
            payload.emplace(); // ���� payload �� emplace ��������һ���µ� hit_payload ����
            payload->hit_obj = object.get(); // �� hit_obj ��Ա����Ϊ��ǰ�����ָ��
            payload->tNear = tNearK; // �� tNear ��Ա����Ϊ tNearK
            payload->index = indexK; // �� index ��Ա����Ϊ indexK
            payload->uv = uvK; // �� uv ��Ա����Ϊ uvK
            tNear = tNearK; // ���� tNear ��ֵΪ tNearK
        }
    }
    //payload����Ч�غɣ��ڴ���������ͨ��һ�㽲�����Ŷ����õ�����
    return payload; // ���� payload ��ֵ
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
            //���䷽��
            Vector3f reflectionDirection = normalize(reflect(dir, N));
            //���䷽��
            Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
            //���ڼ�����������������������ھ���N����ƫ�ƣ���������������-N����ƫ��
            //epsilon�Ƕ����һ����������bias,epsilon=0.00001
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                hitPoint - N * scene.epsilon :
                hitPoint + N * scene.epsilon;
            //�������������ֿ�ʼ��Ϊ������ߣ��ݹ�ʹ��castRay()׷��
            //�����depth+1��˼�Ƿ�����һ�η��䣬������������->maxDepth
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
            //���㷴�����ռ��
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            //������ɫֵ
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
            //��������
            case REFLECTION:
            {
                //����������->�õ��������ռ��kr
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                //���㷴�����߷���
                Vector3f reflectionDirection = reflect(dir, N);
                //���ڼ�����������������������ھ���N����ƫ�ƣ���������������-N����ƫ��
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                    hitPoint + N * scene.epsilon ://epsilon�Ƕ����һ����������bias,epsilon=0.00001
                    hitPoint - N * scene.epsilon;
                //���ŵݹ�ʹ��castRay()�Ѽ���ķ������Orig�ͷ���Dir���룬��*�������ռ��kr��������������ͷ�������
                //�����depth+1��˼�Ƿ�����һ�η��䣬������������->maxDepth
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default:
            {
                // ��ʼ��������;��淴����ɫ
                Vector3f lightAmt = 0, specularColor = 0;
                // ƫ��У��
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                    hitPoint + N * scene.epsilon :
                    hitPoint - N * scene.epsilon;
                // ���������е����й�Դ
                for (auto& light : scene.get_lights()) {
                    // ������߷���
                    Vector3f lightDir = light->position - hitPoint;
                    // ������߾����ƽ��
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    // ��һ�����߷���
                    lightDir = normalize(lightDir);
                    // ������߷���ͷ���֮��ļн�����ֵ
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // ������Ӱ����
                    //һ��shadow ray��hitpoint��������lightdirΪ�������shadow_res = true->˵�����ڵ���
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    // ������ص�tnear<lightdistance -> shadow ray��·���ϵ�ĳһ�������ڵ��ˣ����ﲻ��light�����˵��hitpoint������ray����Ӱ��
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);
                    // �������Ӱ�У����ۼӹ�����ɫ�������ۼӹ�����ɫ
                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    //��������ray�����õ�reflect(),����������lightdir������Ǵӹ�Դ�������������lightDirҪȡ-lightDir
                    //���㷴��ray����Ϊ������߹���׼��
                    Vector3f reflectionDirection = reflect(-lightDir, N);
                    // �ۼӾ��淴����ɫ
                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }
                // ����������ɫ
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
    // ����һ��framebuffer����СΪ�����Ŀ�ȳ��Ը߶�
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    // �������ű���
    float scale = std::tan(deg2rad(scene.fov * 0.5f));//fov�ӳ��ǳ���2��ת��Ϊ�����ƺ�ȡ����ֵ
    // ����ͼ���߱�
    float imageAspectRatio = scene.width / (float)scene.height;
    // �������λ��Ϊ(0,0,0)
    Vector3f eye_pos(0);
    // ����ÿ������
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            float x;
            float y;
            // ����x��y��ֵ
            x = 2 * scale * imageAspectRatio / scene.width * (i + 0.5) - scale * imageAspectRatio;
            y = -2 * scale / scene.height * (j + 0.5) + scale;
            //����dir
            //����ֱ������������������곯��(0,0,-1)����screen space���Ϳ���ʡȥscreen space ->world space��ת��
            Vector3f dir = Vector3f(x, y, -1);
            dir = normalize(dir);// ���㷽����������һ��
            // ����castRay������������洢��framebuffer��
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        // ���½�����
        UpdateProgress(j / (float)scene.height);
    }
    // ���ļ���д��ppm��ʽ��framebufferͼ������
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