//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if(depth > this->maxDepth)
        return Vector3f(0.0, 0.0, 0.0);

    Intersection intersection = intersect(ray);
    if(!intersection.happened)
    {
        return this->backgroundColor;
    }
    
    auto n = normalize(intersection.normal);
    auto p = intersection.coords;
    auto m = intersection.m;
    auto wo = normalize(-ray.direction);

    Vector3f L_dir(0.0, 0.0, 0.0);    
    if(intersection.obj->hasEmit())
    {
        L_dir += intersection.emit;
    }
    else
    {
        float pdf_light;
        Intersection lightSample;
        sampleLight(lightSample, pdf_light);
        auto lightP = lightSample.coords;
        auto nn = normalize(lightSample.normal);
        auto emit = lightSample.emit;
        
        Vector3f ws = lightP - p;
        auto dist = ws.norm();
        ws = normalize(ws);
        Intersection shadowCheck = intersect(Ray(p,ws));

        if(dist - shadowCheck.distance < EPSILON)
        {
            L_dir += emit * m->eval(wo, ws, n) 
                     * dotProduct(ws, n) * dotProduct(-ws, nn)
                     / pow(dist, 2) / pdf_light;
        }
    }
    
    Vector3f L_indir(0.0, 0.0, 0.0);
    float ksi = get_random_float();
    if(ksi <= RussianRoulette)
    {
        Vector3f wi = m->sample(wo, n);
        wi = normalize(wi);
        Ray rayIndir(p, wi);

        Intersection intersIndir = intersect(rayIndir);
        if(intersIndir.happened)
        {
            if(!intersIndir.obj->hasEmit())
            {
                L_indir += castRay(rayIndir, depth + 1) * m->eval(wo, wi, n)
                           * dotProduct(wi, n) / m->pdf(wo, wi, n) / RussianRoulette;
            }
        }
    }

    return L_dir + L_indir;
}

