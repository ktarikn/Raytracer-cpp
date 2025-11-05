#include <iostream>
#include "parser.h"
#include "ppm.h"

#include <cmath>
#include <pthread.h>
#include <string.h>

#include "vectorOP.h"
#include "KDTree.h"

#define tmax 1e300
#define Vec3f parser::Vec3f
#define ZEROVEC3 Vec3f(0,0,0)

typedef unsigned char RGB[3];

parser::Scene scene;
KDNode* headKD;

struct ThreadArgs {
    parser::Camera cam;
    int row;
    unsigned char* image;
};

struct HitInfo {
    bool hit = false;
    double t = 1e300;
    Vec3f pos;
    Vec3f normal;
    int material = 1;
    Vec3f origin;
};



Ray castRay(parser::Camera cam, int i, int j) {
    Vec3f u = cross(normalize(cam.up), constMult(normalize(cam.gaze),-1));
    Vec3f m = add(cam.position, constMult(cam.gaze, cam.near_distance));
    Vec3f q = add(add(m, constMult(normalize(cam.up), cam.near_plane.w)), constMult(u, cam.near_plane.x));
    float su = (i + 0.5) * (cam.near_plane.y - cam.near_plane.x) / cam.image_width;
    float sv = (j + 0.5) * (cam.near_plane.w - cam.near_plane.z) / cam.image_height;
    Vec3f s = subtract(add(q, constMult(u, su)), constMult(normalize(cam.up), sv));
    return Ray(cam.position, normalize(subtract(s, cam.position)));
}

HitInfo intersectSphere(const Ray& ray, parser::Sphere* sphere) {
    HitInfo hit;
    Vec3f oc = subtract(ray.o, scene.vertex_data[sphere->center_vertex_id - 1]);
    
    float a = dot(ray.d, ray.d);
    float b = 2.0f * dot(oc, ray.d);
    float c = dot(oc, oc) - sphere->radius * sphere->radius;
    
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return hit;
    }
    
    float sqrtDisc = sqrt(discriminant);
    float t0 = (-b - sqrtDisc) / (2.0f * a);
    float t1 = (-b + sqrtDisc) / (2.0f * a);
    
    float t = t0;
    if (t < 0) {
        t = t1;
        if (t < 0) {
            return hit;
        }
    }
    
    hit.hit = true;
    hit.t = t;
    hit.pos = add(ray.o, constMult(ray.d, t));
    hit.normal = normalize(subtract(hit.pos, scene.vertex_data[sphere->center_vertex_id - 1]));
    hit.material = sphere->material_id;
    hit.origin = ray.o;
    
    return hit;
}

HitInfo intersectTriangle(const Ray& ray, parser::Face face, int material,bool isShadowRay) {
    HitInfo hit;
    const float EPSILON = 1e-8f;
    
    Vec3f edge1 = subtract(scene.vertex_data[face.v1_id - 1], scene.vertex_data[face.v0_id - 1]);
    Vec3f edge2 = subtract(scene.vertex_data[face.v2_id - 1], scene.vertex_data[face.v0_id - 1]);
    
    Vec3f h = cross(ray.d, edge2);
    float a = dot(edge1, h);
    
    if (fabs(a) < EPSILON) {
        return hit;
    }
    
    float f = 1.0f / a;
    Vec3f s = subtract(ray.o, scene.vertex_data[face.v0_id - 1]);
    float u = f * dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return hit;
    }
    
    Vec3f q = cross(s, edge1);
    float v = f * dot(ray.d, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return hit;
    }
    
    float t = f * dot(edge2, q);
    
    if (t > EPSILON) {
        Vec3f faceNormal = normalize(cross(edge1, edge2));
        
        // Back-face culling: only for primary/reflection rays, NOT shadow rays
        if (!isShadowRay && dot(ray.d, faceNormal) > 0) {
            return hit;
        }
        hit.hit = true;
        hit.t = t;
        hit.pos = add(ray.o, constMult(ray.d, t));
        
        
        
        hit.normal = faceNormal;
        hit.material = material;
        hit.origin = ray.o;
    }
    
    return hit;
}

HitInfo intersectCylinder(const Ray& ray, const parser::Cylinder& cyl, const std::vector<Vec3f>& vertices) {
    HitInfo hit;
    Vec3f C = vertices[cyl.center_vertex_id - 1];
    Vec3f A = cyl.axis;
    float lenA = vLength(A);
    Vec3f An = constMult(A, 1 / lenA);
    float r = cyl.radius;
    float h = cyl.height;

    Vec3f OC = subtract(ray.o, C);
    float DdotA = dot(ray.d, An);
    float OCdotA = dot(OC, An);
    Vec3f Dperp = subtract(ray.d, constMult(An, DdotA));
    Vec3f OCperp = subtract(OC, constMult(An, OCdotA));

    float a = dot(Dperp, Dperp);
    float b = 2 * dot(Dperp, OCperp);
    float c = dot(OCperp, OCperp) - r * r;

    float disc = b * b - 4 * a * c;
    float t_side = 1e30;

    if (disc >= 0) {
        float sqrtDisc = sqrt(disc);
        float t1 = (-b - sqrtDisc) / (2 * a);
        float t2 = (-b + sqrtDisc) / (2 * a);
        for (float t : {t1, t2}) {
            if (t <= 0) continue;
            float z = OCdotA + t * DdotA;
            if (fabs(z) <= h / 2 && t < t_side)
                t_side = t;
        }
    }

    float t_cap = 1e30;
    for (float sgn : {1.f, -1.f}) {
        Vec3f Pcap = add(C, constMult(An, (sgn * h * 0.5f)));
        float denom = dot(ray.d, An);
        if (fabs(denom) < 1e-6) continue;
        float t = dot(subtract(Pcap, ray.o), An) / denom;
        if (t <= 0) continue;
        Vec3f P = add(ray.o, constMult(ray.d, t));
        if (vLength(subtract(subtract(P, Pcap), constMult(An, dot(subtract(P, Pcap), An)))) <= r)
            if (t < t_cap) t_cap = t;
    }

    float tmin = std::min(t_side, t_cap);
    if (tmin < 1e29) {
        hit.hit = true;
        hit.t = tmin;
        hit.pos = add(ray.o, constMult(ray.d, tmin));
        hit.material = cyl.material_id;
        if (tmin == t_cap) {
            hit.normal = (dot(ray.d, An) > 0 ? constMult(An, -1) : An);
        } else {
            Vec3f tmp = subtract(subtract(hit.pos, C), constMult(An, dot(subtract(hit.pos, C), An)));
            hit.normal = normalize(tmp);
        }
    }

    return hit;
}

HitInfo rayIntersect(Ray ray, bool isShadowRay = false) {
    double mint = 1e300;
    HitInfo hitPoint;

    // Check planes
    for(parser::Plane p : scene.planes) {
        if(dot(ray.d, normalize(p.normal)) != 0) {
            double t = dot(subtract(scene.vertex_data[p.center_vertex_id - 1], ray.o), normalize(p.normal)) / dot(ray.d, normalize(p.normal));
            if(t > 0 && t < mint) {
                mint = t;
                hitPoint.hit = true;
                hitPoint.t = t;
                hitPoint.pos = add(ray.o, constMult(ray.d, t));
                hitPoint.normal = normalize(p.normal);
                hitPoint.material = p.material_id;
                hitPoint.origin = ray.o;
            }
        }
    }
    
    // Traverse KD-tree
    std::vector<KDNode*> leafNodes;
    collectLeafNodes(ray, headKD, leafNodes);
    
    
    // Check all objects in all relevant leaf nodes
    for(KDNode* node : leafNodes) {
        for(SceneObject* obj : node->objects) {
            HitInfo hit;
            
            if(obj->type == SPHERE) {
                hit = intersectSphere(ray, (parser::Sphere*)(obj->data));
            } 
            else if(obj->type == TRIANGLE) {
                parser::Triangle* tri = (parser::Triangle*)(obj->data);
                hit = intersectTriangle(ray, tri->indices, tri->material_id, isShadowRay);
            } 
            else if(obj->type == CYLINDER) {
                hit = intersectCylinder(ray, *(parser::Cylinder*)(obj->data), scene.vertex_data);
            }
            
            if(hit.hit && hit.t < mint) {
                hitPoint = hit;
                mint = hit.t;
            }
        }
    }
    
    return hitPoint;
}

// calculatemirror and calculatecolor call eachother
void calculateColor(HitInfo hit, unsigned char* color, int depth, parser::Camera cam);

Vec3f calculateMirrorReflection(HitInfo hit, int depth, parser::Camera cam) {
    if (depth >= scene.max_recursion_depth || !hit.hit) {
        return {0, 0, 0};
    }

    const parser::Material& mat = scene.materials[hit.material - 1];
    
    // Only apply mirror reflection if material has mirror component
    if (mat.mirror.x < 0.001f && mat.mirror.y < 0.001f && mat.mirror.z < 0.001f) {
        return {0, 0, 0};
    }

    
    Vec3f hitDir = normalize(subtract(hit.pos, hit.origin));
    Vec3f N = normalize(hit.normal);
    
    // Reflection formula: r = d - 2(d·n)n
    Vec3f reflectDir = subtract(hitDir, constMult(N, 2.0f * dot(hitDir, N)));
    reflectDir = normalize(reflectDir);
    
    // use epsilon while casting
    Ray reflectRay{
        add(hit.pos, constMult(N, scene.shadow_ray_epsilon)),
        reflectDir
    };
    
    HitInfo reflectHit = rayIntersect(reflectRay, false);
    
    Vec3f reflectedColor = {0, 0, 0};
    if (reflectHit.hit) {
        unsigned char reflectPixel[3];
        calculateColor(reflectHit, reflectPixel, depth + 1,cam);
        
        // Scale by mirror reflectance
        reflectedColor.x = (float)reflectPixel[0] * mat.mirror.x;
        reflectedColor.y = (float)reflectPixel[1] * mat.mirror.y;
        reflectedColor.z = (float)reflectPixel[2] * mat.mirror.z;
    } else {
        // Hit background
        reflectedColor.x = scene.background_color.x * mat.mirror.x;
        reflectedColor.y = scene.background_color.y * mat.mirror.y;
        reflectedColor.z = scene.background_color.z * mat.mirror.z;
    }
    
    return reflectedColor;
}

//The color of the pixel will be calculated here, back and forth calls with mirror function will take place
void calculateColor(HitInfo hit, unsigned char* color, int depth, parser::Camera cam) {
    if (!hit.hit) {
        color[0] = scene.background_color.x;
        color[1] = scene.background_color.y;
        color[2] = scene.background_color.z;
        return;
    }

    const parser::Material& mat = scene.materials[hit.material - 1];
    Vec3f N = normalize(hit.normal);
    Vec3f V = normalize(subtract(hit.origin, hit.pos));

    // Ambient
    Vec3f ambient = {
        mat.ambient.x * scene.ambient_light.x,
        mat.ambient.y * scene.ambient_light.y,
        mat.ambient.z * scene.ambient_light.z
    };

    Vec3f diffuse{0, 0, 0};
    Vec3f specular{0, 0, 0};

    
    for (parser::PointLight light : scene.point_lights) {
        Vec3f L = normalize(subtract(light.position, hit.pos));
        float lightDist = vLength(subtract(light.position, hit.pos));
        
        // Cast shadow ray
        Ray shadowRay{add(hit.pos, constMult(N, scene.shadow_ray_epsilon)), L};
        HitInfo shadowHit = rayIntersect(shadowRay, true);
        
        
        // If shadow ray hits something closer than light, skip this light
        if(shadowHit.hit && shadowHit.t < lightDist - scene.shadow_ray_epsilon)
            continue;

        // Irradiance falls off as 1/d²
        float irradiance = 1.0f / (lightDist * lightDist);
        Vec3f E = {
            light.intensity.x * irradiance,
            light.intensity.y * irradiance,
            light.intensity.z * irradiance
        };

        // Diffuse
        float NdotL = std::max(0.0f, dot(N, L));
        diffuse.x += mat.diffuse.x * E.x * NdotL;
        diffuse.y += mat.diffuse.y * E.y * NdotL;
        diffuse.z += mat.diffuse.z * E.z * NdotL;

        // Specular: BlinnPhong
        Vec3f H = normalize(add(L, V));
        float NdotH = std::max(0.0f, dot(N, H));
        float spec = pow(NdotH, mat.phong_exponent);
        specular.x += mat.specular.x * E.x * spec;
        specular.y += mat.specular.y * E.y * spec;
        specular.z += mat.specular.z * E.z * spec;
    }

    // Mirror reflection element
    Vec3f mirrorContribution = calculateMirrorReflection(hit, depth,cam);
    
    // Sum all 
    Vec3f finalColor = {
        ambient.x + diffuse.x + specular.x + mirrorContribution.x,
        ambient.y + diffuse.y + specular.y + mirrorContribution.y,
        ambient.z + diffuse.z + specular.z + mirrorContribution.z
    };

    // Clamp and round to [0, 255]
    color[0] = (unsigned char)std::min(255, (int)std::round(finalColor.x));
    color[1] = (unsigned char)std::min(255, (int)std::round(finalColor.y));
    color[2] = (unsigned char)std::min(255, (int)std::round(finalColor.z));
}

//thread function, casts rays upon the given row
void* castRaysRow(void* threadArg) {
    struct ThreadArgs* args = (struct ThreadArgs*) threadArg;
    for(int x = 0; x < args->cam.image_width; x++) {
        unsigned char* color = args->image + (args->row) * args->cam.image_width * 3 + x * 3;
        Ray ray = castRay(args->cam, x, args->row);
        HitInfo hp = rayIntersect(ray, false);
        calculateColor(hp, color, 0, args->cam);
    }

    delete args;
    return nullptr;
}

int main(int argc, char* argv[]) {
    scene.loadFromXml(argv[1]);
    
    std::vector<SceneObject*>* objs = gather_objects(&scene);
    headKD = buildKDTree(objs, 0, 20, 3);

    
    for (parser::Camera cam : scene.cameras) {
        int W = cam.image_width, H = cam.image_height;
        //this indexing is weird to do D:
        //1D array, columns, rows and R G B
        unsigned char* image = new unsigned char[W * H * 3];

        std::vector<pthread_t> threads(H);

        for(int y = 0; y < H; y++) {
            //concurrencymaxxing (for each row not to overdo)
            ThreadArgs* args = new ThreadArgs{cam, y, image};
            pthread_create(&threads[y], nullptr, castRaysRow, args);
        }
        
        for(pthread_t th: threads) {
            //wait for ducklings
            pthread_join(th, nullptr);
        }
        
        char* name = new char[cam.image_name.length() + 1];
        strcpy(name, cam.image_name.c_str());
        write_ppm(name, image, W, H);
        //DONE!
    }

    return 0;
}