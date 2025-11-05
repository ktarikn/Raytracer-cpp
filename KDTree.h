#include "parser.h"
#include <algorithm>

//vscode detecs 'errors' but works ?? >cannot find Vec3f etc for some reason

enum ObjectType {
    SPHERE,
    TRIANGLE,
    CYLINDER,
};


struct AABB {
    Vec3f min; // minimum corner (x_min, y_min, z_min)
    Vec3f max; // maximum corner (x_max, y_max, z_max)
};

struct SceneObject {
    ObjectType type;
    void* data;  // pointer to actual parser:: object
    AABB bounds;

    AABB getBounds(){ return bounds;};
};

struct KDNode {
    AABB bounds;
    int axis;         // 0=x, 1=y, 2=z
    float split;      // coordinate of splitting plane
    KDNode* left;
    KDNode* right;
    std::vector<SceneObject*> objects; // only in leaf nodes

    KDNode() : left(nullptr), right(nullptr), axis(0), split(0) {}
};


AABB computeCylinderAABB(const parser::Cylinder& cyl, const std::vector<Vec3f>& vertices) {
    AABB box;
    Vec3f C = vertices[cyl.center_vertex_id - 1]; // center point
    Vec3f A = cyl.axis;
    float r = cyl.radius;
    float h = cyl.height;

    // normalize axis
    float len = sqrt(A.x*A.x + A.y*A.y + A.z*A.z);
    Vec3f An = { A.x/len, A.y/len, A.z/len };

    // compute extents along each axis
    auto extent = [&](float n){
        return fabs(n) * (h * 0.5f) + sqrtf(1 - n*n) * r;
    };

    float ex = extent(An.x);
    float ey = extent(An.y);
    float ez = extent(An.z);

    box.min.x = C.x - ex;
    box.max.x = C.x + ex;

    box.min.y = C.y - ey;
    box.max.y = C.y + ey;

    box.min.z = C.z - ez;
    box.max.z = C.z + ez;

    return box;
}


AABB computeSphereAABB(const parser::Sphere& s, const std::vector<Vec3f>& vertices) {
    Vec3f c = vertices[s.center_vertex_id - 1]; // 1-based index in XML
    float r = s.radius;
    return {
        {c.x - r, c.y - r, c.z - r},
        {c.x + r, c.y + r, c.z + r}
    };
}

//box intersection?
bool intersectAABB(const Ray& ray, const AABB& box, float& tmin, float& tmax) {
    tmin = 0.0f;
    tmax = 1e30f;
    
    for (int i = 0; i < 3; i++) {
        float rayOrigin, rayDir, boxMin, boxMax;
        
        switch(i) {
            case 0:
                rayOrigin = ray.o.x;
                rayDir = ray.d.x;
                boxMin = box.min.x;
                boxMax = box.max.x;
                break;
            case 1:
                rayOrigin = ray.o.y;
                rayDir = ray.d.y;
                boxMin = box.min.y;
                boxMax = box.max.y;
                break;
            case 2:
                rayOrigin = ray.o.z;
                rayDir = ray.d.z;
                boxMin = box.min.z;
                boxMax = box.max.z;
                break;
        }
        
        if (fabs(rayDir) < 1e-8f) {
            // Ray is parallel to boxside
            if (rayOrigin < boxMin || rayOrigin > boxMax) {
                return false;  // aand out of bounds, miss
            }
        } else {
            // Compute intersection t values
            float t1 = (boxMin - rayOrigin) / rayDir;
            float t2 = (boxMax - rayOrigin) / rayDir;
            
            // Make sure t1 is the near intersection, t2 is far
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            
            // Update tmin and tmax
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            
            
            if (tmin > tmax) {
                return false;
            }
        }
    }
    
    // Ray intersects box if tmax >= 0
    return tmax >= 0;
}

//t of box intersection
float interSectT(Ray ray, AABB box) {
    float tmin, tmax;
    if (intersectAABB(ray, box, tmin, tmax)) {
        return tmin >= 0 ? tmin : -1.0f;
    }
    return -1.0f;
}   

//annoying axis changes, needed this helper to get ray.x as ray[0] for example
float getComponent(Vec3f vec, int a){
    switch (a)
    {
    case 0:
      
        return vec.x;
        break;
    case 1:
     
        return vec.y;
        break;
    case 2:

        return vec.z;
        break;
    
    default:
        return -1;
        break;
    }
}

AABB computeTriangleAABB(const parser::Triangle* t, const std::vector<Vec3f>& vertices) {
    auto v0 = vertices[t->indices.v0_id - 1];
    auto v1 = vertices[t->indices.v1_id - 1];
    auto v2 = vertices[t->indices.v2_id - 1];
    AABB b;
    // Furthest points in each directin
    b.min = { std::min({v0.x, v1.x, v2.x}),
              std::min({v0.y, v1.y, v2.y}),
              std::min({v0.z, v1.z, v2.z}) };
    b.max = { std::max({v0.x, v1.x, v2.x}),
              std::max({v0.y, v1.y, v2.y}),
              std::max({v0.z, v1.z, v2.z}) };
    return b;
}

bool IsKDLeaf(KDNode* node){
    return node->left == nullptr && node->right == nullptr;
}

//could not just go to one leaf
void collectLeafNodes(Ray ray, KDNode* node, std::vector<KDNode*>& leafNodes) {
    if (!node) return;
    
    // Check if ray intersects this node's bounding box
    float tmin, tmax;
    if (!intersectAABB(ray, node->bounds, tmin, tmax)) {
        return; // Ray doesn't hit this node
    }
    
    // If leaf, add to collection
    if (IsKDLeaf(node)) {
        leafNodes.push_back(node);
        return;
    }
    
    // Recurse into both children
    collectLeafNodes(ray, node->left, leafNodes);
    collectLeafNodes(ray, node->right, leafNodes);
}



// Gather objects to a vector,ALL OBJECTS
std::vector<SceneObject*>* gather_objects(parser::Scene* scene) { 
    std::vector<SceneObject*>* objects = new std::vector<SceneObject*>();

        for (auto& s : scene->spheres) {
            SceneObject* obj = new SceneObject();
            obj->type = SPHERE;
            obj->data = &s;
            obj->bounds = computeSphereAABB(s, scene->vertex_data);
            objects->push_back(obj);
        }

        for (auto& t : scene->triangles) {
            SceneObject* obj = new SceneObject();
            obj->type = TRIANGLE;
            obj->data = &t;
            obj->bounds = computeTriangleAABB(&t, scene->vertex_data);
            objects->push_back(obj);
        }

        for (auto& c : scene->cylinders) {
            SceneObject* obj = new SceneObject();
            obj->type = CYLINDER;
            obj->data = &c;
            obj->bounds = computeCylinderAABB( 
                c,
                scene->vertex_data
            );
            objects->push_back(obj);
        }
        for (auto& m : scene->meshes) {
            for(auto& f : m.faces){
                SceneObject* obj = new SceneObject();
                obj->type = TRIANGLE;
                parser::Triangle* tri = new parser::Triangle();
                tri->indices = f;
                tri->material_id = m.material_id;
                obj->data = tri;
                obj->bounds = computeTriangleAABB( // rough approximation
                tri,
                scene->vertex_data
            );
            objects->push_back(obj);
            }
            
            
        }

        return objects;

}



AABB computeBoundingBox(const std::vector<SceneObject*>& objs) {
    AABB box;
    box.min = {1e9, 1e9, 1e9};
    box.max = {-1e9, -1e9, -1e9};

    for (auto obj : objs) {
        AABB b = obj->getBounds();
        box.min.x = std::min(box.min.x, b.min.x);
        box.min.y = std::min(box.min.y, b.min.y);
        box.min.z = std::min(box.min.z, b.min.z);
        box.max.x = std::max(box.max.x, b.max.x);
        box.max.y = std::max(box.max.y, b.max.y);
        box.max.z = std::max(box.max.z, b.max.z);
    }
    return box;
}


KDNode* buildKDTree(std::vector<SceneObject*>* objs, int depth, int maxDepth, int minObjects) {
    KDNode* node = new KDNode();

    // Compute the bounding box of ALL OBJECTS
    node->bounds = computeBoundingBox(*objs);

    // leaf
    if (depth >= maxDepth || objs->size() <= minObjects) {
        node->objects = *objs;
        return node;
    }

    // iterate splitting axis
    int axis = depth % 3;
    node->axis = axis;

    // Compute median split value of object centers
    std::vector<float> centers;
    centers.reserve(objs->size());
    for (auto obj : *objs) {
        AABB b = obj->getBounds();
        float center = (getComponent(b.min,axis) + getComponent(b.max,axis)) * 0.5f;
        centers.push_back(center);
    }
    std::nth_element(centers.begin(), centers.begin() + centers.size()/2, centers.end());
    float splitValue = centers[centers.size()/2];
    node->split = splitValue;

    // Partition objects into left and right
    std::vector<SceneObject*>* leftObjs = new std::vector<SceneObject*>();
    std::vector<SceneObject*>* rightObjs = new std::vector<SceneObject*>();
    // In children they go
    for (auto obj : *objs) {
        AABB b = obj->getBounds();
        if (getComponent(b.min,axis) <= splitValue)
            leftObjs->push_back(obj);
        if (getComponent(b.max,axis) >= splitValue)
            rightObjs->push_back(obj);
    }

    // make sure child != parent, would go forever
    if (leftObjs->size() == objs->size() || rightObjs->size() == objs->size()) {
        node->objects = *objs;
        return node;
    }

    // recurse
    node->left = buildKDTree(leftObjs, depth + 1, maxDepth, minObjects);
    node->right = buildKDTree(rightObjs, depth + 1, maxDepth, minObjects);
    return node;
}
