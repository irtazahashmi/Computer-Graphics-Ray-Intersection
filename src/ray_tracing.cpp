#include "ray_tracing.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

bool intersectRayWithScene(const Scene& scene, 
                           Ray& ray)
{
    bool hit = false;
    for (const auto& mesh : scene.meshes)
        hit |= intersectRayWithShape(mesh, ray);
    for (const auto& sphere : scene.spheres)
        hit |= intersectRayWithShape(sphere, ray);
    for (const auto& box : scene.boxes)
        hit |= intersectRayWithShape(box, ray);
    return hit;
}


// This method receives the three vertices of the triangle, the triangle normal, and the point to 
// be verified p. It returns true if the point is inside the triangle, false otherwise. 
// Note that a point on the triangle edge is considered as inside the triangle.
bool pointInTriangle(const glm::vec3& v0, 
                     const glm::vec3& v1, 
                     const glm::vec3& v2, 
                     const glm::vec3& n, 
                     const glm::vec3& p)
{

    // The three edges of the triangle
    glm::vec3 triangleEdgeOne = v1 - v0;
    glm::vec3 traingleEdgeTwo = v2 - v1;
    glm::vec3 triangleEdgeThree = v0 - v2;

    // The point relative to triangle edges
    glm::vec3 trianglePointOne = p - v0;
    glm::vec3 trianglePointTwo = p - v1;
    glm::vec3 trianglePointThree = p - v2;

    // checking if the point p is inside all three edges of the triangle
    float insideEdgeOne = glm::dot(n, glm::cross(triangleEdgeOne, trianglePointOne));
    float insideEdgeTwo = glm::dot(n, glm::cross(traingleEdgeTwo, trianglePointTwo));
    float insideEdgeThree = glm::dot(n, glm::cross(triangleEdgeThree, trianglePointThree));
    
    // point p inside the trianlge conditions
    if (insideEdgeOne >= 0 && insideEdgeTwo >= 0 && insideEdgeThree >= 0) {
        // point p inside triangle
        return true;
    }

    // point p outside the triangle
    return false;
}


// This method intersectRayWithPlane receives the plane and the ray, 
// returns false if the ray does not intersect the plane, or true otherwise. 
// In the case there is an intersection it should also update the ray.t value.
// If there is an intersection, the next step is to verify if the intersection point is inside the triangle.
bool intersectRayWithPlane(const Plane& plane, 
                           Ray& ray)
{
    // 1) no intersection is found or t <= 0: return false
    // 2) found intersection point p
    //    a) if p is closer to the ray origin then current intersection point : return true and update ray.t
    //    b) else return false
       
    // Getting all the variables to calculate the intersection point t
    // t = (D - dot(o, n)) / dot(d, n)
    // plane vars
    float D = plane.D;
    glm::vec3 n = plane.normal;
    //ray vars
    glm::vec3 o = ray.origin;
    glm::vec3 d = ray.direction;

    // calculate t components
    float numerator = D - glm::dot(o, n);
    float denominator = glm::dot(d, n);

    // if abs(denominator) < epsilon, ray is parrallel is to plane -> no intersection
    float epsilon = 1E-6;
    if (glm::abs(denominator) < epsilon) {
        return false;
    }

    // calculate intersection
    float t = numerator / denominator;

    // if intersection
    if (t > 0) {
        // if the intersection is closer than a previous intersection, update t and return true
        if (t < ray.t) {
            ray.t = t;
            return true;
        }
    }
    
    // else no intersection, return false
    return false;
}


// The trianglePlanereceives the three vertices of a triangle and defines the plane that 
// contains the triangle, that is, you need to set the plane.normal and plane.D values.
Plane trianglePlane(const glm::vec3& v0, 
                    const glm::vec3& v1, 
                    const glm::vec3& v2)
{
    Plane plane;

    // the cross product of difference of the triangle vertices is the normal of the plane:
    // normalize ( cross((v0 - v2), (v1 - v2)) ) = normal of plane
    glm::vec3 edgeAC = v0 - v2;
    glm::vec3 edgeBC = v1 - v2;
    glm::vec3 crossProduct = glm::cross(edgeAC, edgeBC);
    glm::vec3 normalOfPlane = glm::normalize(crossProduct);
    plane.normal = normalOfPlane;

    // D is equal to dot(p, n)
    glm::vec3 p = v0;
    glm::vec3 n = normalOfPlane;
    float D = glm::dot(p, n);
    plane.D = D;

    // return plane
    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, 
                              const glm::vec3& v1, 
                              const glm::vec3& v2, 
                              Ray& ray)
{
    // TIP: Don’t forget to roll back the ray.t value modified in the intersectRayWithPlane method if 
    // the point is not inside the triangle

    // First, compute the plane containing the triangle.
    Plane planeTriangle = trianglePlane(v0, v1, v2);
    
    // Second, compute the intersection point of the rayand the plane.
    float previousT = ray.t;
    bool intersectPlaneSuccess = intersectRayWithPlane(planeTriangle, ray);
    bool intersectTriangleSuccess = false;

    // If there is an (plane) intersection, the third step is to check if the point is inside the triangle.
    if (intersectPlaneSuccess) {
        glm::vec3 n = planeTriangle.normal;
        glm::vec3 p = ray.origin + ray.direction * ray.t;
        // check if point inside the triangle
        intersectTriangleSuccess = pointInTriangle(v0, v1, v2, n, p);

        // the point is in the triangle - intersection ray triangle - success
        if (intersectTriangleSuccess) {
            // storing t for the the nearest triangle intersection
            if (previousT < ray.t) {
                ray.t = previousT;
            }
            return true;
        }
    }

    // no intersection/the point is not inside the triangle -> roll back 
    ray.t = previousT;
    // intersection of ray triangle - fail
    return false;
}

bool intersectRayWithShape(const Mesh& mesh, 
                           Ray& ray)
{
    bool hit = false;
    for (const auto& tri : mesh.triangles) {
        const auto v0 = mesh.vertices[tri[0]];
        const auto v1 = mesh.vertices[tri[1]];
        const auto v2 = mesh.vertices[tri[2]];
        hit |= intersectRayWithTriangle(v0.position, v1.position, v2.position, ray);
    }
    return hit;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, 
                           Ray& ray)
{
    // new origin is now dependent on the center of the sphere
    glm::vec3 newOrigin = ray.origin - sphere.center;

    // calculate a
    float xDirection = (ray.direction.x * ray.direction.x);
    float yDirection = (ray.direction.y * ray.direction.y);
    float zDirection = (ray.direction.z * ray.direction.z);
    float a = xDirection + yDirection + zDirection;

    // calculate b
    float xComponent = ray.direction.x * newOrigin.x;
    float yComponent = ray.direction.y * newOrigin.y;
    float zComponent = ray.direction.z * newOrigin.z;
    float b = 2.0f * (xComponent + yComponent + zComponent);

    // calculate c
    float xOrigin = (newOrigin.x * newOrigin.x);
    float yOrigin = (newOrigin.y * newOrigin.y);
    float zOrigin = (newOrigin.z * newOrigin.z);
    float rSquared = (sphere.radius * sphere.radius);
    float c = xOrigin + yOrigin + zOrigin - rSquared;


    // discriminant test: if its < 0 -> no solution
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0) {
        return false;
    }
    
    // retrieving solutions using the quadaratic equation
    float numeratorOne = -b + glm::sqrt(discriminant);
    float numeratorTwo = -b - glm::sqrt (discriminant);
    float denominaotor = 2.0f * a;
    float solutionOne = numeratorOne / denominaotor;
    float solutionTwo = numeratorTwo / denominaotor;

    // if we have 2 solutions
    if (solutionOne > 0 && solutionTwo > 0) {
        // find the closest intersection point
        float closestT = glm::min(solutionOne, solutionTwo);
        if (closestT < ray.t) {
            // update closest t
            ray.t = closestT;
        }
        // intersection sucess
        return true;
    }

    // if we have one solution: case 1
    if (solutionOne > 0) {
        // find closest t
        if (solutionOne < ray.t) {
            // update closest t
            ray.t = solutionOne;
        }
        return true;
    }


    // if we have one solution: case 2
    if (solutionTwo > 0) {
        // find closest t
        if (solutionTwo < ray.t) {
            // update closest t
            ray.t = solutionTwo;
        }
        return true;
    }

    // no intersection occurred
    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: 
/// minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, 
                           Ray& ray)
{
    
    // using txmin = (xmin - ox) / dx as an example to find all six sides of the box - lecture slides
    // X components
    float tXMin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tXMax = (box.upper.x - ray.origin.x) / ray.direction.x;
    // Y componeneets
    float tYMin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tYMax = (box.upper.y - ray.origin.y) / ray.direction.y;
    // Z component
    float tZMin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float tZMax = (box.upper.z - ray.origin.z) / ray.direction.z;


    // tXIn = min(txmin, txmax) ---- tXOut = max(txmin,txmax) --- lecture slides
    // X componenets
    float tXIn = std::min(tXMin, tXMax);
    float tXOut = std::max(tXMin, tXMax);
    // Y 
    float tYIn = std::min(tYMin, tYMax);
    float tYOut = std::max(tYMin, tYMax);
    // Z
    float tZIn = std::min(tZMin, tZMax);
    float tZOut = std::max(tZMin, tZMax);

    // finding global in and out -- lecture slides
    float tIn = std::max(tXIn, std::max(tYIn, tZIn));
    float tOut = std::min(tXOut, std::min(tYOut, tZOut));

    // ray misses -- no intersection conditions
    if (tIn > tOut || tOut < 0) {
        return false;
    } 

 
     // there is an intersection of ray and AABB
    if (tIn < ray.t) {
        // if the intersection is behind the origin of the ray, then update ray.t as tOut
        if (tIn < 0) {
            ray.t = tOut;
        }
        else {
            // else the intersection is infront. update ray.t as tIn (where the ray hits the box) and return true
            ray.t = tIn;
        }
        return true;
    }

    // no intersection
    return false;
}
