#pragma once
#include "scene.h"
#include <framework/mesh.h>
#include <framework/ray.h>

void drawShape(const Mesh& mesh);
void drawShape(const Sphere& sphere);
void drawShape(const AxisAlignedBox& box);

void drawScene(const Scene& scene);

void drawRay(const Ray& ray);
