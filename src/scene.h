#pragma once
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/vec3.hpp>
DISABLE_WARNINGS_POP()
#include <filesystem>
#include <framework/mesh.h>
#include <framework/ray.h>
#include <optional>
#include <vector>

enum SceneType {
    SingleTriangle,
    Cube,
    Dragon,
    AABBs,
    Spheres,
    Mixed,
    Custom
};

struct Plane {
    float D = 0.0f;
    glm::vec3 normal { 0.0f, 1.0f, 0.0f };
};

struct AxisAlignedBox {
    glm::vec3 lower { 0.0f };
    glm::vec3 upper { 1.0f };

    Material material;
};

struct Sphere {
    glm::vec3 center { 0.0f };
    float radius = 1.0f;

    Material material;
};

struct Scene {
    std::vector<Mesh> meshes;
    std::vector<Sphere> spheres;
    std::vector<AxisAlignedBox> boxes;
};

// Load a prebuilt scene.
Scene loadScene(SceneType type, const std::filesystem::path& dataDir, std::optional<Ray>& optRay);