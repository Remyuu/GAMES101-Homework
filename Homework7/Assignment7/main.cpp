#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char **argv) {
    // Change the definition here to change resolution
    Scene scene(784, 784);
//    Scene scene(512, 512);


    Material *red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material *green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material *white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material *light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
                                             15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
                                             18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    light->Kd = Vector3f(0.65f);

    // ---

    Material *red_m = new Material(MICROFACET_DIFFUSE, Vector3f(0.0f));
    red_m->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material *green_m = new Material(MICROFACET_DIFFUSE, Vector3f(0.0f));
    green_m->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material *white_m = new Material(MICROFACET_DIFFUSE, Vector3f(0.0f));
    white_m->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    white_m->Ks = Vector3f(1,1,1) - white_m->Kd;


    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    MeshTriangle shortbox_m("../models/cornellbox/shortbox.obj", white_m);
    MeshTriangle bunny("../models/bunny/bunny.obj", white_m);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);
    MeshTriangle tallbox_m("../models/cornellbox/tallbox.obj", white_m);
    MeshTriangle sphere("../models/base_object/sphere_translated_ground.obj", white_m);

    MeshTriangle left("../models/cornellbox/left.obj", red);
    MeshTriangle right("../models/cornellbox/right.obj", green);
    MeshTriangle light_("../models/cornellbox/light.obj", light);

    scene.Add(&floor);
//    scene.Add(&shortbox);
//    scene.Add(&shortbox_m);
//    scene.Add(&bunny);
//    scene.Add(&tallbox);
    scene.Add(&tallbox_m);
    scene.Add(&sphere);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count()
              << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count()
              << " seconds\n";

    return 0;
}