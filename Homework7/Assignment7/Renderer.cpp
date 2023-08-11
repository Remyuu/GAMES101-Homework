//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
//#include <omp.h>
#include "mutex"
#include "thread"


inline float deg2rad(const float &deg) { return deg * M_PI / 180.0f; }

const float EPSILON = 0.00001;

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = (float)scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    // int m = 0;

    // change the spp value to change sample ammount
    int spp = 1024;//16
    int thread_num = 2;// Threads Num
    int thread_height = scene.height / thread_num;
    std::vector<std::thread> threads(thread_num);
    std::cout << "SPP: " << spp << "\n";

    //多线程实现
    std::mutex mtx;
    float process=0;
    float Reciprocal_Scene_height=1.f/ (float)scene.height;
    auto castRay = [&](int thread_index)
    {
        float thread_progress = 0.f;
        int height = thread_height * (thread_index + 1);
        std::cout << "Caling:" <<  height - thread_height << "-" << height << "\n";
        for (uint32_t j = height - thread_height; j < height; j++)
        {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                //eye的位置对结果有影响
                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++)
                {
                    framebuffer[j*scene.width+i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            mtx.lock();
            process = process + Reciprocal_Scene_height;
//            UpdateProgress(process);
            thread_progress += Reciprocal_Scene_height;
            std::cout << "Thread[" << thread_index << "] Progress: " << thread_progress*100*thread_num << std::endl;
            mtx.unlock();
        }
    };

    for (int k = 0; k < thread_num; k++)
    {
        threads[k] = std::thread(castRay,k);
        std::cout << "Thread[" << k << "] Started:" << threads[k].get_id() << "\n";
    }
    for (int k = 0; k < thread_num; k++)
    {
        threads[k].join();
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}



// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.

//void Renderer::Render(const Scene &scene) {
//    std::vector<Vector3f> framebuffer(scene.width * scene.height);
//
//    float scale = tan(deg2rad(scene.fov * 0.5));
//    float imageAspectRatio = scene.width / (float) scene.height;
//    Vector3f eye_pos(278, 273, -800);
//
//    // change the spp value to change sample ammount
//    //int spp = 16; origin
//    int spp = 4;
//    std::cout << "SPP: " << spp << "\n";
//    int process = 0;
//
//    int m = 0;
////#pragma omp parallel for
//    for (uint32_t j = 0; j < scene.height; ++j) {
//        for (uint32_t i = 0; i < scene.width; ++i) {
//            // generate primary ray direction
//            float x = (2 * (i + 0.5) / (float) scene.width - 1) *
//                      imageAspectRatio * scale;
//            float y = (1 - 2 * (j + 0.5) / (float) scene.height) * scale;
//
//            Vector3f dir = normalize(Vector3f(-x, y, 1));
//            for (int k = 0; k < spp; k++) {
//                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
//            }
//            m++;
//        }
//        UpdateProgress(j / (float) scene.height);
//    }
//
//
//    UpdateProgress(1.f);
//
//    // save framebuffer to file
//    FILE *fp = fopen("binary_finish.ppm", "wb");
//    (void) fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
//    for (auto i = 0; i < scene.height * scene.width; ++i) {
//        static unsigned char color[3];
//        color[0] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
//        color[1] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
//        color[2] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
//        fwrite(color, 1, 3, fp);
//    }
//    fclose(fp);
//}