#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //model旋轉矩陣，先初始化為單位矩陣
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    //將函數傳入的rotation_angle轉換為弧度表示。因為cos等函數接收的是弧度。
    auto radian = (float)(rotation_angle/180.0*MY_PI);
    Eigen::Matrix4f translate;
    translate << cos(radian),-sin(radian),0,0,
            sin(radian),cos(radian),0,0,
            0,0,1,0,
            0,0,0,1;
    model = translate * model;
    return model;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1,  eye_pos[2],
                 0, 0, 0, 1;
    view = translate * view;
    return view;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar){
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f M_p2o = Eigen::Matrix4f ::Identity();

    M_p2o <<    zNear,0,0,0,
                0,zNear,0,0,
                0,0,zNear+zFar,-zNear*zFar,
                0,0,1,0;

    Eigen::Matrix4f M_orth = Eigen::Matrix4f ::Identity();
    float fov = eye_fov*MY_PI/180;
    float top = tan(fov) * zNear;
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;
    M_orth <<   2/(right-left),0,0,-(right+left)/(right-left),
                0,2/(top-bottom),0,-(top+bottom)/(top-bottom),
                0,0,2/(zNear-zFar),-(zNear+zFar)/(zNear-zFar),
                0,0,0,1;
    projection = M_orth * M_p2o;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle){
    Eigen::Matrix4f M_rodrigues = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f M_temp_rodrigues = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f M_I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f M_N;
    M_N <<  0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    auto radian = (float)(angle/180.0*MY_PI);
    M_temp_rodrigues << cos(radian)*M_I+(1-cos(radian))*axis*axis.transpose()+sin(radian)*M_N;
    M_rodrigues <<  M_temp_rodrigues(0,0), M_temp_rodrigues(0,1), M_temp_rodrigues(0,2), 0,
            M_temp_rodrigues(1,0), M_temp_rodrigues(1,1), M_temp_rodrigues(1,2), 0,
            M_temp_rodrigues(2,0), M_temp_rodrigues(2,1), M_temp_rodrigues(2,2), 0,
            0, 0, 0, 1;
    return M_rodrigues;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);
    Eigen::Vector3f eye_pos = {0, 0, 5};
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(0,1,0),angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(1);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }
    return 0;
}