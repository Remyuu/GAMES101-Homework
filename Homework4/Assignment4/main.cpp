#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#define POINTNUM 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < POINTNUM)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    // if there's only one control point left, return it
    if(control_points.size() == 1)
        return control_points[0];

    std::vector<cv::Point2f> new_points;

    // Compute points for next recursive step, between every pair of control points
    for(size_t i = 0; i < control_points.size()-1; i++){
        float x = (1 - t) * control_points[i].x + t * control_points[i+1].x;
        float y = (1 - t) * control_points[i].y + t * control_points[i+1].y;
        new_points.push_back(cv::Point2f(x, y));
    }

    // recursively continue on new set of points
    return recursive_bezier(new_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    const double step = 0.001;
    for(double t = 0; t <= 1; t += step)
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point) = cv::Vec3b(0, 255, 0);
    }
}
void addColor(cv::Mat &img, cv::Point2f point, cv::Vec3b color) {
//    cv::Point2f center(std::round(point.x), std::round(point.y));
//    cv::Point2f p1(center.x - 1, center.y - 1);
//    cv::Point2f p2(center.x, center.y - 1);
//    cv::Point2f p3(center.x - 1, center.y);
//    cv::Point2f p4(center.x, center.y);


    cv::Point2f p1(point.x, point.y);
    cv::Point2f p2(point.x + 1, point.y);
    cv::Point2f p3(point.x, point.y + 1);
    cv::Point2f p4(point.x + 1, point.y + 1);

    float dx = point.x - p1.x;
    float dy = point.y - p1.y;

    img.at<cv::Vec3b>(p1) += color * ((1-dx) * (1-dy));
    img.at<cv::Vec3b>(p2) += color * (dx * (1-dy));
    img.at<cv::Vec3b>(p3) += color * ((1-dx) * dy);
    img.at<cv::Vec3b>(p4) += color * (dx * dy);
}
void bezier_sa(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    const double step = 0.001f;

    for(double t = 0; t <= 1; t += step)
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        addColor(window, point, cv::Vec3b(0, 255, 0));
    }
}


int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    control_points.emplace_back(10, 10);
    control_points.emplace_back(690, 10);
    control_points.emplace_back(10, 690);
    control_points.emplace_back(690, 690);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == POINTNUM)
        {
//            naive_bezier(control_points, window);
//            bezier(control_points, window);
//            bezier_sa(control_points, window);
            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
