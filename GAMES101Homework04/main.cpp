#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

// 定义控制点向量
std::vector<cv::Point2f> control_points;

// 鼠标事件处理函数，可以捕获鼠标点击的位置
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
    // 如果鼠标左键被点击，且控制点数量小于10，则将点击位置添加到控制点向量中
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() <= 10)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
            << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

// 递归的 Bézier 曲线绘制函数，使用参数 t 计算曲线上的点
cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    // 如果控制点数量为 1，则返回该控制点
   if (control_points.size() == 1) 
   {
       return control_points[0];
   }
   else 
   {
       // 创建一个新的控制点向量
       std::vector<cv::Point2f> new_control_points;
       // 遍历控制点向量，使用 Bézier 曲线公式计算新的控制点
       for (int i = 0; i < control_points.size() - 1; i++) 
       {
           cv::Point2f new_point = (1 - t) * control_points[i] + t * control_points[i + 1];
           new_control_points.push_back(new_point);
       }
       
       // 递归调用函数，计算新的控制点向量上的点
       return recursive_bezier(new_control_points, t);
   }
}
// 绘制 Bézier 曲线的函数，使用递归的 Bézier 曲线绘制函数
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    //加上控制线，以及抗锯齿参数
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        cv::line(window, control_points[i], control_points[i + 1], cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    }
   // 从 t = 0 到 t = 1，每次增加 0.001，计算 Bézier 曲线上的点
    for (double t = 0.0; t <= 1.0; t += 0.001) // 遍历t值
    {
        auto point = recursive_bezier(control_points, t); // 计算贝塞尔曲线上的点
        std::cout << point.x << ", " << point.y << ")" << '\n'; // 输出点的坐标
        for (int i = -1; i <= 1; i++) { // 遍历周围的点
            for (int j = -1; j <= 1; j++) {
                cv::Point2f t;
                t.x = point.x + i; // 计算周围点的x坐标
                t.y = point.y + j; // 计算周围点的y坐标
                float d = sqrt(pow(t.x - ((int)t.x + i) - 0.5, 2) + pow(t.y - ((int)t.y + j) - 0.5, 2)); // 计算像素离点的距离
                float ratio = 1.0 - sqrt(2) / 3.0 * d; // 离得越远着色越淡
                window.at<cv::Vec3b>(t.y, t.x)[1] = std::fmax(255 * ratio, window.at<cv::Vec3b>(t.y, t.x)[1]); // 更新像素值
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);
    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 6) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);
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
