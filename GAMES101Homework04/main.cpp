#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

// ������Ƶ�����
std::vector<cv::Point2f> control_points;

// ����¼������������Բ����������λ��
void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
    // �����������������ҿ��Ƶ�����С��10���򽫵��λ����ӵ����Ƶ�������
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() <= 10)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
            << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

// �ݹ�� B��zier ���߻��ƺ�����ʹ�ò��� t ���������ϵĵ�
cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    // ������Ƶ�����Ϊ 1���򷵻ظÿ��Ƶ�
   if (control_points.size() == 1) 
   {
       return control_points[0];
   }
   else 
   {
       // ����һ���µĿ��Ƶ�����
       std::vector<cv::Point2f> new_control_points;
       // �������Ƶ�������ʹ�� B��zier ���߹�ʽ�����µĿ��Ƶ�
       for (int i = 0; i < control_points.size() - 1; i++) 
       {
           cv::Point2f new_point = (1 - t) * control_points[i] + t * control_points[i + 1];
           new_control_points.push_back(new_point);
       }
       
       // �ݹ���ú����������µĿ��Ƶ������ϵĵ�
       return recursive_bezier(new_control_points, t);
   }
}
// ���� B��zier ���ߵĺ�����ʹ�õݹ�� B��zier ���߻��ƺ���
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    //���Ͽ����ߣ��Լ�����ݲ���
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        cv::line(window, control_points[i], control_points[i + 1], cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    }
   // �� t = 0 �� t = 1��ÿ������ 0.001������ B��zier �����ϵĵ�
    for (double t = 0.0; t <= 1.0; t += 0.001) // ����tֵ
    {
        auto point = recursive_bezier(control_points, t); // ���㱴���������ϵĵ�
        std::cout << point.x << ", " << point.y << ")" << '\n'; // ����������
        for (int i = -1; i <= 1; i++) { // ������Χ�ĵ�
            for (int j = -1; j <= 1; j++) {
                cv::Point2f t;
                t.x = point.x + i; // ������Χ���x����
                t.y = point.y + j; // ������Χ���y����
                float d = sqrt(pow(t.x - ((int)t.x + i) - 0.5, 2) + pow(t.y - ((int)t.y + j) - 0.5, 2)); // �����������ľ���
                float ratio = 1.0 - sqrt(2) / 3.0 * d; // ���ԽԶ��ɫԽ��
                window.at<cv::Vec3b>(t.y, t.x)[1] = std::fmax(255 * ratio, window.at<cv::Vec3b>(t.y, t.x)[1]); // ��������ֵ
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
