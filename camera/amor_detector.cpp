#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class Light
{
    public:
        cv::Point2f point[4];
        cv::Point2f top,bottom;
        bool valid = false;
        float ratio;
        float length;
        float width;
        float angle;
        cv::Point2f center;
        std::string classification_result;  
        Light() = default;
        explicit Light(const cv::RotatedRect& Light_box)
        {
            Light_box.points(point);
            std::sort(point, point + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            top = (point[0] + point[1]) / 2;
            bottom = (point[2] + point[3]) / 2;
            length = cv::norm(top - bottom);
            width = cv::norm(point[0] - point[3]);
            center.x = (point[0].x + point[1].x + point[2].x + point[3].x) / 4;
            center.y = (point[0].y + point[1].y + point[2].y + point[3].y) / 4;
            angle = std::atan((top.x - bottom.x) / (top.y - bottom.y)) / CV_PI * 180;
        }
};


struct armorPose{
            cv::Mat position;
            cv::Mat rotation;
};

class Armor
{
    public:
        Light L_Light,R_Light;
        cv::Point2f center;
        float angle;
        float light_height_ratio;
        float light_angle_diff;
        float light_center_dis;
        float dis_to_center;
        std::string cla_result;
        armorPose pose;
        Armor() = default;
        explicit Armor(const Light& L_Light,const Light& R_Light){
            this->L_Light = L_Light;
            this->R_Light = R_Light;
            this->center = (L_Light.center + R_Light.center) / 2 ;
            light_height_ratio = L_Light.length / R_Light.length;
            light_angle_diff = std::abs(std::abs(L_Light.angle) - std::abs(R_Light.angle));
            cv::Point2f&& diff = L_Light.center - R_Light.center;
            angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
            float&& average_light_length = (L_Light.length + R_Light.length) / 2;
            light_center_dis = cv::norm(L_Light.center - R_Light.center) / average_light_length;    
        }
};

/*cv::Mat binarization(cv::Mat& src)
{
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::Mat blur_gauss;
    cv::GaussianBlur(
        gray,
        blur_gauss,
        cv::Size(9, 9),
        0
    );
    cv::Mat blur_bliater;
    cv::bilateralFilter(
        gray,
        blur_bliater,
        3,
        3,
        3
    );
    cv::Mat binary;
    cv::threshold(
        gray,
        binary,
        cv::THRESH_OTSU,
        255,
        cv::THRESH_BINARY
    );
    return binary;
}
*/

cv::Mat binarization_optimized(cv::Mat& src)
{
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_canny;
    cv::Canny(gray, gray_canny,100,300);
    cv::Mat blur_gauss;
    cv::GaussianBlur(
        gray_canny,
        blur_gauss,
        cv::Size(9, 9),
        0
    );
    cv::Mat blur_bliater;
    cv::bilateralFilter(
        blur_gauss,
        blur_bliater,
        3,
        3,
        3
    );
    cv::Mat binary;
    cv::threshold(
        blur_bliater,
        binary,
        cv::THRESH_OTSU,
        255,
        cv::THRESH_BINARY
    );
    return binary;
}

std::vector<Light> find_lights(cv::Mat src,cv::Mat& binary)
{
    std::vector<Light> lights;
    cv::Mat channels[3];
    cv::split(src, channels);
    cv::Mat color_mask;
    cv::subtract(channels[0],channels[2],color_mask);
    cv::Mat light_counter_image;
    cv::threshold(
        color_mask,
        light_counter_image,
        cv::THRESH_OTSU,
        255,
        cv::THRESH_BINARY
    );
    cv::dilate(light_counter_image, light_counter_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::bitwise_and(light_counter_image, binary, light_counter_image);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(light_counter_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    Light light_temp;
    for (auto& contour : contours)
    {
        if(contour.size() < 4) continue;
        light_temp = Light(cv::minAreaRect(contour));
        light_temp.valid = (light_temp.length > light_temp.width * 3);
        lights.push_back(light_temp);
    }

    return lights;
}

int main()
{
    std::vector<Light> lights;
    std::vector<Armor> armors;
    cv::Mat src = cv::imread("./38.jpg");
    if (src.empty())
    {
        std::cerr << "Error: Could not load image!" << std::endl;
        return -1;
    }
    cv::Mat binary = binarization_optimized(src);
    cv::imshow("binary",binary);
    cv::waitKey(0);
    lights = find_lights(src,binary);
    return 0;
}