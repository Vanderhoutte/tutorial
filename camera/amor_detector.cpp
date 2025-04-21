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
            //std::sort(point, point + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
            // 新排序策略：先找左上角点，然后顺时针排序
            cv::Point2f centers = Light_box.center;
            std::sort(point, point+4, [centers](cv::Point2f& a, cv::Point2f& b) {
            return (a.x - centers.x) * (b.y - centers.y) - 
                   (a.y - centers.y) * (b.x - centers.x) > 0;
        });
    
        // 验证并确保第一个点是左上角
        if (point[0].x > point[1].x) std::swap(point[0], point[1]);
        if (point[3].x > point[2].x) std::swap(point[2], point[3]);

    // 保持原有计算逻辑
            top = (point[0] + point[3]) / 2;
            bottom = (point[1] + point[2]) / 2;
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

cv::Mat binarization(cv::Mat& src)
{
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
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
/*
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
*/

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
    cv::imshow("mask",light_counter_image);
    cv::waitKey(0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11,11));
    cv::morphologyEx(light_counter_image, light_counter_image, cv::MORPH_OPEN, kernel);
        // 将椭圆核改为矩形核（保持直角特征）
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,3)); // 水平方向7像素，垂直3像素
    
        // 改用矩形特征增强的形态学组合
        cv::morphologyEx(light_counter_image, light_counter_image, cv::MORPH_OPEN, kernel);
        
        // 垂直方向膨胀（加强矩形长边）
        cv::dilate(light_counter_image, light_counter_image, 
                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,5))); 
    
        // 水平方向轻微腐蚀（保持垂直边缘锐利）
        cv::erode(light_counter_image, light_counter_image, 
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,1)));
    //cv::dilate(light_counter_image, light_counter_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    cv::bitwise_and(light_counter_image, binary, light_counter_image);
    cv::imshow("LC",light_counter_image);
    cv::waitKey(0);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(light_counter_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    Light light_temp;
    cv::Mat visualization = src.clone();
    for (auto& contour : contours)
    {
        if(contour.size() < 4) continue;
        light_temp = Light(cv::minAreaRect(contour));
        light_temp.valid = (light_temp.length > light_temp.width * 3);
        if (light_temp.valid)
        {
            lights.push_back(light_temp);
            //cv::line(visualization, light_temp.top, light_temp.bottom, cv::Scalar(0, 255, 0), 2);
             for(int j=0; j<4; j++) {
            cv::line(visualization, light_temp.point[j], 
                    light_temp.point[(j+1)%4], 
                    cv::Scalar(255, 0, 0), 2);  // 蓝色边框
            cv::putText(visualization, 
                    std::to_string(j),  // 显示当前点索引
                    light_temp.point[j] + cv::Point2f(5,5), // 偏移防止遮挡
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    cv::Scalar(255,255,255), // 白色文本
                    1);
        }
        std::cout << "center" << light_temp.center << std::endl;
        cv::circle(visualization, light_temp.center, 4, cv::Scalar(0, 255, 255), -1); 
            cv::putText(visualization,
                        "center",
                        light_temp.center + cv::Point2f(5,5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255,255,255),
                        2);
        }
    }
    if(!lights.empty())
    {
        std::cout << "Lights found" << std::endl;
        cv::imshow("Detected Lights", visualization);
        cv::waitKey(0);
    }
    else
        std::cout << "Light_Not_Found" << std::endl;
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
    cv::Mat binary = binarization(src);
    cv::imshow("binary",binary);
    cv::waitKey(0);
    lights = find_lights(src,binary);
    cv::destroyAllWindows();
    return 0;
}