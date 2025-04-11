#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

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


int main()
{
    
}