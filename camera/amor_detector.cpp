#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>

//camera_matrix: [2065.0580175762857, 0.0, 658.9098266395495, 0.0, 2086.886458338243, 531.5333174739342, 0.0, 0.0, 1.0]
//distortion_coefficients: [-0.051836613762195866, 0.29341513924119095, 0.001501183796729562, 0.0009386915104617738, 0.0]

cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    2065.0580175762857, 0.0, 658.9098266395495, 0.0, 2086.886458338243, 531.5333174739342, 0.0, 0.0, 1.0);
cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << 
    -0.051836613762195866, 0.29341513924119095, 0.001501183796729562, 0.0009386915104617738, 0.0);
std::vector<cv::Point3f> object_points = {
    {0.0, 0.0, 0.0},
    {0.135, 0.0, 0.0},
    {0.135, 0.055, 0.0},
    {0.0, 0.055, 0.0}
};

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
        std::vector<cv::Point2f> points;
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
            this->points = {
                L_Light.point[0],  // 左上角对应object_points[0]
                R_Light.point[1],  // 右上角对应object_points[1] 
                R_Light.point[2],  // 右下角对应object_points[2]
                L_Light.point[3]   // 左下角对应object_points[3]
            };
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
/*
std::vector<Light> find_lights(cv::Mat src,cv::Mat& binary)
{
    std::vector<Light> lights;
    cv::Mat channels[3];
    cv::Mat kernel;
    cv::split(src, channels);
    cv::Mat color_mask;
    cv::subtract(channels[0],channels[2],color_mask);
    cv::imshow("test",color_mask);
    cv::waitKey(0);
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
    
     kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::morphologyEx(light_counter_image, light_counter_image, cv::MORPH_OPEN, kernel);
 
    // 将椭圆核改为矩形核（保持直角特征）
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,1)); // 水平方向7像素，垂直3像素
    
    // 改用矩形特征增强的形态学组合
    cv::morphologyEx(light_counter_image, light_counter_image, cv::MORPH_OPEN, kernel);
        
    // 垂直方向膨胀（加强矩形长边）
    cv::dilate(light_counter_image, light_counter_image, 
              cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,5))); 
    
    // 水平方向轻微腐蚀（保持垂直边缘锐利）
    cv::erode(light_counter_image, light_counter_image, 
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,1)));
    
    cv::dilate(light_counter_image, light_counter_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    cv::bitwise_and(light_counter_image, binary, light_counter_image);
    cv::imshow("LC",light_counter_image);
    cv::waitKey(0);
    std::vector<std::vector<cv::Point>> contours;
     kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(light_counter_image, light_counter_image, kernel, cv::Point(-1, -1), 2);

    // 显示腐蚀后的结果
    cv::imshow("Eroded mask", light_counter_image);
    cv::waitKey(0);  
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
*/
std::vector<Armor> Armor_Establish(std::vector<Light> lights_in)
{
    std::vector<Armor> new_Armors;
    for (const auto& left_light:lights_in)
    {
        for (const auto& right_light:lights_in)
        {
            if(left_light.center.x >= right_light.center.x)
            {
                continue;
            }

            Armor Armor_temp = Armor(left_light,right_light);
            bool light_height_ratio_valid = Armor_temp.light_height_ratio < 1.2 && Armor_temp.light_height_ratio > 0.8;
            bool light_angle_diff_valid = Armor_temp.light_angle_diff < 10;
            bool angle_valid = Armor_temp.angle < 30;
            bool light_center_distance_valid = (Armor_temp.light_center_dis > 0.8 && Armor_temp.light_center_dis < 3.2)
                                                || (Armor_temp.light_center_dis > 3.2 && Armor_temp.light_center_dis < 5.5);
            if(light_height_ratio_valid && light_angle_diff_valid && angle_valid && light_center_distance_valid)
            {
                new_Armors.push_back(Armor_temp);
            }
        }    
    }
    if(new_Armors.empty())
    {
        std::cout << "Armor fetch failed" << std::endl;  
    }
    else
    {
        std::cout << "Armor fectched" << std::endl;
    }
    return new_Armors;
}


 std::vector<Light>  find_lights(const cv::Mat& input,cv::Mat preprocessed_image) {
    std::vector<Light> tmp_lights;
    cv::Mat channels[3];
    cv::Mat kernel;
    cv::Mat color_mask;
    cv::Mat light_contour_binary_image;
    cv::Mat visualization = input.clone();
    // 敌方颜色通道 - 己方颜色通道
    cv::split(input, channels);
    cv::subtract(channels[0], channels[2], color_mask);
    cv::threshold(
        color_mask,
        light_contour_binary_image,
        cv::THRESH_OTSU,
        255,
        cv::THRESH_BINARY
    );
    // 膨胀，使灯条更加连续
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(light_contour_binary_image, light_contour_binary_image, kernel);
    cv::imshow("test",light_contour_binary_image);
    cv::waitKey(0);
    // 与预处理图像取交集，获取又亮又符合颜色的区域
    cv::bitwise_and(preprocessed_image, light_contour_binary_image, light_contour_binary_image);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(light_contour_binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour: contours) {
        if (contour.size() < 4) {
            continue;
        }

        Light light_tmp = Light(cv::minAreaRect(contour));
        light_tmp.valid = (light_tmp.length > light_tmp.width * 3) && (std::abs(light_tmp.angle) < 25);

        if (light_tmp.valid == true) {
            tmp_lights.push_back(light_tmp);
            for(int j=0; j<4; j++) {
                cv::line(visualization, light_tmp.point[j], 
                        light_tmp.point[(j+1)%4], 
                        cv::Scalar(255, 0, 0), 2);  // 蓝色边框
                cv::putText(visualization, 
                        std::to_string(j),  // 显示当前点索引
                        light_tmp.point[j] + cv::Point2f(5,5), // 偏移防止遮挡
                        cv::FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        cv::Scalar(255,255,255), // 白色文本
                        1);
        }
    }
}
    if(!tmp_lights.empty())
    {
        std::cout << "Lights found" << std::endl;
        cv::imshow("Detected Lights", visualization);
        cv::waitKey(0);
    }
    else
        std::cout << "Light_Not_Found" << std::endl;
    return tmp_lights;
}

/* std::vector<Light> find_lights_HSV(cv::Mat input)
{

} */

int main()
{
    std::vector<Light> lights;
    std::vector<Armor> armors;
    cv::Mat src = cv::imread("../data_armor/armors/21.jpg");
    if (src.empty())
    {
        std::cerr << "Error: Could not load image!" << std::endl;
        return -1;
    }
    cv::Mat binary = binarization(src);
    cv::imshow("binary",binary);
    cv::waitKey(0);
    lights = find_lights(src,binary);
    armors = Armor_Establish(lights);
    cv::Mat visualization = src.clone();

    for(auto& armor:armors)
    {
        cv::Mat rvec;
        std::cout << "Armor found" << std::endl;
        std::cout << "L_Light_center" << armor.L_Light.center << std::endl;
        std::cout << "R_Light_center" << armor.R_Light.center << std::endl;
        std::vector<cv::Point2f> image_points = armor.points;
        
        // 绘制装甲板轮廓
        for(int i=0; i<4; i++) {
            cv::line(visualization, 
                    armor.points[i], 
                    armor.points[(i+1)%4], 
                    cv::Scalar(0, 255, 0),  // 绿色线条
                    2);                     // 线宽
        }
        
        // 绘制中心点
        cv::circle(visualization, armor.center, 5, cv::Scalar(0, 0, 255), -1);  // 红色实心圆
        cv::putText(visualization, "Armor", armor.center + cv::Point2f(10, -10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2);
        cv::solvePnP(
            object_points,
            image_points,    
            camera_matrix,
            distortion_coefficients,
            rvec,
            armor.pose.position
        );
        cv::Rodrigues(rvec, armor.pose.rotation);

        std::cout << "Armor position: " << armor.pose.position << std::endl;
        std::cout << "Armor rotation: " << armor.pose.rotation << std::endl;
    }
    cv::imshow("Detected Armor", visualization);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}