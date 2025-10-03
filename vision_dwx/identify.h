#pragma once
#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <vector>
#include <string>

namespace rmcs_core::detect{

    using namespace cv;
    // 定义颜色范围结构体
    struct ColorRange {
        cv::Scalar lower;    // HSV下限
        cv::Scalar upper;    // HSV上限
        std::string colorName;
    };

    // 灯条结构体
    struct LightBar {
        cv::RotatedRect rect;
        float aspectRatio;
        std::string color;
        cv::Point2f center;
    };

    class Armor_Identifier : public rclcpp::Node {

        public:
            Armor_Identifier()
            : Node("armor_identifier", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
            , logger_(get_logger()) {
                // 默认颜色范围
                hsvColorRanges = {
                    {Scalar(0, 80, 50), Scalar(15, 255, 255), "Red"},
                    {Scalar(160, 80, 50), Scalar(180, 255, 255), "Red"},
                    {Scalar(90, 80, 40), Scalar(140, 255, 255), "Blue"},
                    {Scalar(0, 0, 100), Scalar(180, 50, 255), "White"}
                };
                init();
            }

            void init() {
                // 可以从参数服务器动态加载HSV阈值
                // 例如：get_parameter("red_h_lower").as_int();
            }

            // 设置颜色范围的公有方法
            void setColorRanges(const std::vector<ColorRange>& color_ranges) {
                hsvColorRanges = color_ranges;
            }

            void update(const cv::Mat& image) {
                cv::Mat img_first = image.clone();
                cv::Mat output_image = image.clone();
                std::vector<LightBar> light_bars; 

                detect_light_bars(img_first, light_bars, output_image);
                
                //cv::Mat display;
                //cv::cvtColor(img_first, display, cv::COLOR_RGB2HSV);
                // 显示结果
                cv::imshow("Detection Results", output_image);
                //cv::imshow("Detection Results", display);
                cv::waitKey(1);
            }

        private:
            rclcpp::Logger logger_;
            std::vector<ColorRange> hsvColorRanges; // 成员变量

            // 改进的灯条检测函数
            void detect_light_bars(const cv::Mat& input_img, std::vector<LightBar>& light_bars, cv::Mat& output_img) {
                light_bars.clear();
                
                // 1. 转换为HSV颜色空间
                cv::Mat hsv_img;
                cv::cvtColor(input_img, hsv_img, COLOR_BGR2HSV);
                
                // 2. 分别检测红色和蓝色
                std::vector<cv::Mat> color_masks;
                
                // 红色范围1 (0-15)
                cv::Mat red_mask1, red_mask2, red_mask;
                cv::inRange(hsv_img, hsvColorRanges[0].lower, hsvColorRanges[0].upper, red_mask1);
                cv::inRange(hsv_img, hsvColorRanges[1].lower, hsvColorRanges[1].upper, red_mask2);
                red_mask = red_mask1 | red_mask2;
                
                // 蓝色范围
                cv::Mat blue_mask;
                cv::inRange(hsv_img, hsvColorRanges[2].lower, hsvColorRanges[2].upper, blue_mask);
                
                // 3. 形态学操作去除噪声
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
                cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
                
                // 4. 查找轮廓
                std::vector<std::vector<cv::Point>> red_contours, blue_contours;
                cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(blue_mask, blue_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
                // 5. 处理红色灯条
                for (const auto& contour : red_contours) {
                    process_contour(contour, input_img, light_bars, "Red", output_img);
                }
                
                // 6. 处理蓝色灯条
                for (const auto& contour : blue_contours) {
                    process_contour(contour, input_img, light_bars, "Blue", output_img);
                }
                
                // 7. 检测装甲板
                detect_armor(output_img, light_bars);
            }

            void process_contour(const std::vector<cv::Point>& contour, const cv::Mat& img, 
                               std::vector<LightBar>& light_bars, const std::string& color, cv::Mat& output_img) {
                // 过滤小面积轮廓
                double area = cv::contourArea(contour);
                if (area < 40 || area > 8000) return;//灯条范围*
                
                // 获取最小外接矩形
                cv::RotatedRect rect = cv::minAreaRect(contour);
                
                // 计算宽高比（确保长边为高度）
                float width = rect.size.width;
                float height = rect.size.height;
                float aspect_ratio = (width > height) ? width / height : height / width;
                
                // 灯条特征：长宽比较大（通常2:1以上）
                if (aspect_ratio < 1.5) return;
                
                // 面积与矩形面积比（过滤不规则形状）
                double rect_area = width * height;
                double area_ratio = area / rect_area;
                if (area_ratio < 0.6) return;
                
                // 创建灯条对象
                LightBar light_bar;
                light_bar.rect = rect;
                light_bar.aspectRatio = aspect_ratio;
                light_bar.color = color;
                light_bar.center = rect.center;
                
                light_bars.push_back(light_bar);
                
                // 在图像上绘制灯条
                draw_light_bar(output_img, light_bar);
            }

            void draw_light_bar(cv::Mat& img, const LightBar& light_bar) {
                cv::Point2f vertices[4];
                light_bar.rect.points(vertices);
                
                // 绘制灯条轮廓
                cv::Scalar color = (light_bar.color == "Red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
                
                for (int j = 0; j < 4; j++) {
                    cv::line(img, vertices[j], vertices[(j + 1) % 4], color, 2);
                }
                
                // 绘制中心点
                cv::circle(img, light_bar.center, 3, color, -1);
                
                // 显示颜色标签
                cv::putText(img, light_bar.color, light_bar.center + cv::Point2f(10, -10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }

            void detect_armor(cv::Mat& img, const std::vector<LightBar>& light_bars) {
                if (light_bars.size() < 2) return;
                
                for (size_t i = 0; i < light_bars.size(); i++) {
                    for (size_t j = i + 1; j < light_bars.size(); j++) {
                        // 只匹配相同颜色的灯条
                        if (light_bars[i].color != light_bars[j].color) continue;
                        
                        const LightBar& bar1 = light_bars[i];
                        const LightBar& bar2 = light_bars[j];
                        
                        // 计算两个灯条的距离
                        //float distance = cv::norm(bar1.center - bar2.center);
                        
                        // 距离筛选（根据实际情况调整）
                        //if (distance < 30 || distance > 200) continue;
                        
                        // 角度筛选（灯条应该大致平行）
                        float angle_diff = std::abs(bar1.rect.angle - bar2.rect.angle);
                        if (angle_diff > 50 && angle_diff < 130) continue; // 允许一定角度差
                        
                        // 高度差筛选
                        float height_diff = std::abs(bar1.rect.size.height - bar2.rect.size.height);
                        if (height_diff > 30) continue;
                        
                        // 绘制装甲板
                        draw_armor(img, bar1, bar2);
                    }
                }
            }

void draw_armor(cv::Mat& img, const LightBar& bar1, const LightBar& bar2) {
    // 确定左右灯条
    const LightBar& left_bar = (bar1.center.x < bar2.center.x) ? bar1 : bar2;
    const LightBar& right_bar = (bar1.center.x < bar2.center.x) ? bar2 : bar1;
    
    // 获取灯条的角点
    cv::Point2f left_points[4], right_points[4];
    left_bar.rect.points(left_points);
    right_bar.rect.points(right_points);
    
    // 对左灯条的点先按y坐标排序，再按x坐标筛选
    std::vector<cv::Point2f> left_sorted_y(left_points, left_points + 4);
    std::sort(left_sorted_y.begin(), left_sorted_y.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });
    
    // 从y排序后的点中，按x坐标找到真正的左上和左下点
    std::vector<cv::Point2f> left_top_candidates = {left_sorted_y[0], left_sorted_y[1]};
    std::sort(left_top_candidates.begin(), left_top_candidates.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    cv::Point2f tl = left_top_candidates[0]; // 左上点（y最小且x最小）
    
    std::vector<cv::Point2f> left_bottom_candidates = {left_sorted_y[2], left_sorted_y[3]};
    std::sort(left_bottom_candidates.begin(), left_bottom_candidates.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    cv::Point2f bl = left_bottom_candidates[0]; // 左下点（y最大且x最小）
    
    // 对右灯条的点先按y坐标排序，再按x坐标筛选
    std::vector<cv::Point2f> right_sorted_y(right_points, right_points + 4);
    std::sort(right_sorted_y.begin(), right_sorted_y.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });
    
    // 从y排序后的点中，按x坐标找到真正的右上和右下点
    std::vector<cv::Point2f> right_top_candidates = {right_sorted_y[0], right_sorted_y[1]};
    std::sort(right_top_candidates.begin(), right_top_candidates.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x > b.x;
    });
    cv::Point2f tr = right_top_candidates[0]; // 右上点（y最小且x最大）
    
    std::vector<cv::Point2f> right_bottom_candidates = {right_sorted_y[2], right_sorted_y[3]};
    std::sort(right_bottom_candidates.begin(), right_bottom_candidates.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x > b.x;
    });
    cv::Point2f br = right_bottom_candidates[0]; // 右下点（y最大且x最大）
    
    // 创建装甲板的四个角点
    std::vector<cv::Point2f> armor_points = {tl, tr, br, bl};
    
    cv::Scalar armor_color = (bar1.color == "Red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    
    // 绘制装甲板四边形
    for (int i = 0; i < 4; i++) {
        cv::line(img, armor_points[i], armor_points[(i + 1) % 4], armor_color, 3);
    }
}
    };
}