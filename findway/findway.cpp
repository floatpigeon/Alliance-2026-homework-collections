/*#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MinimumJerkTrajectory {
private:
    vector<vector<double>> path_;
    int num_segments_;
    int poly_order_;
    int coeff_per_segment_;
    
public:
    MinimumJerkTrajectory(const vector<vector<double>>& path) 
        : path_(path), num_segments_(path.size() - 1), poly_order_(4), coeff_per_segment_(5) {}
    
    // 构建Q矩阵（jerk平方的积分）
    Mat buildQMatrix(const vector<double>& segment_times) {
        int total_coeffs = num_segments_ * coeff_per_segment_;
        Mat Q = Mat::zeros(total_coeffs, total_coeffs, CV_64F);
        
        for (int seg = 0; seg < num_segments_; seg++) {
            double T = segment_times[seg];
            int start_idx = seg * coeff_per_segment_;
            
            // jerk平方的积分: ∫(24a₄t + 6a₃)² dt from 0 to T
            Q.at<double>(start_idx + 1, start_idx + 1) += 36 * T;
            Q.at<double>(start_idx, start_idx) += 192 * pow(T, 3);
            Q.at<double>(start_idx, start_idx + 1) += 144 * pow(T, 2);
            Q.at<double>(start_idx + 1, start_idx) += 144 * pow(T, 2);
        }
        
        return Q;
    }
    
    // 构建约束矩阵
    void buildConstraintMatrices(const vector<double>& segment_times, 
                                const vector<double>& waypoints,
                                Mat& A, Mat& b) {
        int total_coeffs = num_segments_ * coeff_per_segment_;
        int num_constraints = 2 * (num_segments_ + 1) + 3 * (num_segments_ - 1);
        A = Mat::zeros(num_constraints, total_coeffs, CV_64F);
        b = Mat::zeros(num_constraints, 1, CV_64F);
        
        int constraint_idx = 0;
        
        // 位置约束
        for (int seg = 0; seg < num_segments_; seg++) {
            double T = segment_times[seg];
            
            // 起点位置约束
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 4) = 1.0;
            b.at<double>(constraint_idx, 0) = waypoints[seg];
            constraint_idx++;
            
            // 终点位置约束
            for (int i = 0; i < coeff_per_segment_; i++) {
                A.at<double>(constraint_idx, seg * coeff_per_segment_ + i) = pow(T, 4 - i);
            }
            b.at<double>(constraint_idx, 0) = waypoints[seg + 1];
            constraint_idx++;
        }
        
        // 连续性约束
        for (int seg = 0; seg < num_segments_ - 1; seg++) {
            double T = segment_times[seg];
            
            // 位置连续
            for (int i = 0; i < coeff_per_segment_; i++) {
                A.at<double>(constraint_idx, seg * coeff_per_segment_ + i) = pow(T, 4 - i);
            }
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 4) = -1.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
            
            // 速度连续
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 0) = 4 * pow(T, 3);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 1) = 3 * pow(T, 2);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 2) = 2 * T;
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 3) = 1.0;
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 3) = -1.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
            
            // 加速度连续
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 0) = 12 * pow(T, 2);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 1) = 6 * T;
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 2) = 2.0;
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 2) = -2.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
        }
    }
    
    // 求解QP问题
    vector<vector<double>> solveQP(const Mat& Q, const Mat& A, const Mat& b) {
        int n = Q.rows;
        int m = A.rows;
        
        Mat KKT = Mat::zeros(n + m, n + m, CV_64F);
        Mat rhs = Mat::zeros(n + m, 1, CV_64F);
        
        Q.copyTo(KKT(Rect(0, 0, n, n)));
        A.copyTo(KKT(Rect(0, n, n, m)));
        Mat A_transpose = A.t();
        A_transpose.copyTo(KKT(Rect(n, 0, m, n)));
        b.copyTo(rhs(Rect(0, n, 1, m)));
        
        Mat solution;
        solve(KKT, rhs, solution, DECOMP_SVD);
        
        vector<vector<double>> all_coeffs(num_segments_);
        for (int seg = 0; seg < num_segments_; seg++) {
            vector<double> coeffs(coeff_per_segment_);
            for (int i = 0; i < coeff_per_segment_; i++) {
                coeffs[i] = solution.at<double>(seg * coeff_per_segment_ + i, 0);
            }
            all_coeffs[seg] = coeffs;
        }
        
        return all_coeffs;
    }
    
    // 计算多项式值和导数
    void evaluatePolynomial(const vector<double>& coeffs, double t, 
                          double& pos, double& vel, double& acc, double& jerk) {
        pos = coeffs[0]*pow(t,4) + coeffs[1]*pow(t,3) + coeffs[2]*pow(t,2) + coeffs[3]*t + coeffs[4];
        vel = 4*coeffs[0]*pow(t,3) + 3*coeffs[1]*pow(t,2) + 2*coeffs[2]*t + coeffs[3];
        acc = 12*coeffs[0]*pow(t,2) + 6*coeffs[1]*t + 2*coeffs[2];
        jerk = 24*coeffs[0]*t + 6*coeffs[1];
    }
    
    // 计算总jerk代价
    double computeTotalJerk(const vector<vector<double>>& all_coeffs,
                           const vector<double>& segment_times) {
        double total_jerk = 0.0;
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& coeffs = all_coeffs[seg];
            double T = segment_times[seg];
            double jerk_cost = 192 * pow(coeffs[0], 2) * pow(T, 3) +
                             144 * coeffs[0] * coeffs[1] * pow(T, 2) +
                             36 * pow(coeffs[1], 2) * T;
            total_jerk += jerk_cost;
        }
        return total_jerk;
    }
    
    void optimize() {
        vector<double> segment_times = {2.0, 2.0, 2.0};
        
        // 提取x坐标和y坐标
        vector<double> x_points, y_points;
        for (const auto& point : path_) {
            x_points.push_back(point[0]);
            y_points.push_back(point[1]);
        }
        
        cout << "=== Minimum Jerk Trajectory Optimization ===" << endl;
        cout << "Path points: ";
        for (const auto& point : path_) {
            cout << "[" << point[0] << "," << point[1] << "] ";
        }
        cout << endl;
        
        // 构建Q矩阵
        Mat Q = buildQMatrix(segment_times);
        
        // 分别优化x轴和y轴
        Mat A_x, b_x, A_y, b_y;
        buildConstraintMatrices(segment_times, x_points, A_x, b_x);
        buildConstraintMatrices(segment_times, y_points, A_y, b_y);
        
        auto x_coefficients = solveQP(Q, A_x, b_x);
        auto y_coefficients = solveQP(Q, A_y, b_y);
        
        double total_jerk_x = computeTotalJerk(x_coefficients, segment_times);
        double total_jerk_y = computeTotalJerk(y_coefficients, segment_times);
        
        cout << "\n=== OPTIMAL SOLUTION ===" << endl;
        cout << "Total jerk cost - X: " << total_jerk_x << ", Y: " << total_jerk_y << endl;
        cout << "Overall jerk cost: " << total_jerk_x + total_jerk_y << endl;
        
        cout << "\n=== X-AXIS POLYNOMIALS ===" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            cout << "T_x" << seg + 1 << "(t) = ";
            cout << x_coefficients[seg][0] << "t^4 + " 
                 << x_coefficients[seg][1] << "t^3 + "
                 << x_coefficients[seg][2] << "t^2 + "
                 << x_coefficients[seg][3] << "t + "
                 << x_coefficients[seg][4] << endl;
        }
        
        cout << "\n=== Y-AXIS POLYNOMIALS ===" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            cout << "T_y" << seg + 1 << "(t) = ";
            cout << y_coefficients[seg][0] << "t^4 + " 
                 << y_coefficients[seg][1] << "t^3 + "
                 << y_coefficients[seg][2] << "t^2 + "
                 << y_coefficients[seg][3] << "t + "
                 << y_coefficients[seg][4] << endl;
        }
        
        // 验证约束
        cout << "\n=== CONSTRAINT VERIFICATION ===" << endl;
        verifyConstraints(x_coefficients, y_coefficients, segment_times);
        
        // 可视化
        visualizeTrajectory(x_coefficients, y_coefficients, segment_times);
    }
    
    void verifyConstraints(const vector<vector<double>>& x_coeffs,
                          const vector<vector<double>>& y_coeffs,
                          const vector<double>& segment_times) {
        cout << "X-axis constraints:" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& coeffs = x_coeffs[seg];
            double T = segment_times[seg];
            
            double pos_start, vel, acc, jerk;
            evaluatePolynomial(coeffs, 0.0, pos_start, vel, acc, jerk);
            double pos_end;
            evaluatePolynomial(coeffs, T, pos_end, vel, acc, jerk);
            
            cout << "  Segment " << seg + 1 << ": " << pos_start << " -> " << pos_end
                 << " (expected: " << path_[seg][0] << " -> " << path_[seg + 1][0] << ")" << endl;
        }
        
        cout << "\nY-axis constraints:" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& coeffs = y_coeffs[seg];
            double T = segment_times[seg];
            
            double pos_start, vel, acc, jerk;
            evaluatePolynomial(coeffs, 0.0, pos_start, vel, acc, jerk);
            double pos_end;
            evaluatePolynomial(coeffs, T, pos_end, vel, acc, jerk);
            
            cout << "  Segment " << seg + 1 << ": " << pos_start << " -> " << pos_end
                 << " (expected: " << path_[seg][1] << " -> " << path_[seg + 1][1] << ")" << endl;
        }
    }
    
    void visualizeTrajectory(const vector<vector<double>>& x_coeffs,
                            const vector<vector<double>>& y_coeffs,
                            const vector<double>& segment_times) {
        vector<double> time_axis, x_axis, y_axis, x_vel, y_vel, x_acc, y_acc;
        double total_time = 0;
        
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& x_coeff = x_coeffs[seg];
            const auto& y_coeff = y_coeffs[seg];
            double T = segment_times[seg];
            
            for (double t = 0; t <= T; t += 0.01) {
                double x_pos, x_v, x_a, x_j;
                double y_pos, y_v, y_a, y_j;
                
                evaluatePolynomial(x_coeff, t, x_pos, x_v, x_a, x_j);
                evaluatePolynomial(y_coeff, t, y_pos, y_v, y_a, y_j);
                
                time_axis.push_back(total_time + t);
                x_axis.push_back(x_pos);
                y_axis.push_back(y_pos);
                x_vel.push_back(x_v);
                y_vel.push_back(y_v);
                x_acc.push_back(x_a);
                y_acc.push_back(y_a);
            }
            total_time += T;
        }
        
        // 创建两个窗口
        Mat traj_image(600, 600, CV_8UC3, Scalar(255, 255, 255));
        Mat states_image(800, 1000, CV_8UC3, Scalar(255, 255, 255));
        
        // 绘制轨迹
        double min_x = 0, max_x = 2, min_y = 0, max_y = 3;
        for (size_t i = 1; i < x_axis.size(); i++) {
            Point p1(50 + (x_axis[i-1] - min_x) / (max_x - min_x) * 500,
                    550 - (y_axis[i-1] - min_y) / (max_y - min_y) * 500);
            Point p2(50 + (x_axis[i] - min_x) / (max_x - min_x) * 500,
                    550 - (y_axis[i] - min_y) / (max_y - min_y) * 500);
            line(traj_image, p1, p2, Scalar(0, 0, 255), 2);
        }
        
        // 标记路径点
        for (const auto& point : path_) {
            Point p(50 + (point[0] - min_x) / (max_x - min_x) * 500,
                   550 - (point[1] - min_y) / (max_y - min_y) * 500);
            circle(traj_image, p, 8, Scalar(0, 0, 0), -1);
            circle(traj_image, p, 8, Scalar(255, 255, 255), 2);
        }
        
        putText(traj_image, "2D Trajectory", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
        
        // 绘制状态图
        int graph_height = 200;
        int margin = 60;
        
        // X位置
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + graph_height - x_axis[i-1] / max_x * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + graph_height - x_axis[i] / max_x * (graph_height - 20));
            line(states_image, p1, p2, Scalar(255, 0, 0), 2);
        }
        
        // Y位置
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 2 * graph_height - y_axis[i-1] / max_y * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 2 * graph_height - y_axis[i] / max_y * (graph_height - 20));
            line(states_image, p1, p2, Scalar(0, 255, 0), 2);
        }
        
        // X速度
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 3 * graph_height - (x_vel[i-1] + 1) / 2 * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 3 * graph_height - (x_vel[i] + 1) / 2 * (graph_height - 20));
            line(states_image, p1, p2, Scalar(0, 0, 255), 2);
        }
        
        // Y速度
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 4 * graph_height - (y_vel[i-1] + 1) / 2 * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 4 * graph_height - (y_vel[i] + 1) / 2 * (graph_height - 20));
            line(states_image, p1, p2, Scalar(255, 0, 255), 2);
        }
        
        putText(states_image, "X Position", Point(10, margin + graph_height/2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        putText(states_image, "Y Position", Point(10, margin + graph_height*1.5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        putText(states_image, "X Velocity", Point(10, margin + graph_height*2.5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        putText(states_image, "Y Velocity", Point(10, margin + graph_height*3.5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        
        imshow("2D Trajectory", traj_image);
        imshow("State Variables", states_image);
        waitKey(0);
    }
};

int main() {
    vector<vector<double>> path = {{0, 0}, {1, 0}, {1, 1}, {1, 2}};
    
    MinimumJerkTrajectory optimizer(path);
    optimizer.optimize();
    
    return 0;
}*/
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 动力学约束
const double MAX_VELOCITY = 1.0;
const double MAX_ACCELERATION = 1.0;

class MinimumJerkTrajectory {
private:
    vector<vector<double>> path_;
    int num_segments_;
    int poly_order_;
    int coeff_per_segment_;
    
public:
    MinimumJerkTrajectory(const vector<vector<double>>& path) 
        : path_(path), num_segments_(path.size() - 1), poly_order_(4), coeff_per_segment_(5) {}
    
    // 构建Q矩阵（jerk平方的积分）
    Mat buildQMatrix(const vector<double>& segment_times) {
        int total_coeffs = num_segments_ * coeff_per_segment_;
        Mat Q = Mat::zeros(total_coeffs, total_coeffs, CV_64F);
        
        for (int seg = 0; seg < num_segments_; seg++) {
            double T = segment_times[seg];
            int start_idx = seg * coeff_per_segment_;
            
            Q.at<double>(start_idx + 1, start_idx + 1) += 36 * T;
            Q.at<double>(start_idx, start_idx) += 192 * pow(T, 3);
            Q.at<double>(start_idx, start_idx + 1) += 144 * pow(T, 2);
            Q.at<double>(start_idx + 1, start_idx) += 144 * pow(T, 2);
        }
        
        return Q;
    }
    
    // 构建约束矩阵
    void buildConstraintMatrices(const vector<double>& segment_times, 
                                const vector<double>& waypoints,
                                Mat& A, Mat& b) {
        int total_coeffs = num_segments_ * coeff_per_segment_;
        int num_constraints = 2 * (num_segments_ + 1) + 3 * (num_segments_ - 1);
        A = Mat::zeros(num_constraints, total_coeffs, CV_64F);
        b = Mat::zeros(num_constraints, 1, CV_64F);
        
        int constraint_idx = 0;
        
        // 位置约束
        for (int seg = 0; seg < num_segments_; seg++) {
            double T = segment_times[seg];
            
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 4) = 1.0;
            b.at<double>(constraint_idx, 0) = waypoints[seg];
            constraint_idx++;
            
            for (int i = 0; i < coeff_per_segment_; i++) {
                A.at<double>(constraint_idx, seg * coeff_per_segment_ + i) = pow(T, 4 - i);
            }
            b.at<double>(constraint_idx, 0) = waypoints[seg + 1];
            constraint_idx++;
        }
        
        // 连续性约束
        for (int seg = 0; seg < num_segments_ - 1; seg++) {
            double T = segment_times[seg];
            
            for (int i = 0; i < coeff_per_segment_; i++) {
                A.at<double>(constraint_idx, seg * coeff_per_segment_ + i) = pow(T, 4 - i);
            }
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 4) = -1.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
            
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 0) = 4 * pow(T, 3);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 1) = 3 * pow(T, 2);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 2) = 2 * T;
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 3) = 1.0;
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 3) = -1.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
            
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 0) = 12 * pow(T, 2);
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 1) = 6 * T;
            A.at<double>(constraint_idx, seg * coeff_per_segment_ + 2) = 2.0;
            A.at<double>(constraint_idx, (seg + 1) * coeff_per_segment_ + 2) = -2.0;
            b.at<double>(constraint_idx, 0) = 0.0;
            constraint_idx++;
        }
    }
    
    // 求解QP问题
    vector<vector<double>> solveQP(const Mat& Q, const Mat& A, const Mat& b) {
        int n = Q.rows;
        int m = A.rows;
        
        Mat KKT = Mat::zeros(n + m, n + m, CV_64F);
        Mat rhs = Mat::zeros(n + m, 1, CV_64F);
        
        Q.copyTo(KKT(Rect(0, 0, n, n)));
        A.copyTo(KKT(Rect(0, n, n, m)));
        Mat A_transpose = A.t();
        A_transpose.copyTo(KKT(Rect(n, 0, m, n)));
        b.copyTo(rhs(Rect(0, n, 1, m)));
        
        Mat solution;
        solve(KKT, rhs, solution, DECOMP_SVD);
        
        vector<vector<double>> all_coeffs(num_segments_);
        for (int seg = 0; seg < num_segments_; seg++) {
            vector<double> coeffs(coeff_per_segment_);
            for (int i = 0; i < coeff_per_segment_; i++) {
                coeffs[i] = solution.at<double>(seg * coeff_per_segment_ + i, 0);
            }
            all_coeffs[seg] = coeffs;
        }
        
        return all_coeffs;
    }
    
    // 计算多项式值和导数
    void evaluatePolynomial(const vector<double>& coeffs, double t, 
                          double& pos, double& vel, double& acc, double& jerk) {
        pos = coeffs[0]*pow(t,4) + coeffs[1]*pow(t,3) + coeffs[2]*pow(t,2) + coeffs[3]*t + coeffs[4];
        vel = 4*coeffs[0]*pow(t,3) + 3*coeffs[1]*pow(t,2) + 2*coeffs[2]*t + coeffs[3];
        acc = 12*coeffs[0]*pow(t,2) + 6*coeffs[1]*t + 2*coeffs[2];
        jerk = 24*coeffs[0]*t + 6*coeffs[1];
    }
    
    // 检查动力学约束
    bool checkDynamicsConstraints(const vector<vector<double>>& x_coeffs,
                                 const vector<vector<double>>& y_coeffs,
                                 const vector<double>& segment_times) {
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& x_coeff = x_coeffs[seg];
            const auto& y_coeff = y_coeffs[seg];
            double T = segment_times[seg];
            
            // 采样检查约束
            for (double t = 0; t <= T; t += 0.05) {
                double x_pos, x_vel, x_acc, x_jerk;
                double y_pos, y_vel, y_acc, y_jerk;
                
                evaluatePolynomial(x_coeff, t, x_pos, x_vel, x_acc, x_jerk);
                evaluatePolynomial(y_coeff, t, y_pos, y_vel, y_acc, y_jerk);
                
                // 计算合速度和合加速度
                double total_vel = sqrt(x_vel*x_vel + y_vel*y_vel);
                double total_acc = sqrt(x_acc*x_acc + y_acc*y_acc);
                
                if (total_vel > MAX_VELOCITY + 1e-6) {
                    return false;
                }
                if (total_acc > MAX_ACCELERATION + 1e-6) {
                    return false;
                }
            }
        }
        return true;
    }
    
    // 计算总jerk代价
    double computeTotalJerk(const vector<vector<double>>& all_coeffs,
                           const vector<double>& segment_times) {
        double total_jerk = 0.0;
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& coeffs = all_coeffs[seg];
            double T = segment_times[seg];
            double jerk_cost = 192 * pow(coeffs[0], 2) * pow(T, 3) +
                             144 * coeffs[0] * coeffs[1] * pow(T, 2) +
                             36 * pow(coeffs[1], 2) * T;
            total_jerk += jerk_cost;
        }
        return total_jerk;
    }
    
    // 时间优化：找到满足动力学约束的最短时间
    void optimizeTime() {
        cout << "=== Minimum Time Trajectory Optimization ===" << endl;
        cout << "Dynamics constraints: Velocity <= " << MAX_VELOCITY 
             << ", Acceleration <= " << MAX_ACCELERATION << endl;
        
        // 提取坐标
        vector<double> x_points, y_points;
        for (const auto& point : path_) {
            x_points.push_back(point[0]);
            y_points.push_back(point[1]);
        }
        
        double best_total_time = 100.0;
        vector<double> best_times;
        vector<vector<double>> best_x_coeffs, best_y_coeffs;
        double best_jerk = 1e10;
        
        // 暴力搜索时间分配（简化版）
        // 在实际应用中应该使用更高效的优化算法如SQP
        for (double base_time = 1.0; base_time <= 5.0; base_time += 0.2) {
            vector<double> times = {base_time, base_time, base_time};
            double total_time = base_time * 3;
            
            if (total_time >= best_total_time) continue;
            
            try {
                Mat Q = buildQMatrix(times);
                Mat A_x, b_x, A_y, b_y;
                buildConstraintMatrices(times, x_points, A_x, b_x);
                buildConstraintMatrices(times, y_points, A_y, b_y);
                
                auto x_coeffs = solveQP(Q, A_x, b_x);
                auto y_coeffs = solveQP(Q, A_y, b_y);
                
                if (checkDynamicsConstraints(x_coeffs, y_coeffs, times)) {
                    double jerk_x = computeTotalJerk(x_coeffs, times);
                    double jerk_y = computeTotalJerk(y_coeffs, times);
                    double total_jerk = jerk_x + jerk_y;
                    
                    if (total_time < best_total_time) {
                        best_total_time = total_time;
                        best_times = times;
                        best_x_coeffs = x_coeffs;
                        best_y_coeffs = y_coeffs;
                        best_jerk = total_jerk;
                        
                        cout << "Found feasible solution: total_time = " << total_time 
                             << ", jerk = " << total_jerk << endl;
                    }
                }
            } catch (...) {
                continue;
            }
        }
        
        if (best_times.empty()) {
            cout << "No feasible solution found within time limits!" << endl;
            return;
        }
        
        // 输出最优解
        cout << "\n=== OPTIMAL SOLUTION ===" << endl;
        cout << "Minimum total time: " << best_total_time << " seconds" << endl;
        cout << "Segment times: ";
        for (double t : best_times) cout << t << " ";
        cout << endl;
        cout << "Total jerk cost: " << best_jerk << endl;
        
        cout << "\n=== X-AXIS POLYNOMIALS ===" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            cout << "T_x" << seg + 1 << "(t) = ";
            cout << best_x_coeffs[seg][0] << "t^4 + " 
                 << best_x_coeffs[seg][1] << "t^3 + "
                 << best_x_coeffs[seg][2] << "t^2 + "
                 << best_x_coeffs[seg][3] << "t + "
                 << best_x_coeffs[seg][4] << endl;
        }
        
        cout << "\n=== Y-AXIS POLYNOMIALS ===" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            cout << "T_y" << seg + 1 << "(t) = ";
            cout << best_y_coeffs[seg][0] << "t^4 + " 
                 << best_y_coeffs[seg][1] << "t^3 + "
                 << best_y_coeffs[seg][2] << "t^2 + "
                 << best_y_coeffs[seg][3] << "t + "
                 << best_y_coeffs[seg][4] << endl;
        }
        
        // 验证约束
        cout << "\n=== CONSTRAINT VERIFICATION ===" << endl;
        verifyConstraints(best_x_coeffs, best_y_coeffs, best_times);
        
        // 可视化
        visualizeTrajectory(best_x_coeffs, best_y_coeffs, best_times);
    }
    
    void verifyConstraints(const vector<vector<double>>& x_coeffs,
                          const vector<vector<double>>& y_coeffs,
                          const vector<double>& segment_times) {
        cout << "Position constraints:" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            double x_start, x_v, x_a, x_j;
            double y_start, y_v, y_a, y_j;
            evaluatePolynomial(x_coeffs[seg], 0.0, x_start, x_v, x_a, x_j);
            evaluatePolynomial(y_coeffs[seg], 0.0, y_start, y_v, y_a, y_j);
            
            cout << "  Segment " << seg + 1 << " start: [" << x_start << "," << y_start 
                 << "] (expected: [" << path_[seg][0] << "," << path_[seg][1] << "])" << endl;
        }
        
        cout << "\nDynamics constraints (max values):" << endl;
        for (int seg = 0; seg < num_segments_; seg++) {
            double max_vel = 0, max_acc = 0;
            double T = segment_times[seg];
            
            for (double t = 0; t <= T; t += 0.01) {
                double x_pos, x_vel, x_acc, x_jerk;
                double y_pos, y_vel, y_acc, y_jerk;
                
                evaluatePolynomial(x_coeffs[seg], t, x_pos, x_vel, x_acc, x_jerk);
                evaluatePolynomial(y_coeffs[seg], t, y_pos, y_vel, y_acc, y_jerk);
                
                double total_vel = sqrt(x_vel*x_vel + y_vel*y_vel);
                double total_acc = sqrt(x_acc*x_acc + y_acc*y_acc);
                
                max_vel = max(max_vel, total_vel);
                max_acc = max(max_acc, total_acc);
            }
            
            cout << "  Segment " << seg + 1 << ": max_vel = " << max_vel 
                 << ", max_acc = " << max_acc << endl;
        }
    }
    
    void visualizeTrajectory(const vector<vector<double>>& x_coeffs,
                            const vector<vector<double>>& y_coeffs,
                            const vector<double>& segment_times) {
        vector<double> time_axis, x_axis, y_axis, vel_axis, acc_axis;
        double total_time = 0;
        
        for (int seg = 0; seg < num_segments_; seg++) {
            const auto& x_coeff = x_coeffs[seg];
            const auto& y_coeff = y_coeffs[seg];
            double T = segment_times[seg];
            
            for (double t = 0; t <= T; t += 0.01) {
                double x_pos, x_vel, x_acc, x_j;
                double y_pos, y_vel, y_acc, y_j;
                
                evaluatePolynomial(x_coeff, t, x_pos, x_vel, x_acc, x_j);
                evaluatePolynomial(y_coeff, t, y_pos, y_vel, y_acc, y_j);
                
                double total_vel = sqrt(x_vel*x_vel + y_vel*y_vel);
                double total_acc = sqrt(x_acc*x_acc + y_acc*y_acc);
                
                time_axis.push_back(total_time + t);
                x_axis.push_back(x_pos);
                y_axis.push_back(y_pos);
                vel_axis.push_back(total_vel);
                acc_axis.push_back(total_acc);
            }
            total_time += T;
        }
        
        // 创建可视化窗口
        Mat traj_image(600, 600, CV_8UC3, Scalar(255, 255, 255));
        Mat states_image(800, 1000, CV_8UC3, Scalar(255, 255, 255));
        
        // 绘制2D轨迹
        double min_x = 0, max_x = 2, min_y = 0, max_y = 3;
        for (size_t i = 1; i < x_axis.size(); i++) {
            Point p1(50 + (x_axis[i-1] - min_x) / (max_x - min_x) * 500,
                    550 - (y_axis[i-1] - min_y) / (max_y - min_y) * 500);
            Point p2(50 + (x_axis[i] - min_x) / (max_x - min_x) * 500,
                    550 - (y_axis[i] - min_y) / (max_y - min_y) * 500);
            line(traj_image, p1, p2, Scalar(0, 0, 255), 2);
        }
        
        // 标记路径点
        for (const auto& point : path_) {
            Point p(50 + (point[0] - min_x) / (max_x - min_x) * 500,
                   550 - (point[1] - min_y) / (max_y - min_y) * 500);
            circle(traj_image, p, 8, Scalar(0, 0, 0), -1);
        }
        
        putText(traj_image, "2D Trajectory (Minimum Time)", Point(10, 30), 
               FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
        
        // 绘制状态图
        int graph_height = 200;
        int margin = 60;
        
        // 速度
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + graph_height - vel_axis[i-1] / MAX_VELOCITY * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + graph_height - vel_axis[i] / MAX_VELOCITY * (graph_height - 20));
            line(states_image, p1, p2, Scalar(0, 255, 0), 2);
        }
        
        // 加速度
        for (size_t i = 1; i < time_axis.size(); i++) {
            Point p1(margin + (time_axis[i-1]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 2 * graph_height - acc_axis[i-1] / MAX_ACCELERATION * (graph_height - 20));
            Point p2(margin + (time_axis[i]) / time_axis.back() * (1000 - 2 * margin),
                    margin + 2 * graph_height - acc_axis[i] / MAX_ACCELERATION * (graph_height - 20));
            line(states_image, p1, p2, Scalar(0, 0, 255), 2);
        }
        
        // 添加约束线
        line(states_image, Point(margin, margin + graph_height), 
             Point(1000 - margin, margin + graph_height), Scalar(0, 0, 0), 1);
        line(states_image, Point(margin, margin + 2 * graph_height), 
             Point(1000 - margin, margin + 2 * graph_height), Scalar(0, 0, 0), 1);
        
        putText(states_image, "Velocity (max=" + to_string(MAX_VELOCITY) + ")", 
               Point(10, margin + graph_height/2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        putText(states_image, "Acceleration (max=" + to_string(MAX_ACCELERATION) + ")", 
               Point(10, margin + graph_height*1.5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0), 1);
        
        imshow("2D Trajectory", traj_image);
        imshow("Dynamics Constraints", states_image);
        waitKey(0);
    }
};

int main() {
    vector<vector<double>> path = {{0, 0}, {1, 0}, {1, 1}, {1, 2}};
    
    MinimumJerkTrajectory optimizer(path);
    optimizer.optimizeTime();
    
    return 0;
}