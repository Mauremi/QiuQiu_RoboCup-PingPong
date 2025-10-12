#pragma comment(lib, "winmm.lib")
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "visionProto/messages_robocup_ssl_detection.pb.h"
#include "visionProto/messages_robocup_ssl_geometry.pb.h"
#include "visionProto/messages_robocup_ssl_wrapper.pb.h"

#include "gcProto/ssl_gc_referee_message.pb.h"

#include <iostream>
#include "endMotor/MotorIK.h"
#include <windows.h>
#include <time.h>
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/jkerr.h"
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/jktypes.h"
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
#include <chrono>

#include <thread>
#include <mutex>
#include <mmsystem.h>
#include <cmath>
#include <complex>
#include <conio.h> // Windows specific
#include <list>
#include <queue>
#include <vector>
#include <fstream>
#include <Eigen/Dense>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;


#define pi222 3.1415926
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)

#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define TABLE_HEIGHT 760
#define TARGET_RADIUS 50
#define OFFSET_SCALE (TARGET_RADIUS / 2.3)

#define GRAVITY (9800.0 + 1800)
#define BALL_MASS 2.7
#define BALL_RADIUS 20.0

#define AIR_DENSITY 1.2e-9
#define AIR_VISCOSITY 1.8e-4
#define DRAG_COEFF 0.47
#define BALL_AREA (pi222 * BALL_RADIUS * BALL_RADIUS)

/*#define BUCKET_HEIGHT 100*/


enum Logger { Info, Error, Move, Hint, Input };

namespace detail {
    template <typename... Args>
    inline void print_all(std::ostream& os, const Args&... args) {
        int _[] = { 0, ((void)(os << args), 0)... };
        (void)_;
    }
}

template <typename... Args>
void logger(Logger l, const Args&... args) {
    switch (l) {
    case Info:  std::cout << "\t[Info]  "; break;
    case Error: std::cout << "\t\033[0m\033[1;31m[Error] "; break;
    case Move:  std::cout << "\t\033[0m\033[1;32m[Move]  "; break;
    case Input: std::cout << "\t[Input] "; break;
    case Hint:  std::cout << "\t\033[0m\033[1;36m[Hint]  "; break;
    }

    detail::print_all(std::cout, args...);

    switch (l) {
    case Error: case Move: case Hint:
        std::cout << "\033[0m";
        break;
    default:
        break;
    }
    std::cout << '\n';
}


std::map<int, std::pair<double, double>> targets = {
    {1, {TABLE_LENGTH + 380,  620}},
    {2, {TABLE_LENGTH + 380,  500}},
    {3, {TABLE_LENGTH + 380,  380}},
    {4, {TABLE_LENGTH + 380,  260}},
    {5, {TABLE_LENGTH + 380,  140}},
    {6, {TABLE_LENGTH + 500,  500}},
    {7, {TABLE_LENGTH + 500,  380}},
    {8, {TABLE_LENGTH + 500,  260}},
    {9, {TABLE_LENGTH + 620,  380}}
};
// No, x, y, score

struct Point
{
    double x;
    double y;
    double t; // time
};

class TargetChunk
{
public:
    int* target_indices;
    int size;
    Point center;
    TargetChunk(int* indices, int s = 4) : target_indices(indices), size(s)
    {
        for (int i = 0; i < size; i++)
        {
            int index = target_indices[i];
            center.x += targets[index].first;
            center.y += targets[index].second;
        }
        center.x /= size;
        center.y /= size;
    }

    int targets_not_hit(bool score_[]);
};

std::vector<TargetChunk> target_chunks = {
    TargetChunk(new int[4] {2, 3, 6, 7}),
    TargetChunk(new int[4] {3, 4, 7, 8}),
    TargetChunk(new int[4] {1, 2, 3, 6}),
    TargetChunk(new int[4] {2, 3, 4, 7}),
    TargetChunk(new int[4] {3, 4, 5, 8}),
    TargetChunk(new int[4] {6, 7, 8, 9})
};

std::vector<int> neighbors[10] = {
    {},
    {2,6},
    {1,3,6,7},
    {2,4,7},
    {3,5,7,8},
    {4,8},
    {1,2,7,9},
    {2,3,4,6,8,9},
    {4,5,7,9},
    {6,7,8}
};

struct Config {
    // config
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;
    double jaka_angle;
    const int velocity_calculating_method = 2;//or 2
    const bool is_left = false;
    const bool is_yellow = true;


    // DYNAMIC
    // motor speed
    int speed = 740; // Maximum speed for the motor
    const int hitting_threshold = 250; // Threshold ball_x to decide when to hit the ball
    const int jaka_speed_dynamic = 5;
    const double projectile_coefficient = 0.6;
    const double velocity_factor_dynamic = 0.43;
    const double tangent_factor = 1.1 / 7.0;
    const double parameter_e = 0.1;

    // paddle position
    const double paddle_z_left = 225, paddle_z_right = 245; // Paddle heights for left and right sides
    const double table_height_left = 760, table_height_right = 760;
    const double paddle_ball_center_distance = 10; // Distance from paddle center to ball center
    const double jaka_offset_x = 0, jaka_offset_y = -10;
    double moter_speed;

    // trajectory prediction
    const double linear_weight_quadratic = 0.5; // Weight for linear prediction in trajectory prediction
    const double offset_angle_learning_rate = 0.5;

    // IFs
    const bool if_record_trajectory = true; // If record trajectory
    const bool if_vision_correction = false;
    const int if_complex_velocity = 2;


    // STATIC
    // speed
    const double velocity_factor_static = 0.65; // velocity shift factor 35.5 for second targets
    const double jaka_speed_static = 150;
    const int jaka_acceleration = 4000;

    const double mid_velocity = 580;
    const double min_velocity = 470;
    const double max_velocity = 685;

    // Paddle location
    const double paddle_ball_center_hitting_distance = 10; // distance between paddle and ball center. Recommended value is 19 mm, which is the radius of the ball.
    const double paddle_ball_center_preparing_distance = 60;

    const double joint_threshold = 1.0;

    // Adjacent learning
    const double primary_learning_rate = 0.6;
    const double secondary_learning_rate = 0.4;

    // Bounce detection
    const double check_bounce_threshold = 3;
    const double check_bounce_rmse = 7;

    // target choosing
    const int static_target_choosing_method = 1; // 1 for following order, 2 for solid translation with order, 3 for sensible translation with order, 4 for chunk hitting, 5 for chunk hitting and sensible translation
    int order_static_left[10] = { 7, 3, 9, 4, 2, 8, 6, 5, 1 };
    int order_static_right[10] = { 7, 3, 9, 2, 4, 6, 8, 1, 5 };
    int order_dynamic_left[10] = { 7, 4, 2, 8, 6, 3, 5, 9, 1 };
    int order_dynamic_right[10] = { 7, 2, 4, 6, 8, 3, 1, 9, 5 };
} config;

double coming_angle;
double coming_velocity;
double leaving_angle;
double leaving_velocity;//为了计算速度

struct Adjustment
{
    double ball_distance;
    double prepare_distance;
    double prepare_vertical;
    int ball_distance_x_sign;
    int ball_distance_y_sign;
    int prepare_vertical_x_sign;
    int prepare_vertical_y_sign;
};

Point ball_position;
Point landing_position;
Point target;
int target_no;
Point hitting_position;
int balls_n;
std::mutex mtx;
std::ofstream trajectory_outfile;
int hitting_round = 0;
char cmd = 'H';
double hitting_angle = -10;
JointValue jval, jvalH, inijval;
double y_quadratic_predicted_;
bool score[10] = { false };
std::ofstream offset_file;  // 覆盖
int* static_order;
int* dynamic_order;
std::array<std::pair<double, double>, 9> initial_offsets;

bool go_to_left = true; // Flag to determine if the paddle should go to the left or right
bool come_from_left = true;
bool is_hitting = false;
bool is_static = true;

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();   // Function to get vision data
void get_referee();  // Function to get referee data
void getBallLoc();
Point choose_target();


// Linear Predictors for trajectory prediction Class
/*
LinearPredictor(vector<Point>& points): Initializes with a series of points (vector<Point>), and calculates the slope and intercept for linear regression.
predict(double x): Calculates the predicted y value for a given x using the linear equation Y = slope * X + intercept.
*/
class LinearPredictor
{
    double intercept; // Y-intercept of the line
    // Y = slope * X + intercept
public:
    double slope;     // Slope of the line

    LinearPredictor(const std::vector<Point>& points)
    {
        // Based on series of points, calculate the slope and intercept for linear regression
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = points.size();

        for (const auto& point : points)
        {
            sum_x += point.x;
            sum_y += point.y;
            sum_xx += point.x * point.x;
            sum_xy += point.x * point.y;
        }

        // Calculate slope and y-intercept
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        intercept = (sum_y - slope * sum_x) / n;
    }

    LinearPredictor(std::queue<Point>& points)
    {
        // Based on series of points, calculate the slope and intercept for linear regression
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = points.size();

        while (points.size() > 0)
        {
            Point point = points.front();
            points.pop();
            sum_x += point.x;
            sum_y += point.y;
            sum_xx += point.x * point.x;
            sum_xy += point.x * point.y;
        }

        // Calculate slope and y-intercept
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        intercept = (sum_y - slope * sum_x) / n;
    }

    double predict(double x)
    {
        // Predict y value for given x
        return slope * x + intercept;
    }
};

// Quadratic Predictors for trajectory prediction Class
/*
QuadraticPredictor(vector<Point>& points): Initializes with a series of points (vector<Point>), and calculates the coefficients for quadratic regression.
predict(double x): Calculates the predicted y value for a given x using the quadratic equation Y = ax^2 + bx + c.
*/
class QuadraticPredictor
{
    double a;       // Coefficient for x^2
    double b_coeff; // Coefficient for x
    double c;       // Constant term
public:
    QuadraticPredictor(const std::vector<Point> points)
    {
        int n = points.size();

        // Constructing the matrices
        Eigen::MatrixXd A(n, 3); // Design matrix
        Eigen::VectorXd b(n);    // Observations vector

        for (int i = 0; i < n; ++i)
        {
            A(i, 0) = points[i].x * points[i].x; // x^2
            A(i, 1) = points[i].x;               // x
            A(i, 2) = 1;                         // Constant term
            b(i) = points[i].y;                  // y
        }

        // Solve for coefficients using the least squares solution
        // A * coeffs = b => coeffs = (A^T * A)^(-1) * A^T * b
        Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        // Coefficients for y = ax^2 + bx + c
        a = coeffs(0);
        b_coeff = coeffs(1);
        c = coeffs(2);
    }

    double predict(double x)
    {
        // Predict y value for given x using the quadratic formula
        return a * x * x + b_coeff * x + c;
    }
};

class AdjacentLearning
{
public:
    struct Offset
    {
        int target;
        double y_offset;
        double x_offset;
    };

    std::array<Offset, 9> offsets;
    std::unordered_map<int, std::vector<std::pair<int, double>>> neighbor_map_;

    AdjacentLearning() {
        for (int i = 0; i < 9; ++i) {
            offsets[i] = { i + 1, 0.0, 0.0 };
        }

        neighbor_map_ = {
            {1, {{2, 0.75}, {3, 0.5}, {6, 0.1}}},
            {2, {{1, 0.75}, {3, 0.75}, {6, 0.2}, {7, 0.1}}},
            {3, {{1, 0.5}, {2, 0.75}, {4, 0.75}, {5, 0.5}, {6, 0.1}, {7, 0.2}, {8, 0.1}}},
            {4, {{3, 0.75}, {5, 0.75}, {8, 0.2}, {7, 0.1}}},
            {5, {{4, 0.75}, {8, 0.1}}},
            {6, {{1, 0.5}, {2, 0.8}, {3, 0.5}, { 7, 0.75 }, {8, 0.5}, {9, 0.05}}},
            {7, {{2, 0.5}, {3, 0.8}, {4, 0.5}, { 6, 0.75 }, {8, 0.75}, {9, 0.1}}},
            {8, {{5, 0.5}, {4, 0.8}, {3, 0.5}, { 7, 0.75 }, {6, 0.5}, {9, 0.05}}},
            {9, {{6, 0.5}, {7, 0.75}, {8, 0.5}}}
        };
    }
    ~AdjacentLearning() = default;

    void add_offset(const int target, double y_offset, double x_offset) {
        if (y_offset >= 150 || x_offset >= 100)
        {
            if (y_offset >= 250 || x_offset >= 150)
            {
                logger(Error, "Invalid landing point! No offset added.");
                return;
            }
            y_offset /= 2;
            x_offset /= 2;
        }

        offsets[target - 1].y_offset += y_offset * 0.8;
        offsets[target - 1].x_offset += x_offset * 0.8;

        auto it = neighbor_map_.find(target);
        if (it != neighbor_map_.end()) {
            for (const auto& p : it->second) {
                int neighbor = p.first;
                double weight = p.second;
                if (neighbor >= 1 && neighbor <= 9) {
                    offsets[neighbor - 1].y_offset += y_offset * weight;
                    offsets[neighbor - 1].x_offset += x_offset * weight;
                }
            }
        }

        // write csv
        if (offset_file.is_open()) {
            offset_file << target << ","
                << x_offset << ","
                << y_offset << ","
                << "normal" << std::endl;
        }

        logger(Info, "Current offsets (target, y_offset, x_offset): (",
            1, ", ", offsets[0].x_offset, ", ", offsets[0].y_offset, "), (",
            2, ", ", offsets[1].x_offset, ", ", offsets[1].y_offset, "), (",
            3, ", ", offsets[2].x_offset, ", ", offsets[2].y_offset, "), (",
            4, ", ", offsets[3].x_offset, ", ", offsets[3].y_offset, "), (",
            5, ", ", offsets[4].x_offset, ", ", offsets[4].y_offset, "), (",
            6, ", ", offsets[5].x_offset, ", ", offsets[5].y_offset, "), (",
            7, ", ", offsets[6].x_offset, ", ", offsets[6].y_offset, "), (",
            8, ", ", offsets[7].x_offset, ", ", offsets[7].y_offset, "), (",
            9, ", ", offsets[8].x_offset, ", ", offsets[8].y_offset, ").");
    }

    double get_offset_y(const int target) const {
        return offsets[target - 1].y_offset;
    }

    double get_offset_x(const int target) const {
        return offsets[target - 1].x_offset;
    }

} learning;




double calculate_angle(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info, "Calculated leaving angle: ", angle_deg, " degrees.");

    return angle_deg;
}

int TargetChunk::targets_not_hit(bool score_[])
{
    int num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!score_[target_indices[i]]) num++;
    }
    return num;
}

void calculate_score(Point landing_position) {
    double distance;
    int index;
    for (int i = 1; i <= 9; i++) {
        double target_x = targets[i].first;
        double target_y = config.is_left ? targets[i].second : targets[i].second - 760;
        if (i == 1 || distance >= sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2))) {
            distance = sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2));
            index = i;
        }
    }

    if (distance <= 50) {
        if (score[index]) {
            logger(Info, "Target ", index, " already scored.");
        }
        else {
            score[index] = true;
            logger(Info, "Hit target ", index, "!");

            ofstream score_file(string(config.repo_path) + "//national//final//data_score.csv", std::ios::trunc);
            for (int i = 1; i <= 9; i++)
            {
                score_file << score[i] << ",";
            }
            score_file << std::endl;
        }
    }
    else
    {
        logger(Info, "No target hit. Distance to nearest target: ", distance, " mm.");
    }

    logger(Info, "Targets not hit: ", score[1] ? "" : "1 ", score[2] ? "" : "2 ", score[3] ? "" : "3 ",
        score[4] ? "" : "4 ", score[5] ? "" : "5 ", score[6] ? "" : "6 ",
        score[7] ? "" : "7 ", score[8] ? "" : "8 ", score[9] ? "" : "9 ");
}

bool is_valid_offset(std::vector<Point>& full_track) {
    double half_table_length = TABLE_LENGTH / 2.0;

    size_t n = full_track.size();

    size_t cross_idx = n;
    for (size_t i = 0; i < n; ++i) {
        if (full_track[i].x > half_table_length) {
            cross_idx = i; break;
        }
    }

    Point p1 = full_track[cross_idx - 2];
    Point p2 = full_track[cross_idx - 1];
    double vx1 = p2.x - p1.x;
    double vy1 = p2.y - p1.y;

    Point q1 = full_track[cross_idx];
    Point q2 = full_track[cross_idx + 1];
    double vx2 = q2.x - q1.x;
    double vy2 = q2.y - q1.y;

    double dot = vx1 * vx2 + vy1 * vy2;
    double norm1 = sqrt(vx1 * vx1 + vy1 * vy1);
    double norm2 = sqrt(vx2 * vx2 + vy2 * vy2);
    if (norm1 < 1e-6 || norm2 < 1e-6) return true; // 向量过短，无法判定 => 认为有效

    double cos_theta = dot / (norm1 * norm2);
    if (cos_theta > 1.0) cos_theta = 1.0;
    else if (cos_theta < -1.0) cos_theta = -1.0;

    const double PI = std::acos(-1.0);
    double angle_deg = std::acos(cos_theta) * 180.0 / PI;

    logger(Info, "Bounce angle = ", angle_deg, " deg (threshold ", config.check_bounce_threshold, " deg)");

    if (angle_deg > config.check_bounce_threshold) {
        logger(Info, "Invalid offset: angle jump detected.");
        return false;
    }

    // linear regression
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (const auto& pt : full_track) {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_xy += pt.x * pt.y;
        sum_x2 += pt.x * pt.x;
    }
    double a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x + 1e-8);
    double b = (sum_y - a * sum_x) / n;

    double mse = 0;
    for (const auto& pt : full_track) {
        double y_fit = a * pt.x + b;
        mse += (pt.y - y_fit) * (pt.y - y_fit);
    }
    mse /= n;
    double rmse = sqrt(mse);

    logger(Info, "Trajectory RMSE = ", rmse, " (threshold ", config.check_bounce_rmse, ")");

    if (rmse > config.check_bounce_rmse) {
        logger(Info, "Invalid offset: trajectory wave detected.");
        return false;
    }

    return true;
}

void record_trajectory_after() {
    if (!trajectory_outfile.is_open())
    {
        trajectory_outfile.open(string(config.repo_path) + "\\national\\final\\data_trajectory.csv", std::ios::trunc);
        trajectory_outfile << "time,x,y" << std::endl;
    }

    queue<Point> track;  // Vector to store the trajectory points
    std::vector<Point> full_track;

    Point last_ball_pos = { 0, 0, 0 };

    while (fabs(ball_position.x) < 0.1 && fabs(ball_position.y) < 0.1);

    while (true) {
        double ball_x, ball_y, time_stamp;

        std::lock_guard<std::mutex> lock(mtx);  // 使用 std::lock_guard 自动管理锁
        ball_x = ball_position.x;
        ball_y = ball_position.y;
        time_stamp = ball_position.t;

        // Check if the ball is detected
        if (balls_n == 0) {
            Sleep(1000);

            if (balls_n == 0)
            {
                logger(Info, "No ball detected. Recording stop.");
                break;  // No ball detected, stop recording
            }
            ball_x = ball_position.x;
            ball_y = ball_position.y;
            time_stamp = ball_position.t;
        }

        // Check if the ball is within the table boundaries
        if (fabs(ball_x) > TABLE_LENGTH - 150 || fabs(ball_y) > TABLE_WIDTH - 30) {
            logger(Info, "Recording stop.");
            break;  // Automatically stop recording if the ball is out of bounds
        }

        // Check if the ball position has changed significantly
        if (fabs(ball_x - last_ball_pos.x) < 0.5 && fabs(ball_y - last_ball_pos.y) < 0.5) {
            continue;  // Skip writing if the position hasn't changed significantly
        }

        if (config.if_record_trajectory)
        {
            // write the current ball position and timestamp to the CSV file
            trajectory_outfile << time_stamp << "," << ball_x << "," << ball_y << std::endl;
        }

        track.push({ ball_x, ball_y, time_stamp }); // Store the point in the track vector

        if (track.size() == 11)
        {
            track.pop();
        }
        full_track.push_back({ ball_x, ball_y, time_stamp });

        // update the last ball position
        last_ball_pos.x = ball_x;
        last_ball_pos.y = ball_y;
        last_ball_pos.t = time_stamp;
    }

    LinearPredictor predictor(track); // Create a linear predictor based on the recorded trajectory

    double predicted_y = predictor.predict(TABLE_LENGTH); // Predict the y value for the current x position

    trajectory_outfile << 0 << "," << TABLE_LENGTH << "," << predicted_y << std::endl;

    Sleep(3000);

    Point landing_position_ = landing_position;

    logger(Info, "Landing position: (", landing_position_.x, ", ", landing_position_.y, ")");

    double offset_y = target.y - landing_position_.y;
    double offset_x = target.x - landing_position_.x;

    logger(Info, "Offset y: ", offset_y, "; offset x: ", offset_x);

    if (full_track.size() >= 10 && is_static)
    {
        // if not valid offset, close file and return
        if (!is_valid_offset(full_track) || full_track.back().x < 250) {
            logger(Info, "Invalid bounce detected. Offset will not be added.");
        }
        else
        {
            learning.add_offset(target_no, offset_y, offset_x);
        }
    }

    calculate_score(landing_position_);

    trajectory_outfile.close();  // close the file
}



//////////////////////////////////////////////////////////////// DYNAMIC ////////////////////////////////////////////////////////////////////


// Get the current time in microseconds since epoch
uint64_t get_current_time_microseconds()
{
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft); // Get the current system time in FILETIME format

    ULARGE_INTEGER ui;
    ui.LowPart = ft.dwLowDateTime;
    ui.HighPart = ft.dwHighDateTime;

    uint64_t microseconds = (ui.QuadPart - 116444736000000000ULL) / 10;
    return microseconds;
}

double calculate_leaving_angle(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info, "Calculated leaving angle: ", angle_deg, " degrees.");

    return angle_deg;
}

double get_distance(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double sqr = (ball_x - posi_x) * (ball_x - posi_x) + (ball_y - posi_y) * (ball_y - posi_y);
    return sqrt(sqr);
}

// Get the direction of the ball based on its trajectory
double get_ball_coming_direction(std::vector<Point> balls)
{
    if (balls.size() < 2)
    {
        return 0.0;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xy = 0.0;
    double sum_xx = 0.0;

    int n = balls.size();

    for (const auto& point : balls)
    {
        sum_x += point.x;
        sum_y += point.y;
        sum_xy += point.x * point.y;
        sum_xx += point.x * point.x;
    }

    // m is the slope of the line, that is the tangent of the angle
    double m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);

    double angle_radians = std::atan(m);
    double angle_degrees = angle_radians * Rad2Deg;
    return angle_degrees;
}

// Check if the ball is moving toward the robot
// !!! Maybe about to modify
bool is_ball_moving_toward_robot()
{
    std::vector<Point> track;
    int count = 0, maxCount = 4;  // maxCount is the max number of points to check if the ball is moving toward the robot

    for (; count < maxCount; count++)
    {
        // Get the current ball position
        mtx.lock();
        Point curr = ball_position;
        mtx.unlock();

        track.push_back(curr);

        Sleep(1);
    }

    if (track[0].x <= 80 || track[0].x >= 2740) {
        return false;
    }

    if (track[3].y > 0) come_from_left = true;
    else come_from_left = false;

    double directionSum = 0;
    int validCount = 0;

    for (int i = 1; i < track.size(); i++) {
        double dx = track[i].x - track[i - 1].x;
        directionSum += dx;
        validCount++;
    }

    double averageDirection = directionSum / validCount;

    return averageDirection < -5.0;
}

// Collect track data of the ball within a specified range
std::vector<Point> record_trajectory_before(int maxCount, uint64_t& timeStart, uint64_t& timeEnd,
    double xMin, double xMax,
    double yMin = -750.0, double yMax = 750.0)
{
    std::vector<Point> track;
    double lastT = -1.0; // record the last time to avoid duplicates
    int count = 0;

    while (count < maxCount)
    {
        mtx.lock();
        Point curr = ball_position;
        mtx.unlock();

        // x \in [xMin, xMax], y \in [yMin, yMax]
        if (curr.t != lastT &&
            curr.x > xMin && curr.x < xMax &&
            curr.y >= yMin && curr.y <= yMax)
        {
            track.push_back(curr);

            if (count == 0)
                timeStart = get_current_time_microseconds();
            lastT = curr.t;
            count++;
            if (count == maxCount)
                timeEnd = get_current_time_microseconds();

            if (config.if_record_trajectory) trajectory_outfile << curr.t << "," << curr.x << "," << curr.y << std::endl; // Write to CSV file
        }
        else if (curr.t == lastT) {

            continue;
        }
        else if (track.size() > 2) {
            logger(Info, "Point out of range. Stop collecting.");
            break;
        }

        Sleep(5);
    }

    return track;
}

// 计算落点距离
double computeLandingDistance(double velocity) {
    // 初始条件
    double x = 0.0;      // mm
    double z = 0.0;      // mm（桌边为 0，向下为正）
    double vx = velocity; // mm/s
    double vz = 0.0;      // mm/s

    // 常量系数
    double k = 0.5 * AIR_DENSITY * DRAG_COEFF * BALL_AREA;  // g/mm

    double H = TABLE_HEIGHT;
    double DT = 0.4;

    // 积分迭代
    while (z < H) {
        // 速度大小
        double v = std::sqrt(vx * vx + vz * vz);

        // 阻力加速度
        double ax = -(k / BALL_MASS) * v * vx;
        double az = GRAVITY - (k / BALL_MASS) * v * vz;

        // Euler 积分
        vx += ax * DT;
        vz += az * DT;
        x += vx * DT;
        z += vz * DT;
    }

    return x; // 落点水平距离
}

double computeVelocityFromDistance(double distance) {
    double low = 10.0;       // mm/s
    double high = 10000.0;   // mm/s

    for (int i = 0; i < 60; i++) {
        double mid = 0.5 * (low + high);
        double Rmid = computeLandingDistance(mid);
        if (Rmid < distance) {
            low = mid;
        }
        else {
            high = mid;
        }
    }
    return 0.5 * (low + high);
}//辅助方法二计算速度。

double calculate_leaving_velocity(int velocity_calculating_method, double ball_x, double ball_y, double target_x, double target_y) {
    if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01) {
        logger(Error, "No ball detected!");
        return 0; // No speed if no ball detected
    }

    // Calculate the speed based on the distance from the center
    double distance = sqrt(pow(target_x - ball_x, 2) + pow(target_y - ball_y, 2));
    double flight_distance = distance * (target_x - TABLE_LENGTH) / (target_x - ball_x);

    double off_table_y = ball_y + (target_y - ball_y) * (TABLE_LENGTH - ball_x) / (target_x - ball_x);
    double off_table_height = (TABLE_WIDTH / 2 + off_table_y) * (config.table_height_left - config.table_height_right) / TABLE_WIDTH + config.table_height_right;
    if (velocity_calculating_method == 1) {
        double flight_time = sqrt(off_table_height);
        double speed = flight_distance / flight_time;
        double motor_speed = speed * config.velocity_factor_dynamic;

        return motor_speed * config.projectile_coefficient;
    }//平抛计算距离
    if (velocity_calculating_method == 2) {
        double distance = flight_distance;
        double leaving_velocity = computeVelocityFromDistance(distance) * config.velocity_factor_dynamic;

        return leaving_velocity;
    }//空气动力学计算距离
}

double calculate_hitting_angle(double coming_angle, double coming_velocity, double leaving_velocity, double leaving_angle) {

    double numerator = leaving_velocity * std::sin(leaving_angle * Deg2Rad)
        + config.tangent_factor * coming_velocity * std::sin(coming_angle * Deg2Rad);
    double denominator = leaving_velocity * std::cos(leaving_angle * Deg2Rad)
        + config.tangent_factor * coming_velocity * std::cos(coming_angle * Deg2Rad);

    double phi = std::atan2(numerator, denominator) * Rad2Deg;
    return phi;
}//计算击球角度

double calculate_hitting_velocity(double coming_angle, double coming_velocity, double leaving_velocity, double leaving_angle, double e) {

    double phi = calculate_hitting_angle(coming_angle, coming_velocity, leaving_velocity, leaving_angle);
    const double numerator =
        leaving_velocity * std::cos(leaving_angle * Deg2Rad - phi * Deg2Rad)
        - e * coming_velocity * std::cos(phi * Deg2Rad - coming_angle * Deg2Rad);

    const double denominator = 1.0 + e;

    return numerator / denominator; //返回vp
}//计算击球速度

void record_full_trajectory()
{
    vector<Point> full_track(200);
    y_quadratic_predicted_ = 0;
    while (!is_hitting)
    {
        mtx.lock();
        full_track.push_back(ball_position);
        mtx.unlock();

        if (full_track.size() > 50)
        {
            QuadraticPredictor qp(full_track);
            y_quadratic_predicted_ = qp.predict(200);
        }
    }
}

void trajectory_prediction()
{
    errno_t ret;
    uint64_t time_start, time_end;

    if (config.if_record_trajectory) {
        trajectory_outfile.open(string(config.repo_path) + "\\national\\final\\data_trajectory.csv", std::ios::trunc);
        if (!trajectory_outfile.is_open()) {
            logger(Error, "Failed to open trajectory CSV file.");
        }
        trajectory_outfile << "t,x,y\n"; // Write header
    }

    double x = 200;

    std::thread full_trajectory(record_full_trajectory);
    full_trajectory.detach();

    double weight = 1;

    while (!is_hitting)
    {
        std::vector<Point> track = record_trajectory_before(10, time_start, time_end, config.hitting_threshold, 2740);

        coming_angle = get_ball_coming_direction(track);
        double vx = (track.front().x - track.back().x) / (track.back().t - track.front().t);
        double vy = (track.front().y - track.back().y) / (track.back().t - track.front().t);
        coming_velocity = sqrt(vx * vx + vy * vy);

        LinearPredictor lp(track);

        double y_linear_predicted = lp.predict(x);
        double y_quadratic_predicted = y_quadratic_predicted_ ? y_linear_predicted : y_quadratic_predicted_;
        double y_predicted = y_linear_predicted * weight + y_quadratic_predicted * (1 - weight);

        if (weight == 1) weight = config.linear_weight_quadratic;

        go_to_left = (y_predicted > 0); // Determine if the paddle should go to the left or right based on the predicted y value
        hitting_position = { x, y_predicted, 0 };

        JointValue temp;

        if (is_hitting) break;
        mkmotor.goToPos2(go_to_left ? -160 : -20, 1, 400, false);

        leaving_velocity = calculate_leaving_velocity(config.if_complex_velocity, x, y_predicted, target.x, target.y);
        leaving_angle = calculate_leaving_angle(x, y_predicted, target.x, target.y);
        hitting_angle = calculate_hitting_angle(coming_angle, coming_velocity, leaving_velocity, leaving_angle);

        CartesianPose pos;
        pos.tran.x = x - config.paddle_ball_center_distance * cos(hitting_angle * Deg2Rad) + config.jaka_offset_x;
        pos.tran.y = y_predicted - config.paddle_ball_center_distance * sin(hitting_angle * Deg2Rad) + config.jaka_offset_y;
        pos.tran.z = go_to_left ? config.paddle_z_left : config.paddle_z_right;
        pos.rpy.rx = go_to_left ? (-90 + hitting_angle + config.jaka_angle * 0.5) * Deg2Rad : (90 - hitting_angle - config.jaka_angle * 0.5) * Deg2Rad;
        pos.rpy.ry = go_to_left ? -90 * Deg2Rad : 90 * Deg2Rad;
        pos.rpy.rz = 0;
        if (is_hitting) break;
        try
        {
            demo.kine_inverse(&jval, &pos, &jvalH);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed_dynamic, 400 * Deg2Rad, 2, NULL);
            Sleep(200);
        }
        catch (std::runtime_error)
        {
            logger(Error, "Inverse error! Trying to move nearby.");

            if (y_predicted > 300 || (y_predicted > -300 && y_predicted < 0))
            {
                int index = 0;
                while (true)
                {
                    pos.tran.y -= 20;
                    try
                    {
                        demo.kine_inverse(&jval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed_dynamic, 400 * Deg2Rad, 2, NULL);
                        Sleep(200);
                        break; // Break if the inverse kinematics is successful
                    }
                    catch (std::runtime_error)
                    {
                        index++;
                        if (index == 10)
                        {
                            logger(Error, "Inverse error! Cannot find a valid position.");
                            break; // Break if we cannot find a valid position after 10 attempts
                        }
                    }
                }
            }
            else
            {
                int index = 0;
                while (true)
                {
                    pos.tran.y += 20;
                    try
                    {
                        demo.kine_inverse(&jval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed_dynamic, 400 * Deg2Rad, 2, NULL);
                        Sleep(200);
                        break; // Break if the inverse kinematics is successful
                    }
                    catch (std::runtime_error)
                    {
                        index++;
                        if (index == 10)
                        {
                            logger(Error, "Inverse error! Cannot find a valid position.");
                            break; // Break if we cannot find a valid position after 10 attempts
                        }
                    }
                }
            }

        }

        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }

        demo.servo_move_enable(FALSE);
    }
}

void calculate_hit(int round)
{
    logger(Info, "Round ", round, " hitting.");
    is_hitting = false;

    char dued_cmd = config.is_yellow ? 'k' : 'K';

    std::thread t(trajectory_prediction);
    t.detach(); // Start trajectory prediction in a separate thread

    // hitting!
    while (ball_position.x >= config.hitting_threshold && cmd == dued_cmd) {} // Wait for the ball to reach hitting position
    is_hitting = true;
    double speed = calculate_hitting_velocity(coming_angle, coming_velocity, leaving_velocity, leaving_angle, config.parameter_e);
    mkmotor.goToPos2(go_to_left ? -270 : 90, 1, speed, false);
    logger(Info, "Calculated motor speed: ", speed);
    logger(Info, "Motor will hit when ball.x is ", ball_position.x);

    logger(Move, "Motor hitting.");

    std::thread t10(record_trajectory_after);
    t10.detach(); // Start recording trajectory in a separate thread
}



//////////////////////////////////////////////////////////////// STATIC ////////////////////////////////////////////////////////////////////

bool if_reach(double ball_x, double ball_y, JointValue& jvalH, Point& target) {
    double joint0 = jvalH.jVal[0] * 180 / pi222;
    double joint1 = jvalH.jVal[1] * 180 / pi222;
    double joint2 = jvalH.jVal[2] * 180 / pi222;
    double joint3 = jvalH.jVal[3] * 180 / pi222;
    double joint4 = jvalH.jVal[4] * 180 / pi222;
    double joint5 = jvalH.jVal[5] * 180 / pi222;
    logger(Info, joint0);
    logger(Info, joint1);
    logger(Info, joint2);
    logger(Info, joint3);
    logger(Info, joint4);
    logger(Info, joint5);
    if (joint0 == 0 && joint1 == 0 && joint2 == 0 && joint3 == 0 && joint4 == 0 && joint5 == 0)
    {
        logger(Error, "This position can't reach! try to hit target 2!");
        return false;
    }

    if (-360 * config.joint_threshold > joint0 || joint0 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 1 can't reach! try to hit target 2!");
        return false;
    }
    else if (-125 * config.joint_threshold > joint1 || joint1 > 125 * config.joint_threshold)
    {
        logger(Error, "Joint 2 can't reach! try to hit target 2!");
        return false;
    }
    else if (-130 * config.joint_threshold > joint2 || joint2 > 130 * config.joint_threshold)
    {
        logger(Error, "Joint 3 can't reach! try to hit target 2!");
        return false;
    }
    else if (-360 * config.joint_threshold > joint3 || joint3 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 4 can't reach! try to hit target 2!");
        return false;
    }
    else if (-120 * config.joint_threshold > joint4 || joint4 > 120 * config.joint_threshold)
    {
        logger(Error, "Joint 5 can't reach! try to hit target 2!");
        return false;
    }
    else if (-360 * config.joint_threshold > joint5 || joint5 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 6 can't reach! try to hit target 2!");
        return false;
    }
    else {

        return true;
    }
}

double calculate_hitting_angle_static(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info, "Calculated leaving angle: ", angle_deg, " degrees.");

    return angle_deg;
}

// Define the speed based on the ball position
double define_motor_speed(double ball_x, double ball_y, double target_x, double target_y) {
    double y = (target_x - TABLE_LENGTH) * (target_y - ball_y) / (target_x - ball_x);
    double x = target_x - TABLE_LENGTH;
    double distance = sqrt(x * x + y * y);
    double delta = distance - targets[7].first + TABLE_LENGTH;
    double delta_2 = targets[3].first - targets[7].first;
    double delta_1 = targets[9].first - targets[7].first;
    if (delta >= 0)
    {
        double speed = (config.max_velocity - config.mid_velocity) * delta / delta_1 + config.mid_velocity;
        logger(Info, "Motor speed defined: ", speed);
        return speed;
    }
    else
    {
        double speed = (config.min_velocity - config.mid_velocity) * delta / delta_2 + config.mid_velocity;
        logger(Info, "Motor speed defined: ", speed);
        return speed;
    }
}

bool inverse_ensure(int hitting_distance, int preparing_distance, Adjustment& adjust, double& motor_speed, double ball_x, double ball_y, double hitting_angle)
{


    // To be completed: check inverse error.
    //先尝试沿着移动方向偏移，3cm后仍然error，则沿着垂直移动方向偏移至多2cm

    adjust.ball_distance = 0;
    adjust.prepare_distance = 0;
    adjust.prepare_vertical = 0;

    double hitting_angle_rad = hitting_angle * Deg2Rad;



    if (ball_x < 100) //球在下边缘
    {
        adjust.ball_distance_x_sign = 1;
        adjust.ball_distance_y_sign = 1;
        adjust.prepare_vertical_x_sign = ball_y > 0 ? -1 : 1;
        adjust.prepare_vertical_y_sign = ball_y > 0 ? 1 : -1;
        //先确定终点的位置
        bool can_terminal = false;
        for (; adjust.ball_distance <= 10; adjust.ball_distance += 1)
        {
            ball_x += adjust.ball_distance * cos(hitting_angle_rad) * adjust.ball_distance_x_sign;
            ball_y += adjust.ball_distance * sin(hitting_angle_rad) * adjust.ball_distance_y_sign;
            CartesianPose pos_terminal;
            pos_terminal.tran.x = ball_x - hitting_distance * cos(hitting_angle_rad);
            pos_terminal.tran.y = ball_y - hitting_distance * sin(hitting_angle_rad);
            pos_terminal.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_terminal.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = -90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            else {
                pos_terminal.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = 90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_terminal, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                can_terminal = true;
                break;
            }
            catch (std::runtime_error) { continue; }
        }
        if (can_terminal == false)
        {
            logger(Error, "Can't find terminal position!"); return false;
        }
        //确定准备距离
        for (; adjust.prepare_distance <= 30; adjust.prepare_distance += 5)
        {
            preparing_distance -= adjust.prepare_distance;
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad);
            pos_preparing.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad);
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
        }
        for (; adjust.prepare_vertical <= 15; adjust.prepare_vertical += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad) + adjust.prepare_vertical * sin(hitting_angle_rad) * adjust.prepare_vertical_x_sign;
            pos_preparing.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad) + adjust.prepare_vertical * cos(hitting_angle_rad) * adjust.prepare_vertical_y_sign;
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }

        }
        return false;
    }
    if (ball_x >= 200) //球在上边缘
    {
        adjust.ball_distance_x_sign = -1;
        adjust.ball_distance_y_sign = -1;
        adjust.prepare_vertical_x_sign = ball_y > 0 ? 1 : -1;
        adjust.prepare_vertical_y_sign = ball_y > 0 ? -1 : 1;
        //确定调整终点
        bool can_terminal = false;
        for (; adjust.ball_distance <= 10; adjust.ball_distance = +1)
        {
            ball_x += adjust.ball_distance * cos(hitting_angle_rad) * adjust.ball_distance_x_sign;
            ball_y += adjust.ball_distance * sin(hitting_angle_rad) * adjust.ball_distance_y_sign;
            //try to inverse
            CartesianPose pos_terminal;
            pos_terminal.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad);
            pos_terminal.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad);
            pos_terminal.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_terminal.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = -90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            else {
                pos_terminal.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = 90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_terminal, &jvalH_preparing);
                can_terminal = true;
                break;
            }
            catch (std::runtime_error) { continue; }
        }
        if (can_terminal == false)
        {
            logger(Error, "Can't find terminal position!"); return false;
        }
        //确定准备距离
        for (; adjust.prepare_distance <= 20; adjust.prepare_distance += 5)
        {

            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - (preparing_distance - adjust.prepare_distance) * cos(hitting_angle_rad);
            pos_preparing.tran.y = ball_y - (preparing_distance - adjust.prepare_distance) * sin(hitting_angle_rad);
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
        }
        for (; adjust.prepare_vertical <= 15; adjust.prepare_vertical += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad) + adjust.prepare_vertical * sin(hitting_angle_rad) * adjust.prepare_vertical_x_sign;
            pos_preparing.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad) + adjust.prepare_vertical * cos(hitting_angle_rad) * adjust.prepare_vertical_y_sign;
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
        }
        return false;
    }
    if (ball_y > 300) //球在左边缘
    {
        adjust.ball_distance_x_sign = 1;
        adjust.ball_distance_y_sign = -1;
        adjust.prepare_vertical_x_sign = 1;
        adjust.prepare_vertical_y_sign = -1;
        bool can_terminal = false;
        for (; adjust.ball_distance <= 10; adjust.ball_distance += 1)
        {
            ball_x += adjust.ball_distance * cos(hitting_angle_rad) * adjust.ball_distance_x_sign;
            ball_y += adjust.ball_distance * sin(hitting_angle_rad) * adjust.ball_distance_y_sign;
            //try to inverse
            CartesianPose pos_terminal;
            pos_terminal.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad);
            pos_terminal.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad);
            pos_terminal.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_terminal.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = -90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            else {
                pos_terminal.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = 90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_terminal, &jvalH_preparing);
                can_terminal = true;
                break;
            }
            catch (std::runtime_error) { continue; }
        }
        if (can_terminal == false)
        {
            logger(Error, "Can't find terminal position!"); return false;
        }
        for (; adjust.ball_distance <= 30; adjust.ball_distance += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - (preparing_distance - adjust.prepare_distance) * cos(hitting_angle_rad);
            pos_preparing.tran.y = ball_y - (preparing_distance - adjust.prepare_distance) * sin(hitting_angle_rad);
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
        }
        for (; adjust.prepare_vertical <= 15; adjust.prepare_vertical += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad) + adjust.prepare_vertical * sin(hitting_angle_rad) * adjust.prepare_vertical_x_sign;
            pos_preparing.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad) + adjust.prepare_vertical * cos(hitting_angle_rad) * adjust.prepare_vertical_y_sign;
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
        }
        return false;
    }
    if (ball_y < -300) //球在右边缘
    {
        adjust.ball_distance_x_sign = 1;
        adjust.ball_distance_y_sign = 1;
        adjust.prepare_vertical_x_sign = 1;
        adjust.prepare_vertical_y_sign = 1;
        //确定终点位置    
        bool can_terminal = false;
        for (; adjust.ball_distance <= 10; adjust.ball_distance += 1)
        {
            ball_x += adjust.ball_distance * cos(hitting_angle_rad) * adjust.ball_distance_x_sign;
            ball_y += adjust.ball_distance * sin(hitting_angle_rad) * adjust.ball_distance_y_sign;
            //try to inverse
            CartesianPose pos_terminal;
            pos_terminal.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad);
            pos_terminal.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad);
            pos_terminal.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_terminal.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = -90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            else {
                pos_terminal.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_terminal.rpy.ry = 90 * Deg2Rad;
                pos_terminal.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_terminal, &jvalH_preparing);
                can_terminal = true;
                break;
            }
            catch (std::runtime_error) { continue; }
        }
        if (can_terminal == false)
        {
            logger(Error, "Can't find terminal position!"); return false;
        }
        for (; adjust.prepare_distance <= 20; adjust.prepare_distance += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - (preparing_distance - adjust.prepare_distance) * cos(hitting_angle_rad);
            pos_preparing.tran.y = ball_y - (preparing_distance - adjust.prepare_distance) * sin(hitting_angle_rad);
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }

        }
        for (; adjust.prepare_vertical <= 15; adjust.prepare_vertical += 5)
        {
            //try to inverse
            CartesianPose pos_preparing;
            pos_preparing.tran.x = ball_x - preparing_distance * cos(hitting_angle_rad) + adjust.prepare_vertical * sin(hitting_angle_rad) * adjust.prepare_vertical_x_sign;
            pos_preparing.tran.y = ball_y - preparing_distance * sin(hitting_angle_rad) + adjust.prepare_vertical * cos(hitting_angle_rad) * adjust.prepare_vertical_y_sign;
            pos_preparing.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;
            if (ball_y > 0) {
                pos_preparing.rpy.rx = (-90 + hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = -90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            else {
                pos_preparing.rpy.rx = (90 - hitting_angle_rad) * Deg2Rad;
                pos_preparing.rpy.ry = 90 * Deg2Rad;
                pos_preparing.rpy.rz = 0;
            }
            JointValue jvalH_preparing;
            try {
                demo.kine_inverse(&jval, &pos_preparing, &jvalH_preparing);
                logger(Info, "Inverse succeeded after adjusting ball_distance=", adjust.ball_distance, "prepare_distance=", adjust.prepare_distance, "prepare_vertical=", adjust.prepare_vertical, " mm .");
                return true;
            }
            catch (std::runtime_error) { continue; }
            return false;
        }
        return false;
    }

}

void joint_hit(int hitting_distance, double ball_x, double ball_y, double hitting_angle, double motor_speed, Point target)
{
    Adjustment adjust;
    adjust.ball_distance = 0;
    adjust.prepare_distance = 0;
    adjust.prepare_vertical = 0;
    bool can_hit;

    //can_hit = inverse_ensure(hitting_distance, preparing_distance, adjust, motor_speed, ball_x, ball_y, hitting_angle);

    double hitting_angle_rad = hitting_angle * Deg2Rad;



    CartesianPose pos;
    pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad);
    pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad);
    pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

    if (ball_y > 0) {
        pos.rpy.rx = (-90 + hitting_angle + config.jaka_angle) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
    }
    else {
        pos.rpy.rx = (90 - hitting_angle - config.jaka_angle) * Deg2Rad;
        pos.rpy.ry = 90 * Deg2Rad;
        pos.rpy.rz = 0;
    }

    JointValue jvalH;
    try {
        demo.kine_inverse(&jval, &pos, &jvalH);
    }
    catch (std::runtime_error) {
        logger(Error, "Can't inverse hitting position!");
    }

    bool can_reach = if_reach(ball_x, ball_y, jvalH, target);
    if (!can_reach) {
        logger(Error, "This position can't reach! Try to use strange position.");

        return;
    }

    demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

    JointValue jvalH_hitting;

    jvalH_hitting.jVal[0] = jvalH.jVal[0];
    jvalH_hitting.jVal[1] = jvalH.jVal[1];
    jvalH_hitting.jVal[2] = jvalH.jVal[2];
    jvalH_hitting.jVal[3] = jvalH.jVal[3];
    jvalH_hitting.jVal[4] = jvalH.jVal[4];
    jvalH_hitting.jVal[5] = jvalH.jVal[5] + ((ball_y > 0) ? -100.0 : 100.0) * pi222 / 180.0;

    Sleep(500);

    //demo.joint_move(&jvalH_hitting, MoveMode::ABS, false, config.jaka_speed * pi222 / 180);

    Sleep(500);
    mkmotor.goToPos2((ball_y > 0) ? -270 : 90, 0, motor_speed, false); // !DIRECTION

}



//////////////////////////////////////////////////////////////// MAIN ////////////////////////////////////////////////////////////////////

void get_correction_data()
{
    ifstream correction_file(string(config.repo_path) + "\\national\\final\\data_InitialOffsets.csv");

    for (auto& p : initial_offsets) {
        p = { 0.0, 0.0 };
    }

    std::string line;
    while (std::getline(correction_file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        int id;
        double x_offset, y_offset;
        ss >> id;
        ss.ignore(1, ',');
        ss >> x_offset;
        ss.ignore(1, ',');
        ss >> y_offset;

        if (id >= 1 && id <= 9) {
            initial_offsets[id - 1].first += x_offset;
            initial_offsets[id - 1].second += y_offset;
        }
    }

    correction_file.close();

    logger(Info, "[Correction data loaded from CSV]");
    for (int i = 0; i < 9; ++i) {
        logger(Info, "Target ", (i + 1), " -> x_offset: ", initial_offsets[i].first, ", y_offset: ", initial_offsets[i].second);
    }

    ifstream angle_file(string(config.repo_path) + "\\national\\final\\data_AngleCorrection.csv");

    std::getline(angle_file, line);
    std::stringstream ss(line);
    ss >> config.jaka_angle;

    logger(Info, "Jaka angle correction: ", config.jaka_angle, " degrees.");
}

void read_config()
{
    ifstream config_file("config.csv");
    string line;

    getline(config_file, line); // Skip the header line
    getline(config_file, line); // Skip the header line

    stringstream ss(line);
    config.motor_com = 0;
    config.jaka_ip = new char[16];
    config.repo_path = new char[256];
    config.if_referee = false;
    ss >> config.motor_com;
    ss.ignore(1, ','); // Ignore the comma
    ss.getline(config.jaka_ip, 16, ',');
    ss.getline(config.repo_path, 256, ',');
    ss >> config.if_referee;
    // Log the configuration

    logger(Info, "Configuration loaded:",
        "Motor COM:", config.motor_com,
        "; Jaka IP:", config.jaka_ip,
        "; Repo Path:", config.repo_path,
        "; If Referee:", config.if_referee);
}

void read_score()
{
    ifstream score_file(string(config.repo_path) + "//national//final//data_score.csv");
    if (!score_file.is_open()) return;
    string line;
    getline(score_file, line); // Read the first line
    stringstream ss(line);
    for (int i = 1; i <= 9; i++)
    {
        int s;
        ss >> s;
        ss.ignore(1, ','); // Ignore the comma
        score[i] = s;
    }
    logger(Info, "Current score: Target 1 ", score[1] ? "scored" : "none", ", Target 2 ", score[2] ? "scored" : "none", ", Target 3 ", score[3] ? "scored" : "none",
        ", Target 4 ", score[4] ? "scored" : "none", ", Target 5 ", score[5] ? "scored" : "none", ", Target 6 ", score[6] ? "scored" : "none",
        ", Target 7 ", score[7] ? "scored" : "none", ", Target 8 ", score[8] ? "scored" : "none", ", Target 9 ", score[9] ? "scored" : "none");
}

void read_offsets()
{
    ifstream offset_file(string(config.repo_path) + "\\national\\final\\data_offsets.csv");
    if (!offset_file.is_open()) return;
    string line;
    getline(offset_file, line); // Skip the header line
    while (getline(offset_file, line))
    {
        stringstream ss(line);
        double target_no, x_offset, y_offset;
        char point_type;
        ss >> target_no;
        ss.ignore(1, ',');
        ss >> x_offset;
        ss.ignore(1, ',');
        ss >> y_offset;
        ss.ignore(1, ',');
        ss >> point_type;
        learning.add_offset(target_no, y_offset, x_offset);
    }
}

Point choose_target()
{
    if (is_static)
    {
        switch (config.static_target_choosing_method)
        {
        case 1: // order
            for (int i = 0; i < 9; i++)
            {
                if (!score[static_order[i]])
                {
                    logger(Info, "Choose target ", static_order[i], ".");
                    target_no = static_order[i];
                    return { targets[static_order[i]].first, targets[static_order[i]].second - (config.is_left ? 0 : 760), 0 };
                }
            }
        case 2: // solid translation
            for (int i = 0; i < 9; i++)
            {
                if (!score[static_order[i]])
                {
                    logger(Info, "Choose target ", static_order[i], ".");
                    target_no = static_order[i];
                    double offset_x, offset_y;
                    switch (i)
                    {
                    case 2: case 3: case 4:
                        offset_x = 2.0 / 3 * TARGET_RADIUS; break;
                    case 1: case 5:
                        offset_x = 1.0 / 3 * TARGET_RADIUS; break;
                    case 6: case 8: case 7: default:
                        offset_x = 0; break;
                    case 9:
                        offset_x = -2.0 / 3 * TARGET_RADIUS; break;
                    }
                    switch (i)
                    {
                    case 1:
                        offset_y = -2.0 / 3 * TARGET_RADIUS; break;
                    case 5:
                        offset_y = 2.0 / 3 * TARGET_RADIUS; break;
                    case 2: case 6:
                        offset_y = -1.0 / 3 * TARGET_RADIUS; break;
                    case 4: case 8:
                        offset_y = 1.0 / 3 * TARGET_RADIUS; break;
                    case 3: case 7: case 9: default:
                        offset_y = 0; break;
                    }
                    return config.is_left ? Point{ targets[static_order[i]].first + offset_x, targets[static_order[i]].second + offset_y, 0 } :
                        Point{ targets[static_order[i]].first + offset_x, targets[static_order[i]].second - 760 + offset_y, 0 };
                }
            }
        case 3: // sensible translation
            for (int i = 0; i < 9; i++) {
                int id = static_order[i];
                if (!score[id]) {
                    logger(Info, "Choose target ", id, ".");
                    target_no = id;

                    // 计算动态偏移
                    double dx = 0, dy = 0;
                    int count = 0;
                    for (int nb : neighbors[id]) {
                        if (!score[nb]) {
                            dx += targets[nb].first - targets[id].first;
                            dy += targets[nb].second - targets[id].second;
                            count++;
                        }
                    }

                    if (count > 0) {
                        dx /= count;
                        dy /= count;
                        double len = sqrt(dx * dx + dy * dy);
                        if (len > 1e-6) { // 归一化
                            dx = dx / len * OFFSET_SCALE;
                            dy = dy / len * OFFSET_SCALE;
                        }
                    }

                    double target_x = targets[id].first + dx;
                    double target_y = targets[id].second + dy;

                    // 方向镜像（若为左场）
                    if (!config.is_left)
                        target_y -= 760;

                    return Point{ target_x, target_y, 0 };
                }
            }
        case 4: case 5: default:
            int chunk_index = -1;
            int max_num = -1;
            for (int i = 0; i < target_chunks.size(); i++)
            {
                if (target_chunks[i].targets_not_hit(score) > max_num)
                {
                    chunk_index = i;
                    max_num = target_chunks[i].targets_not_hit(score);
                }
            }
            if (config.static_target_choosing_method == 4 || max_num > 2)
                return config.is_left ? target_chunks[chunk_index].center : Point{ target_chunks[chunk_index].center.x, target_chunks[chunk_index].center.y - 760, 0 };
            else
            {
                for (int i = 0; i < 9; i++) {
                    int id = static_order[i];
                    if (!score[id]) {
                        logger(Info, "Choose target ", id, ".");
                        target_no = id;

                        // 计算动态偏移
                        double dx = 0, dy = 0;
                        int count = 0;
                        for (int nb : neighbors[id]) {
                            if (!score[nb]) {
                                dx += targets[nb].first - targets[id].first;
                                dy += targets[nb].second - targets[id].second;
                                count++;
                            }
                        }

                        if (count > 0) {
                            dx /= count;
                            dy /= count;
                            double len = sqrt(dx * dx + dy * dy);
                            if (len > 1e-6) { // 归一化
                                dx = dx / len * OFFSET_SCALE;
                                dy = dy / len * OFFSET_SCALE;
                            }
                        }

                        double target_x = targets[id].first + dx;
                        double target_y = targets[id].second + dy;

                        // 方向镜像（若为左场）
                        if (!config.is_left)
                            target_y -= 760;

                        return Point{ target_x, target_y, 0 };
                    }
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < 9; i++)
        {
            if (!score[dynamic_order[i]])
            {
                logger(Info, "Choose target ", dynamic_order[i], ".");
                target_no = dynamic_order[i];
                double offset_x, offset_y;
                switch (i)
                {
                case 2: case 3: case 4:
                    offset_x = 2.0 / 3 * TARGET_RADIUS; break;
                case 1: case 5:
                    offset_x = 1.0 / 3 * TARGET_RADIUS; break;
                case 6: case 8: case 7: default:
                    offset_x = 0; break;
                case 9:
                    offset_x = -2.0 / 3 * TARGET_RADIUS; break;
                }
                switch (i)
                {
                case 1:
                    offset_y = -2.0 / 3 * TARGET_RADIUS; break;
                case 5:
                    offset_y = 2.0 / 3 * TARGET_RADIUS; break;
                case 2: case 6:
                    offset_y = -1.0 / 3 * TARGET_RADIUS; break;
                case 4: case 8:
                    offset_y = 1.0 / 3 * TARGET_RADIUS; break;
                case 3: case 7: case 9: default:
                    offset_y = 0; break;
                }
                return config.is_left ? Point{ targets[dynamic_order[i]].first + offset_x, targets[dynamic_order[i]].second + offset_y, 0 } :
                    Point{ targets[dynamic_order[i]].first + offset_x, targets[dynamic_order[i]].second - 760 + offset_y, 0 };
            }
        }
    }
}

void static_hit()
{
    logger(Info, "Round ", hitting_round, ". Static hit started.");

    CartesianPose pos;
    double hitting_angle = 0;

    double ball_x, ball_y;
    mtx.lock();
    ball_x = ball_position.x;
    ball_y = ball_position.y;
    mtx.unlock();

    hitting_position = { ball_x, ball_y, 0 };

    logger(Info, "Ball position: (", ball_x, ", ", ball_y, ").");

    target = choose_target();
    double y_offset = learning.get_offset_y(target_no);
    double x_offset = learning.get_offset_x(target_no);
    logger(Info, "Choose target: (", target.x, ", ", target.y, ").");

    Point modified_target = { target.x + x_offset, target.y + y_offset, 0 }; // Apply the offset to the target
    logger(Info, "Target after offset: (", modified_target.x, ", ", modified_target.y, ").");

    // calculate the angle to hit the ball
    hitting_angle = calculate_hitting_angle_static(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
    double hitting_angle_rad = hitting_angle * Deg2Rad;

    double ini_angle = -90;
    mkmotor.goToPos2(ini_angle, 1, 200, false);

    Sleep(200);

    // Decide the speed based on the ball position
    double speed = define_motor_speed(ball_position.x, ball_position.y, modified_target.x, modified_target.y);  // A fake speed, containing jaka and motor

    joint_hit(config.paddle_ball_center_hitting_distance, ball_x, ball_y, hitting_angle, speed, modified_target);

    if (config.if_record_trajectory)
    {
        std::thread t2(record_trajectory_after);
        t2.detach();
    }
}

void dynamic_hit()
{
    logger(Info, "Round ", hitting_round, ". Dynamic hit started.");

    target = choose_target();

    while (!is_ball_moving_toward_robot()) {}
    calculate_hit(hitting_round);
}

int main()
{
    read_config();

    logger(Hint, "Delete history data? (y/n)");
    char yn;
    std::cin >> yn;
    if (yn == 'y')
    {
        offset_file.open(string(config.repo_path) + "\\national\\final\\data_offsets.csv", std::ios::trunc);
        offset_file << "x,y,x_offset,y_offset,point_type" << std::endl;
        ofstream score_file(string(config.repo_path) + "\\national\\final\\data_score.csv", std::ios::trunc);
        score_file << "0,0,0,0,0,0,0,0,0," << std::endl;
    }
    else
    {
        read_offsets();
        offset_file.open(string(config.repo_path) + "\\national\\final\\data_offsets.csv", std::ios::app);
        read_score();
    }

    static_order = config.is_left ? config.order_static_left : config.order_static_right;
    dynamic_order = config.is_left ? config.order_dynamic_left : config.order_dynamic_right;

    logger(Info, "Main loop started.");

    std::thread t1(get_vision);
    t1.detach();
    logger(Info, "Vision system connected.");

    if (config.if_referee)
    {
        std::thread t2(get_referee);
        t2.detach();
        logger(Info, "Referee system connected.");

        std::thread t3(getBallLoc);
        t3.detach();
        logger(Info, "Sandbox system connected.");
    }

    demo.login_in(config.jaka_ip);
    logger(Info, "Jaka mini connected.");

    demo.power_on();
    demo.enable_robot();
    logger(Info, "Jaka mini empowered.");

    mkmotor.ID = 1;
    mkmotor.reduceRatio = 10;
    mkmotor.setSerial(config.motor_com, 115200);
    mkmotor.start();
    logger(Info, "Motor connected.");

    inijval.jVal[0] = 0 * Deg2Rad;
    inijval.jVal[1] = 0 * Deg2Rad;
    inijval.jVal[2] = 90 * Deg2Rad;
    inijval.jVal[3] = 0 * Deg2Rad;
    inijval.jVal[4] = 0 * Deg2Rad;
    inijval.jVal[5] = 0 * Deg2Rad;

    jval = inijval;

    logger(Move, "Jaka mini initializing.");
    demo.joint_move(&inijval, MoveMode::ABS, true, 5);

    logger(Move, "Motor initializing.");
    mkmotor.goToPos2(0, 1, 200, false);

    get_correction_data();

    char dued_cmd;
    if (config.is_yellow) dued_cmd = 'k';
    else dued_cmd = 'K';

    if (!config.if_referee) cmd = dued_cmd;

    while (cmd != 'e') {
        if (cmd != dued_cmd) continue;
        mkmotor.start();
        demo.login_in(config.jaka_ip);
        demo.power_on();
        demo.enable_robot();

        hitting_round++;

        if (balls_n != 0)
        {
            is_static = true;
            static_hit();
        }
        else
        {
            is_static = false;
            dynamic_hit();
        }

        Sleep(1000); // Wait for a while before the next hit
        logger(Move, "Jaka mini and motor returning.");
        mkmotor.goToPos2(0, 1, 200, false);
        demo.joint_move(&inijval, MoveMode::ABS, true, 5);

        mkmotor.stop();
        demo.login_out();

        if (config.if_referee) while (cmd == dued_cmd) {}
    }
}


// Constantly get vision data in a separate thread
void get_vision()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    std::vector<Point> ballP;

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            {
                // Check if the packet contains detection data
                SSL_DetectionFrame detection = packet.detection();
                balls_n = detection.balls_size(); // Get the number of detected balls
                //std::cout << ball.x();
                // Update the ball position
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = ball.x(); // Apply correction
                    pp.y = ball.y(); // Apply correction
                    pp.t = detection.t_capture();
                    mtx.lock();
                    ball_position = pp;
                    mtx.unlock();

                    ballP.push_back(pp);
                }
            }
        }
    }
}

char translate_referee(Referee_Command command)
{

    switch (command)
    {
    case 0:
        cmd = 'H';
        break; // Halt
    case 1:
        cmd = 'S';
        break; // Stop
    case 2:
        cmd = ' ';
        break; // Normal start (Ready)
    case 8:
        cmd = 'k';
        break; // Kickoff Yellow
    case 9:
        cmd = 'K';
        break; // Kickoff Blue
    default:
        std::cout << "refereebox is fail !!!!! command : " << command << std::endl;
        cmd = 'H';
        break;
    }

    return cmd;
}

void get_referee()
{
    int round = 0;
    RoboCupSSLClient client(10003, "224.5.23.1", "");
    client.open(true); // 打开客户端连接
    Referee packet;
    byte datagram[20000];
    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_command())
            { // 检查是否有命令
                Referee_Command next_command = packet.command();
                char cmd = translate_referee(next_command); // 将命令转换为控制字符
                // cout << cmd;
                if (cmd == 'k' && packet.has_yellow())
                {
                    // std::cout << "comand is " << cmd; // 输出命令
                    Referee_TeamInfo team = packet.yellow();
                    // std::cout << "current hit num is " << team.timeouts(); //输出当前的击打数
                    round = team.timeouts();
                }
                if (cmd == 'K' && packet.has_blue())
                {
                    // std::cout << "comand is " << cmd; // 输出命令
                    Referee_TeamInfo team = packet.blue();
                    // std::cout << "current hit num is " << team.timeouts();//输出当前的击打数
                }
            }
            if (packet.has_yellow())
            {
                Referee_TeamInfo team = packet.yellow();
            }
        }
    }
    return;
}

void getBallLoc()
{
    RoboCupSSLClient client(10005, "224.5.23.3", "");
    client.open(true); // 打开客户端连接
    ppBallPos packet;
    while (true) {
        if (client.receive(packet)) {
            if (packet.has_x() && packet.has_y()) {
                landing_position.x = packet.x() + 2740;
                landing_position.y = packet.y();
            }
        }
    }
    return;
}