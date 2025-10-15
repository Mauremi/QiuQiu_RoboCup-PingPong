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

#define GRAVITY 9800.0
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
    {1, {TABLE_LENGTH + 280,  560}},
    {2, {TABLE_LENGTH + 280,  0}},
    {3, {TABLE_LENGTH + 280, -560}},
    {4, {TABLE_LENGTH + 640,  250}},
    {5, {TABLE_LENGTH + 640, -250}},
    {6, {TABLE_LENGTH + 880,  0}}
};
// No, x, y, score

struct Config {
    // config
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;
    const int velocity_calculating_method = 2;//or 2

    // motor speed
    int speed = 740; // Maximum speed for the motor
    const int hitting_threshold = 170; // Threshold ball_x to decide when to hit the ball
    const int jaka_speed = 5;
    const double projectile_coefficient = 1;
    const double velocity_factor = 0.66;
    const double tangent_factor = 8.0 / 7.0;
    const double parameter_e = 0.35;

    // paddle position
    const double paddle_z_left = 235, paddle_z_right = 235; // Paddle heights for left and right sides
    const double table_height_left = 760, table_height_right = 760;
    const double paddle_ball_center_distance = 10; // Distance from paddle center to ball center
    const double jaka_offset_x = 0, jaka_offset_y = -10;
    double jaka_angle_left = 2, jaka_angle_right = 2;
    double moter_speed;

    // trajectory prediction
    const double linear_weight_quadratic = 0.5; // Weight for linear prediction in trajectory prediction
    const double offset_angle_learning_rate = 0.5;

    // IFs
    const bool if_record_trajectory = true; // If record trajectory
    const bool if_vision_correction = false;
    const int if_difficult_velocity = 2;
} config;

double coming_angle;
double coming_velocity;
double leaving_angle;
double leaving_velocity;//为了计算速度

struct Point
{
    double x;
    double y;
    double t; // time
};

Point ball_position;
Point landing_position;
Point target;
Point hitting_position;
int balls_n;
std::mutex mtx;
std::ofstream trajectory_outfile;
int hitting_round = 0;
char cmd = 'H';
double hitting_angle = -10;
JointValue jval, jvalH, inijval;
double y_quadratic_predicted_;
double score = 0;

double ball_y;

bool go_to_left = true; // Flag to determine if the paddle should go to the left or right
bool come_from_left = true;
bool is_hitting = false;

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();   // Function to get vision data
void get_referee();  // Function to get referee data
void getBallLoc();


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

double calculate_angle(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info, "Calculated leaving angle: ", angle_deg, " degrees.");

    return angle_deg;
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

            //logger(Info, "Recorded point ", count, ": (", curr.x, " ", curr.y, ") when ", lastT);

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

double calculate_score(Point landing_position) {
    double distance;
    int index;
    for (int i = 1; i <= 6; i++) {
        double target_x = targets[i].first;
        double target_y = targets[i].second;
        if (i == 1 || distance >= sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2))) {
            distance = sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2));
            index = i;
        }
    }

    double full_score;
    double radius;
    switch (index)
    {
    case 1: case 3: full_score = 1.5; radius = 160; break;
    case 2: full_score = 1; radius = 200; break;
    case 4: case 5: default: full_score = 2.5; radius = 120; break;
    case 6: full_score = 3; radius = 100; break;
    }

    logger(Info, "Close to target ", index);

    double add_score = full_score * (1 - distance / radius);

    return add_score > 0 ? add_score : 0;
}

void record_trajectory_after() {
    queue<Point> track;  // Vector to store the trajectory points

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

        // update the last ball position
        last_ball_pos.x = ball_x;
        last_ball_pos.y = ball_y;
        last_ball_pos.t = time_stamp;
    }

    LinearPredictor predictor(track); // Create a linear predictor based on the recorded trajectory

    double predicted_y = predictor.predict(TABLE_LENGTH); // Predict the y value for the current x position
    double offset_y = (predicted_y > 0) ? 725 : -725;

    trajectory_outfile << 0 << "," << TABLE_LENGTH << "," << predicted_y << std::endl;

    Sleep(3000);

    Point landing_position_ = landing_position;

    logger(Info, "Landing position: (", landing_position_.x, ", ", landing_position_.y, ")");

    double real_angle = calculate_angle(hitting_position.x, hitting_position.y, landing_position_.x, landing_position_.y);
    double wanted_angle = calculate_angle(hitting_position.x, hitting_position.y, target.x, target.y);
    double offset_angle = (wanted_angle - real_angle) * config.offset_angle_learning_rate;
    if (hitting_round <= 2)
    {
        if (hitting_position.y > 0)
        {
            logger(Info, "Left offset angle added ", -offset_angle, " degrees, now is ", config.jaka_angle_left, " degrees.");
            logger(Hint, "Agree? (y/n)");
            char ch;
            cin >> ch;
            if (ch == 'y')
                config.jaka_angle_left -= offset_angle;
        }
        else
        {
            logger(Info, "Right offset angle added ", -offset_angle, " degrees, now is ", config.jaka_angle_right, " degrees.");
            logger(Hint, "Agree? (y/n)");
            char ch;
            cin >> ch;
            if (ch == 'y')
                config.jaka_angle_right -= offset_angle;
        }
    }

    double add_score = calculate_score(landing_position_);
    score += add_score;
    logger(Info, "Score add: ", add_score, ", current score: ", score);

    trajectory_outfile.close();  // close the file
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
        double motor_speed = speed * config.velocity_factor;

        return motor_speed * config.projectile_coefficient;
    }//平抛计算距离
    if (velocity_calculating_method == 2) {
        double distance = flight_distance;
        double leaving_velocity = computeVelocityFromDistance(distance) * config.velocity_factor;

        return leaving_velocity;
    }//空气动力学计算距离
}

double calculate_hitting_angle(double coming_angle, double coming_velocity, double leaving_velocity, double leaving_angle, double ball_y) {

    double numerator = leaving_velocity * std::sin(leaving_angle * Deg2Rad)
        + config.tangent_factor * coming_velocity * std::sin(coming_angle * Deg2Rad);
    double denominator = leaving_velocity * std::cos(leaving_angle * Deg2Rad)
        + config.tangent_factor * coming_velocity * std::cos(coming_angle * Deg2Rad);

    double phi = std::atan2(numerator, denominator) * Rad2Deg;
    return phi - (ball_y > 0 ? config.jaka_angle_left : config.jaka_angle_right);
}//计算击球角度

double calculate_hitting_velocity(double coming_angle, double coming_velocity, double leaving_velocity, double leaving_angle, double e, double ball_y) {

    double phi = calculate_hitting_angle(coming_angle, coming_velocity, leaving_velocity, leaving_angle, ball_y);
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
        trajectory_outfile.open(string(config.repo_path) + "\\national\\dynamic\\data_trajectory.csv", std::ios::trunc);
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
        ball_y = y_predicted;

        if (weight == 1) weight = config.linear_weight_quadratic;

        logger(Info, "Predicted y when x = 200 mm: ", y_predicted, " mm.");

        go_to_left = (y_predicted > 0); // Determine if the paddle should go to the left or right based on the predicted y value
        hitting_position = { x, y_predicted, 0 };

        JointValue temp;

        logger(Move, "Motor preparing.");
        if (is_hitting) break;
        mkmotor.goToPos2(go_to_left ? -160 : -20, 1, 400, false);


        int target_index = come_from_left ? 4 : 5; // !!! ??
        logger(Info, "Choose goal ", target_index, ".");
        target = { targets[target_index].first, targets[target_index].second, 0 };

        leaving_velocity = calculate_leaving_velocity(config.if_difficult_velocity, x, y_predicted, targets[target_index].first, targets[target_index].second);
        leaving_angle = calculate_leaving_angle(x, y_predicted, targets[target_index].first, targets[target_index].second);
        hitting_angle = calculate_hitting_angle(coming_angle, coming_velocity, leaving_velocity, leaving_angle, y_predicted);

        CartesianPose pos;
        pos.tran.x = x - config.paddle_ball_center_distance * cos(hitting_angle * Deg2Rad) + config.jaka_offset_x;
        pos.tran.y = y_predicted - config.paddle_ball_center_distance * sin(hitting_angle * Deg2Rad) + config.jaka_offset_y;
        pos.tran.z = go_to_left ? config.paddle_z_left : config.paddle_z_right;
        pos.rpy.rx = go_to_left ? (-90 + hitting_angle) * Deg2Rad : (90 - hitting_angle) * Deg2Rad;
        pos.rpy.ry = go_to_left ? -90 * Deg2Rad : 90 * Deg2Rad;
        pos.rpy.rz = 0;
        if (is_hitting) break;
        try
        {
            demo.kine_inverse(&inijval, &pos, &jvalH);

            logger(Move, "Jaka mini first adjustment.");
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
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
                        demo.kine_inverse(&inijval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
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
                        demo.kine_inverse(&inijval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
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

    std::thread t(trajectory_prediction);
    t.detach(); // Start trajectory prediction in a separate thread

    // hitting!
    while (ball_position.x >= config.hitting_threshold && cmd == 'k') {} // Wait for the ball to reach hitting position
    is_hitting = true;
    double speed = calculate_hitting_velocity(coming_angle, coming_velocity, leaving_velocity, leaving_angle, config.parameter_e, ball_y);
    mkmotor.goToPos2(go_to_left ? -270 : 90, 1, speed, false);
    logger(Info, "Calculated motor speed: ", speed);
    logger(Info, "Motor will hit when ball.x is ", ball_position.x);

    logger(Move, "Motor hitting.");

    std::thread t10(record_trajectory_after);
    t10.detach(); // Start recording trajectory in a separate thread
}

void get_correction_data()
{
    ifstream angle_file(string(config.repo_path) + "\\national\\dynamic\\data_GoalAngle.csv");

    string line;
    getline(angle_file, line); // Skip the header line
    getline(angle_file, line); // Read the first line

    stringstream ss(line);
    double angle_left, angle_right;
    ss >> angle_left;
    ss.ignore(1, ','); // Ignore the comma
    ss >> angle_right;

    config.jaka_angle_left = angle_left;
    config.jaka_angle_right = angle_right;

    logger(Info, "Jaka mini paddle angle left: ", config.jaka_angle_left, " degrees, right: ", config.jaka_angle_right, " degrees.");
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

int main()
{
    read_config();

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

    logger(Move, "Jaka mini initializing.");
    demo.joint_move(&inijval, MoveMode::ABS, true, 5);

    logger(Move, "Motor initializing.");
    mkmotor.goToPos2(0, 1, 200, false);

    get_correction_data();

    if (!config.if_referee) cmd = 'k';

    while (cmd != 'e') {
        if (cmd != 'k') continue;
        hitting_round++;

        logger(Info, "Round ", hitting_round);
        while (!is_ball_moving_toward_robot()) {}
        calculate_hit(hitting_round);

        Sleep(1000); // Wait for a while before the next hit
        logger(Move, "Jaka mini and motor returning.");
        mkmotor.goToPos2(0, 1, 200, false);
        demo.joint_move(&inijval, MoveMode::ABS, true, 5);

        if (config.if_referee) while (cmd == 'k') {}
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